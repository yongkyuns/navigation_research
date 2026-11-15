"""Construct the opt-in 21-state research build; never mutate upstream sources."""
from pathlib import Path
import argparse, hashlib, json, shutil
from prepare_native import prepare

HERE = Path(__file__).parent

def replace(text, old, new, expected=1):
    if text.count(old) != expected:
        raise ValueError(f'Pinned insertion changed ({text.count(old)} != {expected}): {old[:70]}')
    return text.replace(old, new)

def prepare_bias(upstream: Path, output: Path):
    prepare(upstream, output)
    ekf = output/'sensor_fusion/src/ekf/mod.rs'
    s=ekf.read_text()
    s=replace(s, 'mod types;', 'mod types;\npub mod gnss_bias;')
    s=replace(s,'pub struct Filter {\n    raw: State,','pub struct Filter {\n    pub gnss_bias: gnss_bias::Model,\n    raw: State,')
    s=replace(s,'        Self {\n            raw,','        Self {\n            gnss_bias: gnss_bias::Model::default(),\n            raw,')
    marker='    /// Predicts nominal state and covariance from one raw body-frame IMU delta.'
    a,b=s.split(marker)
    idx=a.rfind('    }')
    a=a[:idx]+'        self.gnss_bias.seed(&mut self.raw.p);\n'+a[idx:]
    s=a+marker+b
    s=replace(s,'        let (f, g) = self.compute_error_transition_with_noise(imu);', '        let (f, g) = self.compute_error_transition_with_noise(imu);\n        self.gnss_bias.predict(&f, imu.dt);')
    s=replace(s,'    pub fn fuse_gps(&mut self, sample: GnssSample) -> GnssUpdateResult {', '    pub fn fuse_gps(&mut self, sample: GnssSample) -> GnssUpdateResult {\n        if self.gnss_bias.enabled() {return self.fuse_gps_nhc_batch(sample, None, None);}')
    for axis,name in enumerate(['pn','pe','pd']):
        s=replace(s,f'sample.pos_ned_m[{axis}] - self.raw.nominal.{name},',f'sample.pos_ned_m[{axis}] - self.raw.nominal.{name} - self.gnss_bias.position_bias({axis}),',2)
    s=replace(s,'    fn fuse_body_vel_yz_batch(&mut self, r_body_vel_y: f32, r_body_vel_z: f32) {','    fn fuse_body_vel_yz_batch(&mut self, r_body_vel_y: f32, r_body_vel_z: f32) {\n        if self.gnss_bias.enabled() {self.bias_nhc(r_body_vel_y,r_body_vel_z);return;}')
    s=replace(s,'self.fuse_measurement(obs.s, &obs.h, &k, &dx);','self.fuse_measurement(obs.s, &obs.h, &k, &dx, innovation);',8)
    s=replace(s,'        dx: &[f32; ERROR_STATES],\n    ) {\n        self.clear_last_batch_diag();', '        dx: &[f32; ERROR_STATES],\n        innovation: f32,\n    ) {\n        if self.gnss_bias.enabled() {self.bias_scalar(innovation_var,h,k,dx,innovation);return;}\n        self.clear_last_batch_diag();')
    s=replace(s,'        diag_types: &[usize; MAX_BATCH_OBS],\n    ) {\n        self.clear_last_batch_diag();','        diag_types: &[usize; MAX_BATCH_OBS],\n    ) {\n        if self.gnss_bias.enabled() {self.bias_batch(obs_count,h_rows,residuals,variances,diag_types);return;}\n        self.clear_last_batch_diag();')
    s=replace(s,'        let gate_failed = !gnss_group_passes_per_axis_sigma_gate(\n            &self.raw.p,','        let mut gate_p=self.raw.p;\n        if position_group && self.gnss_bias.enabled() {\n            for a in 0..3 {gate_p[6+a][6+a]+=2.*self.gnss_bias.cross[6+a][a]+self.gnss_bias.covariance[a][a];}\n        }\n        let gate_failed = !gnss_group_passes_per_axis_sigma_gate(\n            &gate_p,')
    s=replace(s,'        self.raw.p=p;\n        inject_error_state', '        assert!(!self.gnss_bias.enabled(), "Use augmented interval update");\n        self.raw.p=p;\n        inject_error_state')
    ekf.write_text(s)
    shutil.copyfile(HERE/'bias_native/gnss_bias.rs',ekf.parent/'gnss_bias.rs')
    fusion=output/'sensor_fusion/src/fusion.rs';s=fusion.read_text()
    s=replace(s,'        self.ekf = crate::ekf::Filter::new(self.cfg.noise.ekf);','        let bias_settings=self.ekf.gnss_bias.settings;\n        self.ekf = crate::ekf::Filter::new(self.cfg.noise.ekf);\n        self.ekf.gnss_bias.settings=bias_settings;')
    s=replace(s,'            rotate_ekf_covariance_nav_blocks(&mut raw.p, r_n1_n0);', '            rotate_ekf_covariance_nav_blocks(&mut raw.p, r_n1_n0);\n            self.ekf.gnss_bias.reanchor(r_n1_n0);')
    s+='''
impl SensorFusion {
    pub fn research_enable_bias(&mut self, tau:f32, sigma:f32, initial:f32) -> bool {
        if self.ekf_initialized || !tau.is_finite() || tau<=0. || !sigma.is_finite() || sigma<0. || !initial.is_finite() || initial<0. {return false;}
        self.ekf.gnss_bias.settings=crate::ekf::gnss_bias::Settings{enabled:true,tau_s:tau,stationary_sigma_m:sigma,initial_sigma_m:initial};true
    }
    pub fn research_bias_snapshot(&self) -> &crate::ekf::gnss_bias::Model {&self.ekf.gnss_bias}
    pub fn research_bias_interval(&mut self,h:[f32;18],alpha:f32,beta:f32) -> bool {
        if self.ekf().is_none() || !self.ekf.gnss_bias.enabled(){return false;}
        self.ekf.research_bias_interval(h,alpha,beta);true
    }
}
'''
    fusion.write_text(s)
    bridge=output/'bridge/src/lib.rs'
    bridge.write_text(bridge.read_text()+(HERE/'bias_native/bridge_extra.rs').read_text())
    (output/'bias-patch-provenance.json').write_text(json.dumps({
        'base_upstream':'8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6',
        'model':'18 inertial error states + 3 correlated GNSS-position nuisance states',
        'default_enabled':False,
        'added_sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (HERE/'bias_native').glob('*.rs')},
        'patched_sources':{str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [fusion,ekf,bridge]},
        'reference_data_used':False},indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--upstream',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepare_bias(a.upstream,a.output)
