"""Research-only build of pinned native EKF with exact per-event Jacobian trace.

The original prepare_native.py and its defaults are untouched. This module runs
that same pinned build preparation and applies a narrowly scoped derivative-only
patch to the isolated *copy*. All source anchors must match exactly; fail closed
if upstream code changes. Only transition operators are exported: no Schmidt map
factor or accuracy claim is enabled by this preparatory stage.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path


def replace_once(src: str, old: str, new: str, label: str) -> str:
    n = src.count(old)
    if n != 1:
        raise ValueError(f'{label}: expected one exact source anchor; found {n}')
    return src.replace(old, new, 1)


def patch_region(src: str, start: str, end: str, name: str, edits) -> str:
    if src.count(start) != 1 or src.count(end) != 1:
        raise ValueError(f'{name}: source boundaries changed')
    lo = src.index(start)
    hi = src.index(end, lo+len(start))
    body = src[lo:hi]
    for label, old, new in edits:
        body = replace_once(body, old, new, f'{name}.{label}')
    return src[:lo]+body+src[hi:]


def patch_filter(s: str) -> str:
    s=replace_once(s, 'pub mod state_ops;\nmod types;',
        'pub mod state_ops;\nmod research_transition;\nuse research_transition::ResearchTrace;\nmod types;', 'trace module')
    s=replace_once(s, 'pub struct Filter {\n    raw: State,',
        'pub struct Filter {\n    raw: State,\n    research_trace: ResearchTrace,', 'filter storage')
    s=replace_once(s, '        Self {\n            raw,\n            freeze_misalignment_states:',
        '        Self {\n            raw,\n            research_trace: ResearchTrace::new(),\n            freeze_misalignment_states:', 'filter construction')
    s=replace_once(s,
        '        let (f, g) = self.compute_error_transition_with_noise(imu);\n',
        '        let (f, g) = self.compute_error_transition_with_noise(imu);\n'
        '        self.research_trace.predict(&f);\n', 'exact prediction F')
    s=patch_region(s,
        '    pub fn set_freeze_misalignment_states(',
        '    /// Diagnostic-only scale for direct mount correction',
        'mount freeze', [('freeze',
            '            freeze_mount_covariance(&mut self.raw.p);\n',
            '            freeze_mount_covariance(&mut self.raw.p);\n'
            '            self.research_trace.freeze_mount();\n')])
    s=patch_region(s,
        '    pub fn predict(&mut self, imu: ImuDelta) {',
        '    /// Computes the generated error-state transition matrix',
        'prediction freeze', [('freeze',
            '            freeze_mount_covariance(&mut self.raw.p);\n',
            '            freeze_mount_covariance(&mut self.raw.p);\n'
            '            self.research_trace.freeze_mount();\n')])
    s=patch_region(s,'    fn fuse_body_vel_yz_batch(', '    fn fuse_measurement(',
        'NHC batch', [('accepted H,K',
            '            self.freeze_mount_update_if_needed(&mut diag_k, &mut diag_dx);\n',
            '            self.freeze_mount_update_if_needed(&mut diag_k, &mut diag_dx);\n'
            '            self.research_trace.scalar(&obs.h, &diag_k);\n'),
            ('reset',
            '        apply_reset(&mut self.raw.p, &dx);\n',
            '        apply_reset(&mut self.raw.p, &dx);\n'
            '        self.research_trace.reset(&dx);\n'),
            ('freeze',
            '            freeze_mount_covariance(&mut self.raw.p);\n',
            '            freeze_mount_covariance(&mut self.raw.p);\n'
            '            self.research_trace.freeze_mount();\n')])
    s=patch_region(s,'    fn fuse_measurement(', '    fn fuse_batch(',
        'scalar measurement', [('accepted H,K',
            '        update_covariance_joseph_scalar(&mut self.raw.p, innovation_var, h, k);\n',
            '        self.research_trace.scalar(h, k);\n'
            '        update_covariance_joseph_scalar(&mut self.raw.p, innovation_var, h, k);\n'),
            ('reset',
            '        apply_reset(&mut self.raw.p, dx);\n',
            '        apply_reset(&mut self.raw.p, dx);\n'
            '        self.research_trace.reset(dx);\n'),
            ('freeze',
            '            freeze_mount_covariance(&mut self.raw.p);\n',
            '            freeze_mount_covariance(&mut self.raw.p);\n'
            '            self.research_trace.freeze_mount();\n')])
    s=patch_region(s,'    fn fuse_batch(', '    fn gnss_group_passes_per_axis_sigma_gate(',
        'GNSS batch', [('accepted H,K',
            '            for i in 0..ERROR_STATES {\n                dx[i] += row_dx[i];\n            }\n',
            '            for i in 0..ERROR_STATES {\n                dx[i] += row_dx[i];\n            }\n'
            '            // The special mount gain policy intentionally scales the mean\n'
            '            // but can use an unscaled covariance update. Such epochs\n'
            '            // cannot establish an exact navigation-map sensitivity.\n'
            '            if self.freeze_misalignment_states ||\n'
            '                (matches!(diag_types[row], DIAG_GPS_VEL | DIAG_GPS_VEL_D) &&\n'
            '                 self.gnss_velocity_mount_gain_scale != 1.0) {\n'
            '                self.research_trace.invalidate();\n'
            '            } else { self.research_trace.scalar(h, &k); }\n'),
            ('reset',
            '        apply_reset(&mut self.raw.p, &dx);\n',
            '        apply_reset(&mut self.raw.p, &dx);\n'
            '        self.research_trace.reset(&dx);\n'),
            ('freeze',
            '            freeze_mount_covariance(&mut self.raw.p);\n',
            '            freeze_mount_covariance(&mut self.raw.p);\n'
            '            self.research_trace.freeze_mount();\n')])
    s=replace_once(s,
        '        self.ekf.research_apply_moments(dx,p); true',
        '        self.ekf.research_apply_moments(dx,p); true', 'dummy-no-op') if False else s
    # This append is introduced by the base prepare_native module.
    s=replace_once(s,
        '        apply_reset(&mut self.raw.p,&dx);\n    }\n}',
        '        apply_reset(&mut self.raw.p,&dx);\n'
        '        self.research_trace.reset(&dx);\n    }\n}', 'native moment-injection reset')
    s+='''
// Research-only output; this resets the Jacobian journal after each native event.
impl Filter {
    pub fn research_take_transition(&mut self) -> (bool, u32, [[f32; 18]; 18]) {
        self.research_trace.take()
    }
    pub fn research_rotate_transition(&mut self, r: [[f32; 3]; 3]) {
        self.research_trace.reanchor(r);
    }
    pub fn research_invalidate_transition(&mut self) {
        self.research_trace.invalidate();
    }
}
'''
    return s


def patch_facade(s: str) -> str:
    s=replace_once(s,
        '        self.ekf = crate::ekf::Filter::new(self.cfg.noise.ekf);',
        '        self.ekf = crate::ekf::Filter::new(self.cfg.noise.ekf);\n'
        '        // Existing correlated map state must not survive a reseed.\n'
        '        self.ekf.research_invalidate_transition();', 'reseed detection')
    s=replace_once(s,
        '            rotate_ekf_covariance_nav_blocks(&mut raw.p, r_n1_n0);\n'
        '        }\n\n        if let Some(mut last) = self.last_gnss {',
        '            rotate_ekf_covariance_nav_blocks(&mut raw.p, r_n1_n0);\n'
        '        }\n'
        '        if self.ekf_initialized { self.ekf.research_rotate_transition(r_n1_n0); }\n'
        '\n        if let Some(mut last) = self.last_gnss {', 'exact reanchor transform')
    s+='''
impl SensorFusion {
    pub fn research_take_transition(&mut self) -> (bool, u32, [[f32; 18]; 18]) {
        self.ekf.research_take_transition()
    }
}
'''
    return s


def patch_bridge(s: str) -> str:
    if 'fn rf_transition(' in s:
        raise ValueError('Research trace already patched')
    s+='''
/// Consume the exact operator product accumulated by the pinned runtime.
/// Output: [valid, number_of_internal_operators, F_event(18x18 row major)].
/// Return code is -1 for invalid arguments. `valid=0` marks a native reseed or
/// unsupported update and MUST cause the caller to invalidate map correlation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_transition(p: *mut Bridge, out: *mut f64, len: usize) -> i32 {
    if p.is_null() || out.is_null() || len != 326 { return -1; }
    let (valid, count, f) = unsafe {&mut *p}.fusion.research_take_transition();
    let v = unsafe {core::slice::from_raw_parts_mut(out, len)};
    v[0] = if valid { 1. } else { 0. };
    v[1] = count as f64;
    for i in 0..18 {for j in 0..18 {v[2+i*18+j] = f[i][j] as f64;}}
    0
}
'''
    return s


def instrument(output: Path) -> dict:
    root=Path(output)
    files=[
        ('sensor_fusion/src/ekf/mod.rs', patch_filter),
        ('sensor_fusion/src/fusion.rs', patch_facade),
        ('bridge/src/lib.rs', patch_bridge),
    ]
    changes={}
    for rel,patch in files:
        p=root/rel
        before=p.read_bytes();s=patch(before.decode());p.write_text(s)
        changes[rel]={'original_sha256':hashlib.sha256(before).hexdigest(),
                      'research_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
    module=Path(__file__).parent/'research_trace/research_transition.rs'
    dest=root/'sensor_fusion/src/ekf/research_transition.rs'
    dest.write_bytes(module.read_bytes())
    changes['sensor_fusion/src/ekf/research_transition.rs']={'research_sha256':hashlib.sha256(dest.read_bytes()).hexdigest()}
    prov=root/'source-provenance.json'
    record=json.loads(prov.read_text())
    record['patches'].extend(['opt-in native error transition trace through F, scalar/batch H,K, G reset and reanchor',
                              'reseed and unsupported-mount-policy invalidation',
                              'rf_transition ABI: consume once per sensor event or moment injection'])
    record['research_transitions']=changes
    record['changed_navigation_equations']=False
    prov.write_text(json.dumps(record,indent=2)+'\n')
    return changes


def build(upstream: Path, output: Path):
    from prepare_native import prepare
    prepare(upstream,output)
    return instrument(output)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--upstream',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();print(json.dumps(build(args.upstream,args.output),indent=2))
