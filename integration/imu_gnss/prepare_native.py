"""Build an isolated, explicitly patched copy of pinned imu_gnss_fusion.

Only Clone, an f64 anchor getter, and a research moment-injection hook are added.
The original mechanization, updates, mount alignment and gating are unchanged.
"""
from pathlib import Path
import argparse, hashlib, json, shutil, subprocess
REV = '8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6'

def prepare(upstream: Path, output: Path):
    if output.exists(): raise ValueError('Build directory already exists')
    output.mkdir(parents=True)
    shutil.copytree(upstream/'sensor_fusion', output/'sensor_fusion')
    shutil.copytree(Path(__file__).parent/'rust_bridge', output/'bridge')
    original={p.relative_to(upstream).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted((upstream/'sensor_fusion').rglob('*')) if p.is_file()}
    p=output/'sensor_fusion/src/fusion.rs'; s=p.read_text()
    token='#[derive(Debug)]\npub struct SensorFusion {'
    if s.count(token)!=1: raise ValueError('Pinned clone insertion does not match')
    s=s.replace(token,'#[derive(Debug, Clone)]\npub struct SensorFusion {')
    s+='''
// RouteFusion research adapter; not part of the unmodified upstream public API.
impl SensorFusion {
    pub fn research_anchor_f64(&self) -> Option<[f64;3]> {
        self.anchor.valid.then_some([self.anchor.lat_deg,self.anchor.lon_deg,self.anchor.height_m])
    }
    pub fn research_apply_moments(&mut self, dx:[f32;18], p:[[f32;18];18]) -> bool {
        if self.ekf().is_none() {return false;}
        self.ekf.research_apply_moments(dx,p); true
    }
}
'''; p.write_text(s)
    p=output/'sensor_fusion/src/ekf/mod.rs'
    s=p.read_text()+'''
impl Filter {
    /// Research hook: error-coordinate moments, followed by the runtime's own
    /// nominal injection and covariance reset. Caller validates the moments.
    pub fn research_apply_moments(&mut self, dx:[f32;18], p:[[f32;18];18]) {
        self.raw.p=p;
        inject_error_state(&mut self.raw.nominal,&dx);
        apply_reset(&mut self.raw.p,&dx);
    }
}
''';p.write_text(s)
    (output/'Cargo.toml').write_text('[workspace]\nmembers=["sensor_fusion","bridge"]\nresolver="2"\n')
    record={'upstream_repository':'yongkyuns/imu_gnss_fusion','upstream_revision':REV,
            'original_source_sha256':original,'patches':['SensorFusion Clone','f64 anchor accessor','research error-moment injection using upstream injection/reset'],
            'changed_navigation_equations':False}
    (output/'source-provenance.json').write_text(json.dumps(record,indent=2)+'\n')

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--upstream',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();prepare(args.upstream,args.output)
