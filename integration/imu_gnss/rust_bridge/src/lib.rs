//! Research-only C ABI around the actual pinned SensorFusion runtime.
//! No navigation equations are reimplemented here. All buffers are f64 at the
//! boundary; the original runtime retains its f32 state and arithmetic.
use sensor_fusion::{SensorFusion, ImuSample, GnssSample};

#[derive(Clone)]
pub struct Bridge {
    fusion: SensorFusion,
    last_imu: f64,
    last_gnss: f64,
    events: u32,
}
const SNAPSHOT: usize = 358;

#[unsafe(no_mangle)]
pub extern "C" fn rf_new() -> *mut Bridge {
    Box::into_raw(Box::new(Bridge { fusion: SensorFusion::new(),
        last_imu: f64::NEG_INFINITY, last_gnss: f64::NEG_INFINITY, events: 0 }))
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_free(p: *mut Bridge) {
    if !p.is_null() { drop(unsafe {Box::from_raw(p)}); }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_clone(p: *const Bridge) -> *mut Bridge {
    if p.is_null() {return core::ptr::null_mut();}
    Box::into_raw(Box::new(unsafe {(&*p).clone()}))
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_imu(p: *mut Bridge, input: *const f64, len: usize) -> i32 {
    if p.is_null() || input.is_null() || len != 7 {return -1;}
    let b=unsafe {&mut *p}; let v=unsafe {core::slice::from_raw_parts(input,len)};
    if v.iter().any(|x| !x.is_finite()) || v[0]<=b.last_imu {return -2;}
    let u=b.fusion.process_imu(ImuSample {t_s:v[0] as f32,
        gyro_radps:[v[1] as f32,v[2] as f32,v[3] as f32],
        accel_mps2:[v[4] as f32,v[5] as f32,v[6] as f32]});
    b.last_imu=v[0]; b.events=u.gnss_event_mask; 0
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_gnss(p: *mut Bridge, input: *const f64, len: usize) -> i32 {
    if p.is_null() || input.is_null() || len != 14 {return -1;}
    let b=unsafe {&mut *p}; let v=unsafe {core::slice::from_raw_parts(input,len)};
    if v[..13].iter().any(|x| !x.is_finite()) || v[0]<=b.last_gnss
        || v[7..13].iter().any(|x| *x<0.) {return -2;}
    let u=b.fusion.process_gnss(GnssSample {t_s:v[0] as f32,
        lat_deg:v[1],lon_deg:v[2],height_m:v[3],
        vel_ned_mps:[v[4] as f32,v[5] as f32,v[6] as f32],
        pos_std_m:[v[7] as f32,v[8] as f32,v[9] as f32],
        vel_std_mps:[v[10] as f32,v[11] as f32,v[12] as f32],
        heading_rad:if v[13].is_finite(){Some(v[13] as f32)}else{None}});
    b.last_gnss=v[0]; b.events=u.gnss_event_mask; 0
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_snapshot(p: *const Bridge, output: *mut f64, len: usize) -> i32 {
    if p.is_null() || output.is_null() || len!=SNAPSHOT {return -1;}
    let b=unsafe {&*p}; let o=unsafe {core::slice::from_raw_parts_mut(output,len)};
    o.fill(0.); let h=b.fusion.health();
    o[1]=h.navigation_usable as u8 as f64; o[2]=h.state as u32 as f64;
    o[3]=h.reason_mask as f64; o[4]=b.fusion.reanchor_count() as f64;
    o[5]=b.last_imu; o[6]=b.events as f64;
    o[7]=core::mem::size_of::<Bridge>() as f64;
    if let Some(a)=b.fusion.research_anchor_f64(){o[8..11].copy_from_slice(&a);}
    let Some(s)=b.fusion.ekf() else {return 0;};
    let Some(lla)=b.fusion.position_lla_f64() else {return 0;};
    o[0]=1.; o[11..14].copy_from_slice(&lla);
    let n=&s.nominal;
    let vals=[n.vn,n.ve,n.vd,n.q0,n.q1,n.q2,n.q3,n.q_bv0,n.q_bv1,n.q_bv2,n.q_bv3,
        n.pn,n.pe,n.pd,n.bgx,n.bgy,n.bgz,n.bax,n.bay,n.baz];
    for (i,v) in vals.iter().enumerate(){o[14+i]=*v as f64;}
    for i in 0..18 {for j in 0..18 {o[34+i*18+j]=s.p[i][j] as f64;}}
    0
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_moments(p: *mut Bridge, correction: *const f64,
    covariance: *const f64, n:usize) -> i32 {
    if p.is_null() || correction.is_null() || covariance.is_null() || n!=18 {return -1;}
    let d=unsafe {core::slice::from_raw_parts(correction,18)};
    let q=unsafe {core::slice::from_raw_parts(covariance,324)};
    if d.iter().chain(q).any(|x| !x.is_finite()) {return -2;}
    let mut dx=[0f32;18];let mut pp=[[0f32;18];18];
    for i in 0..18 {dx[i]=d[i] as f32;for j in 0..18 {pp[i][j]=q[18*i+j] as f32;}}
    if unsafe {&mut *p}.fusion.research_apply_moments(dx,pp){0}else{-3}
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn bridge_preserves_direct_runtime() {
        let p=rf_new(); let mut direct=SensorFusion::new();
        for i in 0..400 {
            let t=i as f64/100.;let a=[t,0.,0.,0.,0.,0.,-9.81];
            unsafe {assert_eq!(rf_imu(p,a.as_ptr(),7),0);}
            direct.process_imu(ImuSample{t_s:t as f32,gyro_radps:[0.;3],accel_mps2:[0.,0.,-9.81]});
            assert_eq!(unsafe{&*p}.fusion.health().state,direct.health().state);
        }
        let copy=unsafe{rf_clone(p)};let mut a=[0.;SNAPSHOT];let mut b=[0.;SNAPSHOT];
        unsafe {rf_snapshot(p,a.as_mut_ptr(),SNAPSHOT);rf_snapshot(copy,b.as_mut_ptr(),SNAPSHOT);}
        assert_eq!(a,b);
        unsafe{rf_free(p);rf_free(copy);}
    }
    #[test] fn duplicate_samples_fail_without_mutation() {
        let p=rf_new();let a=[0.,0.,0.,0.,0.,0.,-9.81];
        unsafe{assert_eq!(rf_imu(p,a.as_ptr(),7),0);assert_eq!(rf_imu(p,a.as_ptr(),7),-2);rf_free(p);}
    }
}
