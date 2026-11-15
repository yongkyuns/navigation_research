
// Opt-in research extension. All existing ABI entry points remain unchanged.
#[unsafe(no_mangle)]
pub extern "C" fn rf_new_bias(tau:f64,sigma:f64,initial:f64) -> *mut Bridge {
    let p=rf_new();
    if !unsafe{&mut *p}.fusion.research_enable_bias(tau as f32,sigma as f32,initial as f32) {
        unsafe{rf_free(p);}return core::ptr::null_mut();
    }
    p
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_bias_snapshot(p:*const Bridge, output:*mut f64,len:usize) -> i32 {
    if p.is_null() || output.is_null() || len!=66 {return -1;}
    let b=unsafe{&*p}.fusion.research_bias_snapshot();
    let o=unsafe{core::slice::from_raw_parts_mut(output,len)};
    for i in 0..3 {o[i]=b.mean[i] as f64;for j in 0..3{o[3+i*3+j]=b.covariance[i][j] as f64;}}
    for i in 0..18 {for a in 0..3{o[12+i*3+a]=b.cross[i][a] as f64;}}
    0
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_bias_interval(p:*mut Bridge,h:*const f64,len:usize,alpha:f64,beta:f64) -> i32 {
    if p.is_null() || h.is_null() || len!=18 || !alpha.is_finite() || !beta.is_finite() || beta>0. {return -1;}
    let hh=unsafe{core::slice::from_raw_parts(h,len)};
    if hh.iter().any(|x| !x.is_finite()){return -2;}
    let mut h32=[0f32;18];for i in 0..18{h32[i]=hh[i] as f32;}
    if unsafe{&mut *p}.fusion.research_bias_interval(h32,alpha as f32,beta as f32){0}else{-3}
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rf_activate_bias(p:*mut Bridge,tau:f64,sigma:f64,initial:f64,shock:f64) -> i32 {
    if p.is_null() || [tau,sigma,initial,shock].iter().any(|x| !x.is_finite() || x.abs()>1e10) {return -1;}
    if unsafe{&mut *p}.fusion.research_activate_bias(tau as f32,sigma as f32,initial as f32,shock as f32){0}else{-2}
}
