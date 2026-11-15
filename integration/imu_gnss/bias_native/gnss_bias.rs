//! Research-only correlated receiver-position error model.
//! This module is injected into an isolated pinned upstream build. The original
//! 18-state inertial dynamics are retained; three nuisance states are appended.
use super::*;

pub const BIAS_STATES: usize = 3;

#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub enabled: bool,
    pub tau_s: f32,
    pub stationary_sigma_m: f32,
    pub initial_sigma_m: f32,
}
impl Default for Settings {
    fn default() -> Self { Self {enabled:false, tau_s:90.0, stationary_sigma_m:15.0, initial_sigma_m:3.0} }
}

#[derive(Clone, Debug, Default)]
pub struct Model {
    pub settings: Settings,
    pub mean: [f32; 3],
    pub cross: [[f32; 3]; ERROR_STATES],
    pub covariance: [[f32; 3]; 3],
}
impl Model {
    pub fn enabled(&self) -> bool { self.settings.enabled }
    pub fn position_bias(&self, axis:usize) -> f32 { if self.enabled() {self.mean[axis]} else {0.0} }
    pub fn seed(&mut self, p:&mut [[f32;ERROR_STATES];ERROR_STATES]) {
        if !self.enabled() {return;}
        self.mean=[0.;3]; self.cross=[[0.;3];ERROR_STATES];self.covariance=[[0.;3];3];
        let var=self.settings.initial_sigma_m*self.settings.initial_sigma_m;
        // p_hat=z at initialization, hence position error and receiver bias
        // are ANTI-correlated. The variance of p+b remains the original Pp.
        for a in 0..3 { self.covariance[a][a]=var;self.cross[6+a][a]=-var;p[6+a][6+a]+=var; }
    }
    pub fn predict(&mut self,f:&[[f32;ERROR_STATES];ERROR_STATES],dt:f32) {
        if !self.enabled() {return;}
        let phi=libm::expf(-dt/self.settings.tau_s);
        // expm1 retains the driving variance at small physical IMU intervals.
        let q=self.settings.stationary_sigma_m*self.settings.stationary_sigma_m
            *(-libm::expm1f(-2.*dt/self.settings.tau_s));
        let old=self.cross;
        for i in 0..ERROR_STATES { for a in 0..3 {
            let mut c=0.;for k in 0..ERROR_STATES {c+=f[i][k]*old[k][a];}
            self.cross[i][a]=phi*c;
        }}
        for a in 0..3 {self.mean[a]*=phi;for b in 0..3 {
            self.covariance[a][b]*=phi*phi;
            if a==b {self.covariance[a][b]+=q;}
        }}
    }
    pub fn reset(&mut self,dx:&[f32;ERROR_STATES]) {
        if !self.enabled() {return;}
        // Match the inherited upstream reset convention exactly. Upstream
        // currently resets the attitude block only; mount reset is a known limit.
        let g=generated::attitude_reset_jacobian([dx[0],dx[1],dx[2]]);
        let old=self.cross;
        for i in 0..3 {for a in 0..3 {
            self.cross[i][a]=0.;for k in 0..3 {self.cross[i][a]+=g[i][k]*old[k][a];}
        }}
    }
    pub fn reanchor(&mut self,r:[[f32;3];3]) {
        if !self.enabled(){return;}
        let old=self.clone();
        for a in 0..3 {self.mean[a]=0.;for k in 0..3 {self.mean[a]+=r[a][k]*old.mean[k];}}
        for a in 0..3 {for b in 0..3 {
            let mut x=0.;for i in 0..3 {for j in 0..3 {x+=r[a][i]*old.covariance[i][j]*r[b][j];}}
            self.covariance[a][b]=x;
        }}
        for i in 0..ERROR_STATES {for a in 0..3 {
            let mut x=0.;
            if (3..9).contains(&i) {
                let start=if i<6{3}else{6};
                for k in 0..3 {for b in 0..3 {x+=r[i-start][k]*old.cross[start+k][b]*r[a][b];}}
            } else {for b in 0..3 {x+=old.cross[i][b]*r[a][b];}}
            self.cross[i][a]=x;
        }}
    }
}

impl Filter {
    /// Augmented Gaussian update, using the upstream rows at one linearization.
    /// The original per-group gate is evaluated with the augmented innovation
    /// variance before this method. NHC/velocity rows have zero bias Jacobian.
    pub(super) fn bias_batch(&mut self, count:usize,
        hs:&[[f32;ERROR_STATES];MAX_BATCH_OBS], rs:&[f32;MAX_BATCH_OBS],
        variances:&[f32;MAX_BATCH_OBS], kinds:&[usize;MAX_BATCH_OBS]) {
        self.clear_last_batch_diag();
        if count==0 || count>MAX_BATCH_OBS{return;}
        let mut dx=[0.;ERROR_STATES];let mut db=[0.;3];let mut recorded=0;
        for row in 0..count {
            let h=&hs[row];let mut hb=[0.;3];
            if matches!(kinds[row],DIAG_GPS_POS|DIAG_GPS_POS_D) {
                hb.copy_from_slice(&h[6..9]);
            }
            let mut ux=[0.;ERROR_STATES];let mut ub=[0.;3];
            for i in 0..ERROR_STATES {
                for j in 0..ERROR_STATES {ux[i]+=self.raw.p[i][j]*h[j];}
                for a in 0..3 {ux[i]+=self.gnss_bias.cross[i][a]*hb[a];}
            }
            for a in 0..3 {
                for i in 0..ERROR_STATES {ub[a]+=self.gnss_bias.cross[i][a]*h[i];}
                for b in 0..3 {ub[a]+=self.gnss_bias.covariance[a][b]*hb[b];}
            }
            let mut s=variances[row];let mut residual=rs[row];
            for i in 0..ERROR_STATES {s+=h[i]*ux[i];residual-=h[i]*dx[i];}
            for a in 0..3 {s+=hb[a]*ub[a];residual-=hb[a]*db[a];}
            if !s.is_finite() || s<=0. {continue;}
            let mut k=[0.;ERROR_STATES];let mut rd=[0.;ERROR_STATES];let mut kb=[0.;3];
            for i in 0..ERROR_STATES {k[i]=ux[i]/s;rd[i]=k[i]*residual;}
            if self.freeze_misalignment_states {block_mount_injection(&mut k);block_mount_injection(&mut rd);}
            else if matches!(kinds[row],DIAG_GPS_VEL|DIAG_GPS_VEL_D) {
                scale_mount_entries(&mut k,&mut rd,self.gnss_velocity_mount_gain_scale);
            }
            for a in 0..3 {kb[a]=ub[a]/s;db[a]+=kb[a]*residual;}
            for i in 0..ERROR_STATES {dx[i]+=rd[i];}
            // Full Joseph-equivalent update retains covariance consistency even
            // if the inherited policy deliberately changes mount gain entries.
            for i in 0..ERROR_STATES {for j in i..ERROR_STATES {
                let v=self.raw.p[i][j]-k[i]*ux[j]-ux[i]*k[j]+s*k[i]*k[j];
                self.raw.p[i][j]=v;self.raw.p[j][i]=v;
            }}
            for i in 0..ERROR_STATES {for a in 0..3 {
                self.gnss_bias.cross[i][a]+=-k[i]*ub[a]-ux[i]*kb[a]+s*k[i]*kb[a];
            }}
            for a in 0..3 {for b in a..3 {
                let v=self.gnss_bias.covariance[a][b]-ub[a]*ub[b]/s;
                self.gnss_bias.covariance[a][b]=v;self.gnss_bias.covariance[b][a]=v;
            }}
            self.raw.last_obs_types[recorded]=kinds[row] as i32;
            self.raw.last_residuals[recorded]=rs[row];self.raw.last_effective_residuals[recorded]=residual;
            self.raw.last_innovation_vars[recorded]=s;self.raw.last_h_by_obs[recorded]=*h;
            self.raw.last_k_by_obs[recorded]=k;self.raw.last_dx_by_obs[recorded]=rd;recorded+=1;
            self.record_update_diag(kinds[row],residual,s,h,&k,&rd);
        }
        self.raw.last_dx=dx;self.raw.last_obs_count=recorded as i32;
        self.raw.update_diag.last_dx_mount_roll=dx[15];self.raw.update_diag.last_dx_mount_pitch=dx[16];self.raw.update_diag.last_dx_mount_yaw=dx[17];
        for a in 0..3 {self.gnss_bias.mean[a]+=db[a];}
        inject_error_state(&mut self.raw.nominal,&dx);
        apply_reset(&mut self.raw.p,&dx);self.gnss_bias.reset(&dx);
        if self.freeze_misalignment_states {freeze_mount_covariance(&mut self.raw.p);for i in 15..18 {self.gnss_bias.cross[i]=[0.;3];}}
    }

    /// Ordinary scalar measurement with no direct GNSS-bias term. Preserve the
    /// upstream x gain (including optional gain masking), update bias jointly.
    pub(super) fn bias_scalar(&mut self,s:f32,h:&[f32;ERROR_STATES],k:&[f32;ERROR_STATES],dx:&[f32;ERROR_STATES],residual:f32) {
        if s<=0. || !s.is_finite(){return;}
        let mut ux=[0.;ERROR_STATES];let mut ub=[0.;3];
        for i in 0..ERROR_STATES {for j in 0..ERROR_STATES {ux[i]+=self.raw.p[i][j]*h[j];}}
        for a in 0..3 {for i in 0..ERROR_STATES {ub[a]+=self.gnss_bias.cross[i][a]*h[i];}}
        for a in 0..3 {self.gnss_bias.mean[a]+=ub[a]*residual/s;}
        for i in 0..ERROR_STATES {for a in 0..3 {self.gnss_bias.cross[i][a]-=ux[i]*ub[a]/s;}}
        for a in 0..3 {for b in a..3 {
            let v=self.gnss_bias.covariance[a][b]-ub[a]*ub[b]/s;
            self.gnss_bias.covariance[a][b]=v;self.gnss_bias.covariance[b][a]=v;
        }}
        self.clear_last_batch_diag();
        update_covariance_joseph_scalar(&mut self.raw.p,s,h,k);
        inject_error_state(&mut self.raw.nominal,dx);apply_reset(&mut self.raw.p,dx);self.gnss_bias.reset(dx);
        if self.freeze_misalignment_states {freeze_mount_covariance(&mut self.raw.p);for i in 15..18 {self.gnss_bias.cross[i]=[0.;3];}}
    }
    pub(super) fn bias_nhc(&mut self,ry:f32,rz:f32) {
        let mut hs=[[0.;ERROR_STATES];MAX_BATCH_OBS];let mut residuals=[0.;MAX_BATCH_OBS];let mut vars=[0.;MAX_BATCH_OBS];let mut kinds=[0;MAX_BATCH_OBS];let mut count=0;
        let velocity=nominal_vehicle_velocity(&self.raw.nominal);
        for (axis,r) in [(1,ry),(2,rz)] {
            if !r.is_finite() || r<=0. {continue;}
            let obs=if axis==1 {generated::body_vel_y_observation(&self.raw.nominal,&self.raw.p,r)} else {generated::body_vel_z_observation(&self.raw.nominal,&self.raw.p,r)};
            hs[count]=obs.h;residuals[count]=-velocity[axis];vars[count]=r;kinds[count]=if axis==1{DIAG_BODY_VEL_Y}else{DIAG_BODY_VEL_Z};count+=1;
        }
        self.bias_batch(count,&hs,&residuals,&vars,&kinds);
    }
    /// Rank-one moment transform for a map interval. Both covariance blocks and
    /// their cross-covariance must change, even though the map observes only x.
    pub fn research_bias_interval(&mut self,h:[f32;ERROR_STATES],alpha:f32,beta:f32) {
        let mut ux=[0.;ERROR_STATES];let mut ub=[0.;3];let mut dx=[0.;ERROR_STATES];
        for i in 0..ERROR_STATES {for j in 0..ERROR_STATES {ux[i]+=self.raw.p[i][j]*h[j];}dx[i]=ux[i]*alpha;}
        for a in 0..3 {for i in 0..ERROR_STATES {ub[a]+=self.gnss_bias.cross[i][a]*h[i];}}
        for i in 0..ERROR_STATES {for j in 0..ERROR_STATES {self.raw.p[i][j]+=beta*ux[i]*ux[j];}}
        for i in 0..ERROR_STATES {for a in 0..3 {self.gnss_bias.cross[i][a]+=beta*ux[i]*ub[a];}}
        for a in 0..3 {self.gnss_bias.mean[a]+=alpha*ub[a];for b in 0..3 {self.gnss_bias.covariance[a][b]+=beta*ub[a]*ub[b];}}
        inject_error_state(&mut self.raw.nominal,&dx);apply_reset(&mut self.raw.p,&dx);self.gnss_bias.reset(&dx);
    }
}
