#[cfg(test)]
mod bias_tests {
    use super::*;
    fn model() -> Model {Model{settings:Settings{enabled:true,..Settings::default()},..Model::default()}}
    fn filter() -> Filter {
        let mut f=Filter::new(ProcessNoise::lsm6dso_104hz());f.gnss_bias=model();
        f.raw.p=[[0.;ERROR_STATES];ERROR_STATES];
        for i in 0..ERROR_STATES{f.raw.p[i][i]=1.;}
        f.raw.p[6][6]=4.;f.gnss_bias.cross[6][0]=2.;f.gnss_bias.covariance[0][0]=9.;
        f
    }
    fn approx(a:f32,b:f32){assert!((a-b).abs()<2e-4,"{a} != {b}");}
    #[test]fn seed_preserves_observable_sum_and_unknown_bias(){
        let mut m=model();let mut p=[[0.;ERROR_STATES];ERROR_STATES];p[6][6]=4.;m.seed(&mut p);
        approx(p[6][6]+2.*m.cross[6][0]+m.covariance[0][0],4.);
        approx(p[6][6],13.);approx(m.cross[6][0],-9.);
    }
    #[test]fn exact_gauss_markov_discretization_and_cross_prediction(){
        let mut m=model();m.mean=[2.,-3.,4.];m.cross[6][0]=2.;m.covariance[0][0]=7.;
        let mut f=[[0.;ERROR_STATES];ERROR_STATES];for i in 0..ERROR_STATES{f[i][i]=1.;}f[7][6]=0.25;
        m.predict(&f,1.);let phi=libm::expf(-1./90.);
        approx(m.mean[0],2.*phi);approx(m.cross[6][0],2.*phi);approx(m.cross[7][0],0.5*phi);
        approx(m.covariance[0][0],phi*phi*7.+225.*(1.-phi*phi));
    }
    #[test]fn joint_gnss_update_matches_closed_form(){
        let mut f=filter();let mut h=[[0.;ERROR_STATES];MAX_BATCH_OBS];h[0][6]=1.;
        let mut r=[0.;MAX_BATCH_OBS];r[0]=18.;let mut v=[0.;MAX_BATCH_OBS];v[0]=1.;let mut k=[0;MAX_BATCH_OBS];k[0]=DIAG_GPS_POS;
        f.bias_batch(1,&h,&r,&v,&k);
        approx(f.raw.nominal.pn,6.);approx(f.gnss_bias.mean[0],11.);
        approx(f.raw.p[6][6],2.);approx(f.gnss_bias.cross[6][0],2.-66./18.);
        approx(f.gnss_bias.covariance[0][0],9.-121./18.);
    }
    #[test]fn ordinary_observation_updates_nuisance_cross_covariance(){
        let mut f=filter();let mut h=[0.;ERROR_STATES];h[6]=1.;let mut k=[0.;ERROR_STATES];k[6]=0.8;let mut dx=[0.;ERROR_STATES];dx[6]=4.;
        f.bias_scalar(5.,&h,&k,&dx,5.);
        approx(f.raw.nominal.pn,4.);approx(f.gnss_bias.mean[0],2.);
        approx(f.raw.p[6][6],0.8);approx(f.gnss_bias.cross[6][0],0.4);approx(f.gnss_bias.covariance[0][0],8.2);
    }
    #[test]fn masked_gain_keeps_joint_joseph_covariance(){
        let mut f=filter();let mut h=[0.;ERROR_STATES];h[6]=1.;
        f.bias_scalar(5.,&h,&[0.;ERROR_STATES],&[0.;ERROR_STATES],5.);
        approx(f.raw.nominal.pn,0.);approx(f.raw.p[6][6],4.);
        approx(f.gnss_bias.mean[0],2.);approx(f.gnss_bias.cross[6][0],0.4);approx(f.gnss_bias.covariance[0][0],8.2);
    }
    #[test]fn interval_conditions_bias_without_observing_it_directly(){
        let mut f=filter();let mut h=[0.;ERROR_STATES];h[6]=1.;f.research_bias_interval(h,0.3,-0.1);
        approx(f.raw.nominal.pn,1.2);approx(f.gnss_bias.mean[0],0.6);
        approx(f.raw.p[6][6],2.4);approx(f.gnss_bias.cross[6][0],1.2);approx(f.gnss_bias.covariance[0][0],8.6);
    }
    #[test]fn reanchor_rotates_both_cross_covariance_sides(){
        let mut m=model();m.mean=[2.,3.,4.];m.covariance[0][0]=5.;m.covariance[1][1]=7.;m.covariance[2][2]=9.;m.cross[6][0]=2.;m.cross[0][0]=3.;
        m.reanchor([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]);
        assert_eq!(m.mean,[-3.,2.,4.]);approx(m.covariance[0][0],7.);approx(m.covariance[1][1],5.);
        approx(m.cross[7][1],2.);approx(m.cross[0][1],3.);approx(m.cross[6][0],0.);
    }
    #[test]fn clone_owns_its_nuisance_state(){let f=filter();let mut g=f.clone();g.gnss_bias.mean[0]=10.;approx(f.gnss_bias.mean[0],0.);}
}
