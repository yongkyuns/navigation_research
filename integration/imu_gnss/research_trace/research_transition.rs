//! Research-only exact linearized state-transition trace for pinned 18-state ESKF.
//!
//! The trace is a product of the same F, (I-KH), G, and navigation-frame
//! reanchor operators used by the native EKF. It does NOT measure map accuracy,
//! choose a map edge, or update an uncertain map. No allocation is performed.
//!
//! Any untraced/reseed operation must invalidate this trace rather than
//! allow a plausible but stale navigation-to-map cross covariance.

const N: usize = 18;
type Matrix = [[f32; N]; N];

#[derive(Debug, Clone)]
pub struct ResearchTrace {
    transform: Matrix,
    valid: bool,
    count: u32,
}

impl ResearchTrace {
    pub fn new() -> Self { Self { transform: identity(), valid: true, count: 0 } }

    fn left(&mut self, a: &Matrix) {
        let mut out = [[0f32; N]; N];
        for i in 0..N {
            for k in 0..N {
                let aik = a[i][k];
                for j in 0..N { out[i][j] += aik * self.transform[k][j]; }
            }
        }
        self.transform = out;
        self.count = self.count.saturating_add(1);
    }

    pub fn predict(&mut self, f: &Matrix) { self.left(f); }

    /// The full error-coordinate response for an accepted scalar sensor update.
    pub fn scalar(&mut self, h: &[f32; N], k: &[f32; N]) {
        let mut ht = [0f32; N];
        for a in 0..N {
            for b in 0..N { ht[b] += h[a] * self.transform[a][b]; }
        }
        for a in 0..N {
            for b in 0..N { self.transform[a][b] -= k[a] * ht[b]; }
        }
        self.count = self.count.saturating_add(1);
    }

    /// Equal to the original filter's ESKF attitude reset: I - [dtheta]x/2.
    pub fn reset(&mut self, dx: &[f32; N]) {
        let mut g = identity();
        let (x,y,z) = (dx[0]*0.5, dx[1]*0.5, dx[2]*0.5);
        g[0][1] += z;  g[0][2] -= y;
        g[1][0] -= z;  g[1][2] += x;
        g[2][0] += y;  g[2][1] -= x;
        self.left(&g);
    }

    /// Same block rotation as rotate_ekf_covariance_nav_blocks: attitude,
    /// velocity, and position error blocks each rotate from old to new NED.
    pub fn reanchor(&mut self, r: [[f32; 3]; 3]) {
        let mut t = identity();
        for block in 0..3 {
            let base = block * 3;
            for i in 0..3 {for j in 0..3 {t[base+i][base+j]=r[i][j];}}
        }
        self.left(&t);
    }

    pub fn freeze_mount(&mut self) {
        for i in 15..N { self.transform[i] = [0.0; N]; }
        self.count = self.count.saturating_add(1);
    }

    pub fn invalidate(&mut self) { self.valid=false; }

    /// Consume exactly one source-event trace. A second read is identity.
    pub fn take(&mut self) -> (bool, u32, Matrix) {
        let old=core::mem::replace(self, Self::new());
        (old.valid, old.count, old.transform)
    }
}

fn identity() -> Matrix {
    let mut matrix=[[0f32;N];N];
    for i in 0..N {matrix[i][i]=1.;}
    matrix
}

#[cfg(test)]
mod tests {
    use super::*;
    fn close(a:f32,b:f32){assert!((a-b).abs() < 2e-4, "{a} vs {b}");}

    #[test]
    fn scalar_then_reset_keeps_exact_operator_order() {
        let mut t=ResearchTrace::new();
        let mut f=identity();f[6][3]=.04;f[7][4]=.04;
        t.predict(&f);
        let mut h=[0f32;18];h[6]=1.;
        let mut k=[0f32;18];k[6]=.8;k[3]=.2;
        t.scalar(&h,&k);
        let mut dx=[0f32;18];dx[0]=.1;dx[1]=-.07;
        t.reset(&dx);
        let (valid,count,a)=t.take();
        assert!(valid);assert_eq!(count,3);
        close(a[6][6],.2); close(a[6][3],.008);
        close(a[3][6],-.2); close(a[3][3],.992);
        close(a[0][2],.035);close(a[1][2],.05);
        assert_eq!(t.take().1,0);
    }
    #[test]
    fn reanchor_changes_all_three_navigation_blocks_only() {
        let mut t=ResearchTrace::new();
        let r=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]];
        t.reanchor(r);
        let (_,_,m)=t.take();
        for block in 0..3 {
            for i in 0..3 {for j in 0..3 {close(m[3*block+i][3*block+j],r[i][j]);}}
        }
        close(m[9][9],1.);close(m[17][17],1.);
    }
    #[test]
    fn invalidity_and_freeze_do_not_pretend_valid_correlation() {
        let mut t=ResearchTrace::new();t.freeze_mount();t.invalidate();
        let (v,_,m)=t.take();assert!(!v);
        for i in 15..18 {for j in 0..18 {close(m[i][j],0.);}}
        assert!(t.take().0);
    }
}
