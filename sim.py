import numpy as np
from scipy.signal import cont2discrete
from analisis import Kt, M, bval

def plant_dis(ax, z, dtf):
    m = M[ax]; b = bval(ax, z)
    A = np.array([[0, 1, 0], [0, 0, 1], [0, -Kt/m, -b/m]])
    B = np.array([[0], [0], [Kt/m]])
    C = np.array([[1, 0, 0]]); D = np.zeros((1, 1))
    Ad, Bd, *_ = cont2discrete((A, B, C, D), dtf, method="zoh")
    return Ad, Bd

def simulate(ax, z, Kp, Ki, Kd, Ts=0.010, Dcam=0.025, T=12.0, ref=lambda t: 0.0,
             dtf=0.0005, comp_delay=True, noise=None, Nd=None, ref_delayed=True):
    Ad, Bd = plant_dis(ax, z, dtf)
    n = int(round(T/dtf))
    r = int(round(Ts/dtf))          # fine steps per sample
    nd = int(round(Dcam/dtf))       # camera delay in fine steps
    x = np.zeros(3)
    th = np.zeros(n+1)
    u_applied = 0.0
    u_next = 0.0
    I = 0.0
    e_prev = None
    out_t = []; out_e = []; out_u = []
    t = np.arange(n+1)*dtf
    rng = np.random.default_rng(0)
    for k in range(n):
        if k % r == 0:
            tk = k*dtf
            thm = th[max(k-nd, 0)]
            if noise is not None:
                thm = thm + noise*rng.standard_normal()
            e = ref(max(tk - (Dcam if ref_delayed else 0.0), 0.0)) - thm
            if e_prev is None:
                e_prev = e
            I += Ki*Ts*e
            d = Kd*(e-e_prev)/Ts
            e_prev = e
            u_new = Kp*e + I + d
            if comp_delay:
                u_applied = u_next
                u_next = u_new
            else:
                u_applied = u_new
            out_t.append(tk); out_e.append(e); out_u.append(u_new)
        x = Ad @ x + (Bd[:, 0]*u_applied)
        th[k+1] = x[0]
    return t, th, np.array(out_t), np.array(out_e), np.array(out_u)

def stable(ax, z, Kp, Ki, Kd, Ts=0.010, Dcam=0.025, T=20.0):
    t, th, tk, e, u = simulate(ax, z, Kp, Ki, Kd, Ts, Dcam, T, ref=lambda t: 0.1)
    # compare amplitude at the end vs the first part
    tail = np.abs(e[-int(2/Ts):]).max()
    return tail < 5e-3 and np.isfinite(tail)

if __name__ == "__main__":
    # critical Kp (P only and PI with Ki=Kp^2/4 ... use fixed 0.6*Kp) with delay
    for ax in ["pan", "tilt"]:
        for z in [0.05]:
            # bisection on Kp for P-only
            lo, hi = 1.0, 200.0
            for _ in range(30):
                mid = 0.5*(lo+hi)
                if stable(ax, z, mid, 0.0, 0.0): lo = mid
                else: hi = mid
            print(ax, "zeta", z, "Kp crit (P only, Ts=10ms, Dcam=25ms):", round(lo, 2))
    # D map
    print("Stability map vs Kd (Kp=5, Ki=3, Ts=10ms, Dcam=25ms)")
    for ax in ["pan", "tilt"]:
        for z in [0.02, 0.05, 0.10]:
            row = []
            for Kd in [0.0, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]:
                row.append((Kd, stable(ax, z, 5.0, 3.0, Kd)))
            print(ax, z, row)
