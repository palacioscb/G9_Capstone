import numpy as np
from scipy import signal

Kt = 20.0
M = {"pan": 9.91e-4, "tilt": 1.07e-5}
def bval(ax, z):
    return 2*z*np.sqrt(Kt*M[ax])

Dcam = 0.025

def Lfun(w, ax, z, Kp, Ki, Kd, D):
    s = 1j*w
    m = M[ax]; b = bval(ax, z)
    G = Kt/(s*(m*s**2 + b*s + Kt))
    C = Kp + Ki/s + Kd*s
    return C*G*np.exp(-s*D)

def margins(ax, z, Kp, Ki, Kd, D):
    w = np.logspace(-2, 4.5, 400000)
    L = Lfun(w, ax, z, Kp, Ki, Kd, D)
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    # gain crossovers
    idx = np.where(np.diff(np.sign(mag-1)) != 0)[0]
    res = {}
    res["wc"] = [w[i] for i in idx]
    res["PM"] = [np.degrees(((ph[i] + np.pi) )) for i in idx]  # phase relative to -180
    # phase crossovers (phase = -pi mod 2pi)
    k = np.floor((ph + np.pi)/(2*np.pi))
    idp = np.where(np.diff(np.floor((ph+np.pi)/(2*np.pi))) != 0)[0]
    res["wpc"] = [w[i] for i in idp]
    res["GM_dB"] = [-20*np.log10(mag[i]) for i in idp]
    # sensitivity peak
    S = 1/(1+L)
    res["Ms"] = np.max(np.abs(S))
    res["Ms_dB"] = 20*np.log10(res["Ms"])
    T = L/(1+L)
    res["Mt"] = np.max(np.abs(T))
    return res

def cl_poles(ax, z, Kp, Ki, Kd, D, order=6):
    # closed-loop poles with Pade delay; PID with ideal D -> Kd s^2 + Kp s + Ki over s
    m = M[ax]; b = bval(ax, z)
    numG = [Kt]; denG = [m, b, Kt, 0]
    numC = [Kd, Kp, Ki]; denC = [1, 0]
    num = np.polymul(numG, numC); den = np.polymul(denG, denC)
    if D > 0:
        pn, pd = _pade(D, order)
        num = np.polymul(num, pn); den = np.polymul(den, pd)
    char = np.polyadd(den, num)
    return np.roots(char)

def _pade(T, n):
    from math import factorial
    c = [factorial(2*n-k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n-k)) for k in range(n+1)]
    num = [c[k]*(-T)**k for k in range(n+1)][::-1]
    den = [c[k]*(T)**k for k in range(n+1)][::-1]
    return num, den

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for ax in ["pan", "tilt"]:
        print("==", ax, "b(0.05)=", bval(ax, 0.05), "wn=", np.sqrt(Kt/M[ax]), "Kpmax=", bval(ax,0.05)/M[ax])
        for z in [0.02, 0.05, 0.10]:
            print(" zeta", z, "Kpmax", bval(ax, z)/M[ax])
    # CL poles no delay
    for ax in ["pan", "tilt"]:
        for (Kp, Ki) in [(5, 3), (5, 6.25), (5, 0), (3, 1.5)]:
            print(ax, Kp, Ki, np.sort_complex(cl_poles(ax, 0.05, Kp, Ki, 0, 0, 0)))
    # margins
    for ax in ["pan", "tilt"]:
        for z in [0.02, 0.05, 0.1]:
            for (Kp, Ki) in [(5, 3), (5, 6.25), (5, 0)]:
                for D in [0.0, 0.040]:
                    r = margins(ax, z, Kp, Ki, 0, D)
                    print(ax, "z", z, "Kp,Ki", Kp, Ki, "D", D, "wc", np.round(r["wc"], 3), "PM", np.round(r["PM"], 1), "GM", np.round(r["GM_dB"], 1)[:4], "Ms dB", round(r["Ms_dB"], 2))
