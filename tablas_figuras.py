import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analisis import Kt, M, bval, Lfun, margins
from sim import simulate

np.set_printoptions(precision=3, suppress=True)
Dcam = 0.025

print("--- Efecto de Ts (PI 5,3; zeta 0,05; pan) ---")
for Ts in [0.005, 0.010, 0.020, 0.033]:
    D = Dcam + 1.5*Ts
    r = margins("pan", 0.05, 5, 3, 0, D)
    print(f"Ts={Ts*1e3:.0f} ms D={D*1e3:.1f} ms wc={r['wc'][0]:.2f} PM={r['PM'][0]:.1f} GMmin={min(r['GM_dB']):.1f} Ms={r['Ms_dB']:.2f} dB pi/2D={np.pi/(2*D):.1f}")
r0 = margins("pan", 0.05, 5, 3, 0, 0.0)
print("PM sin retardo", r0["PM"][0], "wc", r0["wc"][0], "DM =", np.radians(r0["PM"][0])/r0["wc"][0])

print("--- Rango de ganancias, D = 40 ms ---")
D = Dcam + 1.5*0.010
cands = {"pan": [(3, 1.5), (3, 2.25), (4, 2), (4, 4), (5, 3), (5, 6.25)],
         "tilt": [(5, 3), (8, 8), (8, 16), (12, 18), (12, 36)]}
for ax in ["pan", "tilt"]:
    for z in [0.02, 0.05]:
        for (Kp, Ki) in cands[ax]:
            r = margins(ax, z, Kp, Ki, 0, D)
            print(f"{ax} z={z} Kp={Kp} Ki={Ki}: wc={r['wc'][0]:.2f} PM={r['PM'][0]:.1f} GMmin={min(r['GM_dB']):.1f} dB Ms={r['Ms_dB']:.1f} dB |L(wn)|~{Kp/(bval(ax,z)/M[ax]):.2f}")

# --- simulaciones lineales (digital, Ts=10 ms, Dcam=25 ms)
Ts = 0.010
step = 0.1
def stepinfo(t, th, ref):
    y = th/ref
    ov = (y.max()-1)*100
    idx = np.where(np.abs(y-1) > 0.02)[0]
    ts = t[idx[-1]] if len(idx) else 0.0
    return ov, ts
print("--- escalon 0,1 rad (pan) ---")
resp = {}
for name, (Kp, Ki) in {"P Kp=5": (5, 0), "PI 5/3": (5, 3), "PI 5/6,25": (5, 6.25)}.items():
    t, th, *_ = simulate("pan", 0.05, Kp, Ki, 0.0, Ts, Dcam, 12.0, ref=lambda t: step)
    ov, ts = stepinfo(t, th, step)
    resp[name] = (t, th)
    print(name, "OS=%.1f%%" % ov, "ts2=%.2f s" % ts)

print("--- rampa 0,2 rad/s ---")
v = 0.2
rampresp = {}
for name, (Kp, Ki) in {"P Kp=5": (5, 0), "PI 5/3": (5, 3), "PI 5/6,25": (5, 6.25)}.items():
    t, th, tk, e, u = simulate("pan", 0.05, Kp, Ki, 0.0, Ts, Dcam, 12.0, ref=lambda t: v*t)
    rampresp[name] = (tk, e)
    # error at the end in degrees, and the true error (ref - theta, not delayed)
    true_e = v*t - th
    idx5 = np.where(np.abs(np.degrees(true_e)) > 0.2)[0]
    print('   t(|e|<0.2 deg para siempre) =', t[idx5[-1]] if len(idx5) else 0)
    print(name, "e_ss(true) =", np.degrees(true_e[-1]), "deg;  max |e| =", np.degrees(np.abs(true_e).max()))
# tracking lag due to Dcam (what the controller 'sees' vs real)
print("v*Dcam =", v*Dcam, "rad =", np.degrees(v*Dcam), "deg")

# --- Figura 1: Bode de L (PI 5,3) pan y tilt, con y sin retardo
w = np.logspace(-1, 4, 20000)
fig, axs = plt.subplots(2, 2, figsize=(13, 8.5))
for j, ax in enumerate(["pan", "tilt"]):
    for z, ls in [(0.05, "-"), (0.02, "--")]:
        L = Lfun(w, ax, z, 5, 3, 0, 0.0)
        Ld = Lfun(w, ax, z, 5, 3, 0, D)
        axs[0, j].semilogx(w, 20*np.log10(np.abs(Ld)), ls, label=f"$\\zeta={z}$")
        if z == 0.05:
            axs[1, j].semilogx(w, np.degrees(np.unwrap(np.angle(L))), "-", label="sin retardo")
            axs[1, j].semilogx(w, np.degrees(np.unwrap(np.angle(Ld))), "-", label="con $D=40$ ms")
    axs[0, j].axhline(0, color="k", lw=0.8)
    axs[0, j].set_title(f"Eje {ax}: $|L(j\\omega)|$ con PI $K_p=5$, $K_i=3$")
    axs[0, j].set_ylabel("Magnitud [dB]"); axs[0, j].grid(True, which="both", alpha=0.3)
    axs[0, j].set_ylim(-60, 40)
    axs[0, j].legend()
    axs[1, j].axhline(-180, color="k", lw=0.8)
    axs[1, j].set_ylabel("Fase [°]"); axs[1, j].set_xlabel("$\\omega$ [rad/s]")
    axs[1, j].grid(True, which="both", alpha=0.3)
    axs[1, j].set_ylim(-900, 0)
    axs[1, j].legend()
plt.tight_layout()
plt.savefig("fig_bode_lazo.png", dpi=200)
plt.close()

# --- Figura 2: escalon y rampa (pan, digital con retardo)
fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
for name, (t, th) in resp.items():
    axs[0].plot(t, th/step, label=name)
axs[0].axhline(1, color="k", lw=0.6)
axs[0].set_xlim(0, 8); axs[0].set_xlabel("$t$ [s]"); axs[0].set_ylabel("$\\theta/\\theta_{ref}$")
axs[0].set_title("Escalón (digital, $T_s=10$ ms, $T_d=25$ ms)"); axs[0].grid(alpha=0.3); axs[0].legend()
for name, (tk, e) in rampresp.items():
    axs[1].plot(tk, np.degrees(e), label=name)
axs[1].set_xlim(0, 12); axs[1].set_xlabel("$t$ [s]"); axs[1].set_ylabel("Error de seguimiento $e$ [°]")
axs[1].set_title("Rampa $v=0{,}2$ rad/s (11,5 °/s)"); axs[1].grid(alpha=0.3); axs[1].legend()
plt.tight_layout()
plt.savefig("fig_escalon_rampa.png", dpi=200)
plt.close()

# --- Figura 3: efecto de D en la magnitud del lazo (pan, zeta=0.05)
w = np.logspace(0, 3.2, 20000)
fig, ax = plt.subplots(1, 1, figsize=(13, 4.8))
for Kd in [0.0, 0.05, 0.1, 0.2, 0.5]:
    L = Lfun(w, "pan", 0.05, 5, 3, Kd, 0.0)
    ax.semilogx(w, 20*np.log10(np.abs(L)), label=f"$K_d={Kd}$")
ax.axhline(0, color="k", lw=0.8)
ax.set_ylim(-50, 30); ax.set_xlabel("$\\omega$ [rad/s]"); ax.set_ylabel("$|L(j\\omega)|$ [dB]")
ax.set_title("Eje pan ($\\zeta=0{,}05$): efecto del término derivativo sobre el pico resonante")
ax.grid(True, which="both", alpha=0.3); ax.legend(ncol=5)
plt.tight_layout()
plt.savefig("fig_efecto_D.png", dpi=200)
plt.close()
print("figs ok")
