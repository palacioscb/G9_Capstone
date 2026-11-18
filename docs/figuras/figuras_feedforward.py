#!/usr/bin/env python3
"""
figuras_feedforward.py — figuras y tablas de docs/feedforward_velocidad.pdf (17-11-2026).

Simula SÓLO el lazo interno (ESP32 + eje), con el mismo código que usa main.py en lazo cerrado
simulado: pt/enlace.py (EnlaceSimulado) = planta de pid_t_2 + réplica del PI del firmware a
1 kHz, con la extrapolación de la referencia. La consigna llega cada 12,5 ms y es EXACTA
(posición y velocidad reales del objetivo): así se ve qué aporta el feedforward por sí solo,
sin cámara ni predictor.

Uso (PC):  python3 docs/figuras/figuras_feedforward.py        # ~1 min; escribe ff_*.png y ff_*.pdf
"""
#? Planta ideal de pid_t_2 con ζ = 0,05 supuesto y ganancias de arranque de config.h
#? (pan Kp = 3, Ki = 2,25; tilt Kp = 10, Ki = 25). Sirve para comparar con y sin feedforward.
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "..", "raspberry_pi_5", "rpi5_esqueleto"))
from pt.enlace import EnlaceSimulado            # noqa: E402
from pt.parametros import Parametros            # noqa: E402

T = 1 / 80.0
TINTA, TINTA2, TENUE, GRILLA, EJE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
AZUL, NARANJA, AGUA = "#2a78d6", "#eb6834", "#1baf7a"
SIN, CON = AGUA, NARANJA          # mismas identidades que en prediccion_rls.pdf

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
    "axes.edgecolor": EJE, "axes.linewidth": 0.8, "axes.labelcolor": TINTA2, "text.color": TINTA,
    "xtick.color": TINTA2, "ytick.color": TINTA2, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRILLA,
    "grid.linewidth": 0.6, "axes.axisbelow": True, "legend.frameon": False, "legend.fontsize": 8,
    "savefig.dpi": 220, "savefig.bbox": "tight", "savefig.pad_inches": 0.04,
    "lines.linewidth": 2.0, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
})
coma = lambda x, d=None: (f"{x:.{d}f}" if d is not None else f"{x:g}").replace(".", ",")
COMA = FuncFormatter(lambda x, _: coma(x))


def guardar(fig, nombre):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(AQUI, f"{nombre}.{ext}"))
    plt.close(fig)
    print("ok", nombre)


def seguir(pos, vel, dur, k_ff=0.0, eje=0, ruido_v=0.0, atraso_v=0.0, semilla=0):
    """Lazo interno con consigna cada 12,5 ms: (θ_obj, k_ff·velocidad). Devuelve t, θ_obj, θ, ω_cmd a 1 kHz."""
    P = Parametros()
    P.lazo.lim_pan_deg = P.lazo.lim_tilt_deg = (-90.0, 90.0)      # sin topar los límites de recorrido
    enl = EnlaceSimulado(P)
    rng = np.random.default_rng(semilla)
    t0 = 1_000_000_000
    enl.avanzar_hasta(t0)
    ts, th, om, ob = [], [], [], []
    for k in range(int(dur / T)):
        t = k * T
        ref, wff = [0.0, 0.0], [0.0, 0.0]
        ref[eje] = pos(t)
        wff[eje] = k_ff * vel(t - atraso_v) + (rng.normal(0, ruido_v) if ruido_v else 0.0)
        enl.enviar_consigna(ref[0], ref[1], wff[0], wff[1], t_ns=t0 + int(t * 1e9))
        for _ in range(12 if k % 2 else 13):                      # 12,5 ms a 1 kHz
            enl._paso()
            tt = (enl.t_ns - t0) * 1e-9
            ts.append(tt); th.append(float(enl.x[eje][0])); om.append(float(enl.ctl[eje].omega)); ob.append(pos(tt))
    return np.array(ts), np.array(ob), np.array(th), np.array(om)


# ---------------------------------------------------------------------------------------------
# Figura 1 — el objetivo parte a 20 °/s: ángulo, error y velocidad de comando (eje pan)
# ---------------------------------------------------------------------------------------------
def fig_rampa():
    v0, t_ini = 20.0, 0.5
    pos = lambda t: v0 * max(t - t_ini, 0.0)
    vel = lambda t: v0 if t > t_ini else 0.0
    t, ob, th0, om0 = seguir(pos, vel, 3.5, 0.0)
    _, _, th1, om1 = seguir(pos, vel, 3.5, 1.0)
    fig, (a, b, c) = plt.subplots(3, 1, figsize=(7.2, 5.9), sharex=True, gridspec_kw=dict(hspace=0.30, height_ratios=[1.15, 1, 1]))

    a.plot(t, ob, color=EJE, lw=1.4, label="objetivo")
    a.plot(t, th0, color=SIN, label="plataforma, sin feedforward")
    a.plot(t, th1, color=CON, label="plataforma, con feedforward")
    a.set_ylabel("[°]")
    a.set_title("Ángulo del eje pan", loc="left", color=TINTA)
    a.legend(loc="upper left", handlelength=1.5)
    a.set_ylim(-3, 64)

    b.axhline(0, color=EJE, lw=0.8)
    b.plot(t, ob - th0, color=SIN)
    b.plot(t, ob - th1, color=CON)
    b.set_ylabel("[°]")
    b.set_title("Error de seguimiento (objetivo − plataforma)", loc="left", color=TINTA)
    kw = dict(fontsize=7.8, color=TINTA2, arrowprops=dict(arrowstyle="-", color=TENUE, lw=0.7, shrinkA=1, shrinkB=2))
    e0, e1 = ob - th0, ob - th1
    b.annotate(f"sin feedforward: queda en {coma(e0[-1], 1)}°", (2.6, e0[np.searchsorted(t, 2.6)]), (2.0, 3.9), ha="center", **kw)
    i = int(np.argmax(e1))
    b.annotate(f"con feedforward: {coma(e1[i], 1)}° al partir\n(límite de aceleración) y luego ≈ 0", (t[i], e1[i]), (1.35, 2.1), ha="left", va="center", **kw)
    b.set_ylim(-0.6, 6.2)
    b.yaxis.set_major_formatter(COMA)

    c.axhline(v0, color=EJE, lw=1.2)
    c.plot(t, om0, color=SIN)
    c.plot(t, om1, color=CON)
    c.text(3.46, v0 - 1.6, "línea gris: velocidad del objetivo (20 °/s)", fontsize=7.5, color=TENUE, va="top", ha="right")
    c.set_ylabel("[°/s]")
    c.set_title("Velocidad de comando ω_cmd que el PI entrega al generador de pasos", loc="left", color=TINTA)
    c.set_xlabel("tiempo [s]")
    c.set_ylim(-1, 26)
    c.set_xlim(0, 3.5)
    c.xaxis.set_major_formatter(COMA)
    fig.align_ylabels([a, b, c])
    guardar(fig, "ff_rampa")
    return dict(e_sin_final=float(e0[-1]), e_con_max=float(e1.max()), e_con_final=float(e1[-1]))


# ---------------------------------------------------------------------------------------------
# Figura 2 — objetivo senoidal: error según la frecuencia, con y sin feedforward
# ---------------------------------------------------------------------------------------------
def error_senoidal(f, k_ff, eje, A=15.0, **kw):
    w = 2 * math.pi * f
    t, ob, th, _ = seguir(lambda x: A * math.sin(w * x), lambda x: A * w * math.cos(w * x), 2.0 + max(2.0 / f, 4.0), k_ff, eje, **kw)
    e = (ob - th)[t > 2.0]
    return float(np.sqrt(np.mean(e ** 2)))


def fig_frecuencia():
    frecs = np.array([0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6])
    fig, ejes = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True, gridspec_kw=dict(wspace=0.08))
    tabla = {}
    for ax, eje, nombre in zip(ejes, (0, 1), ("pan (Kp = 3; Ki = 2,25)", "tilt (Kp = 10; Ki = 25)")):
        for k_ff, color, etiqueta in ((0.0, SIN, "sin feedforward"), (1.0, CON, "con feedforward")):
            e = [error_senoidal(float(f), k_ff, eje) for f in frecs]
            tabla[(eje, k_ff)] = dict(zip(frecs.tolist(), e))
            ax.plot(frecs, e, color=color, marker="o", ms=4.5, mec="white", mew=1.0, label=etiqueta)
        ax.axhline(0.115, color=TENUE, lw=1.0)
        ax.text(0.052, 0.125, "1 píxel", fontsize=7.4, color=TENUE, va="bottom")
        ax.set_yscale("log"); ax.set_xscale("log")
        ax.set_xticks([0.05, 0.1, 0.2, 0.5]); ax.xaxis.set_major_formatter(COMA)
        ax.set_yticks([0.01, 0.1, 1, 10]); ax.yaxis.set_major_formatter(COMA)
        ax.minorticks_off()
        ax.set_ylim(0.002, 20)
        ax.set_xlabel("frecuencia del movimiento del objetivo [Hz]")
        ax.set_title(nombre, loc="left", color=TINTA)
    ejes[0].set_ylabel("error de seguimiento RMS [°]")
    ejes[0].legend(loc="lower right", handlelength=1.5)
    guardar(fig, "ff_frecuencia")
    return tabla


# ---------------------------------------------------------------------------------------------
# Figura 3 — la referencia entre consignas: escalera (sin ω_ff) o rampa (extrapolada con ω_ff)
# ---------------------------------------------------------------------------------------------
def fig_extrapolacion():
    v = 20.0
    t = np.linspace(0, 0.0625, 600)
    k = np.floor(t / T)
    fig, ax = plt.subplots(figsize=(7.2, 2.7))
    ax.plot(t * 1e3, v * t, color=EJE, lw=1.4, label="objetivo real (20 °/s)")
    ax.plot(t * 1e3, v * k * T, color=SIN, label="referencia sin ω_ff: escalera de 0,25°")
    ax.plot(t * 1e3, v * k * T + v * (t - k * T), color=CON, ls=(0, (4, 2.5)), label="referencia con ω_ff: el ESP32 la hace avanzar cada 1 ms")
    for i in range(6):
        ax.plot([i * T * 1e3], [v * i * T], "o", ms=6.5, mfc=TINTA2, mec="white", mew=1.4, zorder=5)
    ax.plot([], [], "o", ms=6.5, mfc=TINTA2, mec="white", mew=1.4, label="llega una consigna de la RPi (cada 12,5 ms)")
    ax.set_xlabel("tiempo [ms]")
    ax.set_ylabel("referencia θ_ref [°]")
    ax.yaxis.set_major_formatter(COMA)
    ax.xaxis.set_major_formatter(COMA)
    ax.set_xlim(0, 62.5)
    ax.legend(loc="upper left", handlelength=2.2)
    guardar(fig, "ff_extrapolacion")


if __name__ == "__main__":
    r = fig_rampa()
    print("rampa pan:", {k: round(v, 2) for k, v in r.items()})
    tabla = fig_frecuencia()
    fig_extrapolacion()
    for (eje, k_ff), d in sorted(tabla.items()):
        print(f"senoide 15°, eje {'pan' if eje == 0 else 'tilt'}, k_ff = {k_ff}: " + "  ".join(f"{f:g} Hz: {e:.3f}°" for f, e in d.items()))
    print("\nSensibilidad (pan, senoide de 15° y 0,25 Hz; error RMS):")
    for nombre, kw in (("ideal", {}), ("ganancia 0,8", {"k": 0.8}), ("ganancia 1,2", {"k": 1.2}), ("ganancia 0,5", {"k": 0.5}),
                       ("ruido 0,6 °/s", {"ruido_v": 0.6}), ("ruido 1,8 °/s", {"ruido_v": 1.8}),
                       ("atraso 25 ms", {"atraso_v": 0.025}), ("atraso 50 ms", {"atraso_v": 0.05}), ("atraso 100 ms", {"atraso_v": 0.1}),
                       ("signo invertido", {"k": -1.0})):
        k = kw.pop("k", 1.0)
        print(f"  {nombre:18s} {error_senoidal(0.25, k, 0, **kw):.3f}°")
    print(f"  {'sin feedforward':18s} {error_senoidal(0.25, 0.0, 0):.3f}°")
