#!/usr/bin/env python3
"""
figuras_prediccion.py — figuras de docs/prediccion_rls.pdf (revisión del predictor, 17-11-2026).

Todas las curvas salen del código REAL del predictor (raspberry_pi_5/rpi5_esqueleto/pt/
rls_predictor.py), no de una reimplementación. La figura de lazo cerrado lee
datos_barrido_lazo_cerrado.json, que se genera con herramientas/barrido_lazo_cerrado.py.

Uso (PC):  python3 docs/figuras/figuras_prediccion.py        # ~1 min; escribe pred_*.png y pred_*.pdf
Requiere numpy y matplotlib.
"""
import json
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
from pt.rls_predictor import PredictorRLS          # noqa: E402

FPS = 80.0
T = 1.0 / FPS                    # período de cuadro [s]
H = 0.020                        # horizonte de predicción de referencia [s]

#* Colores: tinta para el texto y un solo matiz por serie. λ es una magnitud ordenada, así que
#* usa una rampa de un solo matiz (claro -> oscuro); orden 2 / orden 3 son identidades (azul / naranja).
TINTA, TINTA2, TENUE, GRILLA, EJE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
AZUL, NARANJA, AGUA = "#2a78d6", "#eb6834", "#1baf7a"
RAMPA_LAM = {0.60: "#86b6ef", 0.80: "#3987e5", 0.90: "#1c5cab", 0.95: "#0d366b"}

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
    "axes.edgecolor": EJE, "axes.linewidth": 0.8, "axes.labelcolor": TINTA2, "text.color": TINTA,
    "xtick.color": TINTA2, "ytick.color": TINTA2, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRILLA,
    "grid.linewidth": 0.6, "axes.axisbelow": True, "legend.frameon": False, "legend.fontsize": 8,
    "figure.dpi": 100, "savefig.dpi": 220, "savefig.bbox": "tight", "savefig.pad_inches": 0.04,
    "lines.linewidth": 2.0, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
})


def coma(x, dec=None):
    """Número con coma decimal (documento en español)."""
    s = f"{x:.{dec}f}" if dec is not None else f"{x:g}"
    return s.replace(".", ",")


COMA = FuncFormatter(lambda x, _: coma(x))


def guardar(fig, nombre):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(AQUI, f"{nombre}.{ext}"))
    plt.close(fig)
    print("ok", nombre)


def punto(ax, x, y, color, tam=7):
    """Marcador con anillo del color de la superficie (se lee aunque cruce una línea)."""
    ax.plot([x], [y], "o", ms=tam, mfc=color, mec="white", mew=1.6, zorder=6)


# ---------------------------------------------------------------------------------------------
# Figura 1 — la idea: ajustar una curva a los últimos cuadros y evaluarla un poco más adelante
# ---------------------------------------------------------------------------------------------
def fig_concepto():
    rng = np.random.default_rng(4)
    lam, n, sigma, N = 0.80, 3, 0.10, 14
    real = lambda t: 12.0 + 26.0 * t + 70.0 * t * t           # objetivo acelerando [°], t en s (0 = último cuadro)
    ts = np.arange(-(N - 1), 1) * T
    ys = real(ts) + rng.normal(0, sigma, N)
    r = PredictorRLS(n=n, lam=lam)
    for t, y in zip(ts, ys):
        r.actualizar(t, y, True)
    pred = r.predecir(H)

    fig, ax = plt.subplots(figsize=(7.2, 3.3))
    tt = np.linspace(ts[0], H + 0.006, 300)
    ax.plot(tt * 1e3, real(tt), color=EJE, lw=1.2, zorder=2)
    ta = np.linspace(-0.100, 0.0, 100)
    ax.plot(ta * 1e3, [r.predecir(t) for t in ta], color=AZUL, zorder=3)
    tb = np.linspace(0.0, H, 30)
    ax.plot(tb * 1e3, [r.predecir(t) for t in tb], color=AZUL, ls=(0, (2.5, 2)), zorder=3)   # tramo extrapolado
    pesos = lam ** np.arange(N - 1, -1, -1)
    ax.scatter(ts * 1e3, ys, s=14 + 62 * pesos, color=AZUL, alpha=0.30 + 0.70 * pesos, lw=0, zorder=4)
    punto(ax, H * 1e3, pred, NARANJA, 8.5)
    ax.plot([0, H * 1e3], [ys[-1], ys[-1]], color=TENUE, lw=1.2, zorder=3)
    ax.plot([H * 1e3], [ys[-1]], "o", ms=5, mfc="white", mec=TENUE, mew=1.3, zorder=5)
    for x in (0.0, H * 1e3):
        ax.axvline(x, color=GRILLA, lw=0.9, zorder=1)

    kw = dict(fontsize=8, color=TINTA2, arrowprops=dict(arrowstyle="-", color=TENUE, lw=0.7, shrinkA=1, shrinkB=3))
    ax.annotate("mediciones de la cámara\n(las recientes pesan más)", (ts[-5] * 1e3, ys[-5]), (-95, 13.25), ha="center", **kw)
    ax.annotate("curva ajustada por el RLS\n(posición, velocidad, aceleración)", (-62, r.predecir(-0.062)), (-118, 11.9), ha="center", **kw)
    ax.annotate("trayectoria real", (-128, real(-0.128)), (-150, 10.55), ha="left", **kw)
    ax.annotate("predicción: la referencia θ_ref\nque se envía al ESP32", (H * 1e3, pred), (-22, 13.45), ha="center", **kw)
    ax.annotate("sin predictor se enviaría\nla última medición", (H * 1e3, ys[-1]), (4, 11.2), ha="center", **kw)
    y0 = ax.get_ylim()[0]
    ax.text(0, y0, "último cuadro ", fontsize=7.5, color=TENUE, ha="right", va="bottom")
    ax.text(H * 1e3, y0, " llega la consigna", fontsize=7.5, color=TENUE, ha="left", va="bottom")
    ax.set_xlim(-172, 52)
    ax.set_xlabel("tiempo respecto del último cuadro [ms]")
    ax.set_ylabel("dirección del objetivo [°]")
    ax.yaxis.set_major_formatter(COMA)
    guardar(fig, "pred_concepto")


# ---------------------------------------------------------------------------------------------
# Figura 2 — qué hace λ: (a) cuánto pesa cada cuadro, (b) cómo responde a una maniobra
# ---------------------------------------------------------------------------------------------
def fig_lambda():
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 3.1), gridspec_kw=dict(wspace=0.28))

    edad = np.arange(0, 33)
    for lam, color in reversed(list(RAMPA_LAM.items())):          # leyenda en el orden de las curvas
        a.plot(edad * T * 1e3, lam ** edad, color=color, marker="o", ms=2.6, mec="none",
               label=f"λ = {coma(lam, 2)}  (memoria ≈ {T / (1 - lam) * 1e3:.0f} ms)")
    a.legend(loc="upper right", handlelength=1.4, borderaxespad=0.2)
    a.set_xlim(0, 400)
    a.set_ylim(0, 1.03)
    a.set_xlabel("antigüedad de la medición [ms]")
    a.set_ylabel("peso en el ajuste")
    a.set_title("(a) Memoria: peso de cada cuadro", loc="left", color=TINTA)
    a.yaxis.set_major_formatter(COMA)

    #* (b) El objetivo está quieto y en t = 0 parte a 20 °/s. Mismo ruido para los tres valores de λ.
    rng = np.random.default_rng(11)
    sigma, v = 0.05, 20.0
    k = np.arange(-40, 92)
    t = k * T
    real = lambda x: np.where(x > 0, v * x, 0.0)
    y = real(t) + rng.normal(0, sigma, t.size)
    nombres = {0.60: "λ = 0,60: rápido, deja pasar ruido", 0.80: "λ = 0,80", 0.95: "λ = 0,95: suave, pero lento"}
    for lam in (0.95, 0.80, 0.60):
        r = PredictorRLS(n=3, lam=lam)
        err = []
        for ti, yi in zip(t, y):
            r.actualizar(ti, yi, True)
            err.append(r.predecir(ti + H) - float(real(ti + H)))
        b.plot(t * 1e3, err, color=RAMPA_LAM[lam], lw=1.8, label=nombres[lam])
    b.axhline(0, color=EJE, lw=0.8)
    b.set_xlim(-150, 1100)
    b.set_xlabel("tiempo desde que el objetivo parte [ms]")
    b.set_ylabel("error de la predicción [°]")
    b.set_title("(b) Reacción a una maniobra", loc="left", color=TINTA)
    b.yaxis.set_major_formatter(COMA)
    b.legend(loc="lower right", handlelength=1.4, borderaxespad=0.2)
    b.set_ylim(-1.3, 0.8)
    guardar(fig, "pred_lambda")


# ---------------------------------------------------------------------------------------------
# Figura 3 — cuál λ conviene: error de predicción según λ, orden y ruido de la medición
# ---------------------------------------------------------------------------------------------
def error_senoidal(n, lam, sigma, semillas=(0, 1), N=3000, A=20.0, f=0.25):
    """RMS del error de predicción en t + H para un objetivo senoidal (lazo abierto)."""
    w = 2 * math.pi * f
    e2, m = 0.0, 0
    for s in semillas:
        rng = np.random.default_rng(s)
        ruido = rng.normal(0, sigma, N) if sigma > 0 else np.zeros(N)
        r = PredictorRLS(n=n, lam=lam)
        for k in range(N):
            t = (k + 1) * T
            r.actualizar(t, A * math.sin(w * t) + ruido[k], True)
            if k > 150:
                d = r.predecir(t + H) - A * math.sin(w * (t + H))
                e2 += d * d
                m += 1
    return math.sqrt(e2 / m)


def fig_error_vs_lambda():
    lams = np.round(np.arange(0.575, 0.976, 0.025), 3)
    sigmas = (0.05, 0.10, 0.20)
    A, f = 20.0, 0.25
    w = 2 * math.pi * f
    fig, ejes = plt.subplots(1, 3, figsize=(7.2, 3.05), sharey=True, gridspec_kw=dict(wspace=0.08))
    tabla = {}
    for ax, sigma in zip(ejes, sigmas):
        #* Referencia "sin predictor": usar la última medición como si fuera la posición en t + H.
        sin_pred = math.sqrt((A * w * H) ** 2 / 2 + sigma ** 2)
        ax.axhline(sin_pred, color=TENUE, lw=1.4)
        for n, color in ((2, AZUL), (3, NARANJA)):
            e = [error_senoidal(n, float(l), sigma) for l in lams]
            tabla[(sigma, n)] = dict(zip(lams.tolist(), e))
            ax.plot(lams, e, color=color)
            i = int(np.argmin(e))
            punto(ax, lams[i], e[i], color)
            if n == 3:                                   # se rotula sólo el mínimo que importa
                ax.annotate(f"mínimo {coma(e[i], 3)}°\ncon λ = {coma(float(lams[i]), 2)}", (lams[i], e[i]), (0, -9),
                            textcoords="offset points", ha="center", va="top", fontsize=7.4, color=TINTA2, linespacing=1.15)
        ax.set_yscale("log")
        ax.set_title(f"ruido σ = {coma(sigma, 2)}°", loc="left", color=TINTA)
        ax.set_xlabel("factor de olvido λ")
        ax.set_xticks([0.6, 0.7, 0.8, 0.9])
        ax.xaxis.set_major_formatter(COMA)
        ax.set_ylim(0.02, 3.0)
        ax.set_yticks([0.05, 0.1, 0.2, 0.5, 1, 2])
        ax.yaxis.set_major_formatter(COMA)
        ax.minorticks_off()
        ax.text(0.585, sin_pred * 1.10, "sin predictor", fontsize=7.4, color=TENUE, va="bottom")
    ejes[0].set_ylabel("error RMS de la predicción [°]")
    ejes[0].plot([], [], color=AZUL, label="orden 2 (posición + velocidad)")
    ejes[0].plot([], [], color=NARANJA, label="orden 3 (+ aceleración)")
    fig.legend(loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.06), handlelength=1.6, columnspacing=2.2)
    guardar(fig, "pred_error_lambda")
    return tabla, {s: math.sqrt((A * w * H) ** 2 / 2 + s ** 2) for s in sigmas}


# ---------------------------------------------------------------------------------------------
# Figura 4 — lazo cerrado simulado: el feedforward decide; λ sólo importa con él
# ---------------------------------------------------------------------------------------------
def fig_lazo_cerrado():
    datos = json.load(open(os.path.join(AQUI, "datos_barrido_lazo_cerrado.json"), encoding="utf-8"))["filas"]
    #* Régimen = tramos de SEGUIMIENTO sin oclusiones de la corrida sintética (1–2,9 s y 7–10 s).
    regimen = lambda r: math.sqrt((1.9 * r["rms_1a2.9s_px"] ** 2 + 3.0 * r["rms_7a10s_px"] ** 2) / 4.9)
    series = (("sin feedforward\n(orden 3)", 0, 3, AGUA), ("con feedforward,\norden 2", 1, 2, AZUL), ("con feedforward,\norden 3", 1, 3, NARANJA))
    fig, ejes = plt.subplots(1, 2, figsize=(7.2, 3.1), sharey=True, gridspec_kw=dict(wspace=0.5))
    resumen = {}
    for ax, ruido in zip(ejes, (0.0, 1.0)):
        for nombre, ff, orden, color in series:
            f = sorted((r for r in datos if r["ruido_px"] == ruido and r["feedforward"] == ff and r["orden"] == orden), key=lambda r: r["lam"])
            x, y = [r["lam"] for r in f], [regimen(r) for r in f]
            resumen[(ruido, ff, orden)] = dict(zip(x, y))
            ax.plot(x, y, color=color, marker="o", ms=4.5, mec="white", mew=1.0)
            ax.text(0.962, y[-1], nombre, fontsize=7.6, color=TINTA2, va="center", ha="left", linespacing=1.1)
        ax.set_xlim(0.585, 0.955)
        ax.set_ylim(0, 33)
        ax.set_xticks([0.6, 0.7, 0.8, 0.9])
        ax.xaxis.set_major_formatter(COMA)
        ax.set_xlabel("factor de olvido λ")
        ax.set_title("centroide sin ruido agregado" if ruido == 0 else "centroide con ruido de 1 px", loc="left", color=TINTA)
    ejes[0].set_ylabel("error de centrado RMS en régimen [px]")
    guardar(fig, "pred_lazo_cerrado")
    return resumen


# ---------------------------------------------------------------------------------------------
# Tabla — ganancia de ruido analítica (mínimos cuadrados con pesos exponenciales)
# ---------------------------------------------------------------------------------------------
def ganancias(n, lam, h=H, N=6000):
    """(σ_predicción/σ_medición en t + h,  σ_velocidad/σ_medición [1/s])."""
    i = np.arange(N)
    s = -i * T
    Phi = np.vstack([s ** k for k in range(n)]).T
    w = lam ** i
    Mi = np.linalg.inv((Phi * w[:, None]).T @ Phi)
    C = Mi @ ((Phi * (w ** 2)[:, None]).T @ Phi) @ Mi
    ph = np.array([h ** k for k in range(n)])
    dv = np.array([k * h ** (k - 1) if k else 0.0 for k in range(n)])
    return float(np.sqrt(ph @ C @ ph)), float(np.sqrt(dv @ C @ dv))


if __name__ == "__main__":
    fig_concepto()
    fig_lambda()
    tabla, sin_pred = fig_error_vs_lambda()
    resumen = fig_lazo_cerrado()
    print("\nGanancia de ruido (h = 20 ms, 80 fps):  λ | memoria [ms] | orden 2: pos, vel | orden 3: pos, vel")
    for lam in (0.60, 0.70, 0.80, 0.85, 0.90, 0.95):
        g2, g3 = ganancias(2, lam), ganancias(3, lam)
        print(f"  {lam:.2f} | {T / (1 - lam) * 1e3:5.0f} | {g2[0]:.2f}, {g2[1]:5.1f} | {g3[0]:.2f}, {g3[1]:5.1f}")
    print("\nError RMS de predicción [°], objetivo senoidal (20°, 0,25 Hz):")
    for (sigma, n), d in sorted(tabla.items()):
        print(f"  σ={sigma:.2f} orden {n}: " + "  ".join(f"{l:.3f}:{e:.3f}" for l, e in d.items() if round(l * 1000) % 50 == 0 or abs(l - 0.85) < 1e-6))
    print("  sin predictor:", {s: round(v, 3) for s, v in sin_pred.items()})
    print("\nLazo cerrado, error de centrado RMS en régimen [px]:")
    for clave, d in sorted(resumen.items()):
        print(f"  ruido {clave[0]} px, ff={clave[1]}, orden {clave[2]}: " + "  ".join(f"{l:.2f}:{e:.1f}" for l, e in d.items()))
