#!/usr/bin/env python3
"""
analizar_ruido_tilt.py — ¿de dónde sale el "ruido" del error de tilt en las trazas simuladas?

Lee los cuatro CSV de herramientas/comparar_ganancias_ff.py (registros/comp_ganancias/) y
separa, en régimen (desde 8 s), lo que MIDE el detector de lo que pasa DE VERDAD:

    error medido   = dtilt_deg                      (centroide detectado respecto del centro)
    error real     = −(w_real − cy)·IFOV            (posición con que se dibujó el punto)
    error de medición = medido − real               (lo aporta el detector)

Sólo sirve con registros SIMULADOS: en la cámara real no existe la columna w_real.

Uso (PC, después de comparar_ganancias_ff.py):
    python3 herramientas/analizar_ruido_tilt.py
Escribe docs/figuras/ruido_tilt.png y .pdf, e imprime la tabla.
"""
#* Magnitudes de referencia, para leer la escala: 1 px = 0,115°; 1 cuenta del AS5600 = 0,088°;
#* 1 micropaso a 1/16 = 0,1125°; 1 paso completo = 1,8° (relación 1:1 en ambos ejes).
#? La planta simulada es continua: NO tiene la cuantización del paso del motor. En la
#? plataforma real el piso del error lo pone el micropaso, no lo que se ve aquí.
import csv
import glob
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.normpath(os.path.join(AQUI, ".."))
sys.path.insert(0, RAIZ)
from pt.geometria import Geometria                                # noqa: E402
from pt.parametros import Parametros                              # noqa: E402

CASOS = [("A_sin_ff", "Grupo A, sin feedforward"), ("A_con_ff", "Grupo A, con feedforward"),
         ("B_sin_ff", "Grupo B, sin feedforward"), ("B_con_ff", "Grupo B, con feedforward")]
LSB_AS5600 = 360.0 / 4096.0
rms = lambda x: float(np.sqrt(np.nanmean(np.asarray(x, dtype=float) ** 2)))
coma = lambda x, d=3: f"{x:.{d}f}".replace(".", ",")


def cargar(caso, carpeta, geo, desde=8.0):
    rutas = sorted(glob.glob(os.path.join(carpeta, caso, "*.csv")), key=os.path.getmtime)
    if not rutas:
        sys.exit(f"Falta {caso}: correr antes herramientas/comparar_ganancias_ff.py")
    with open(rutas[-1], newline="") as f:
        filas = list(csv.DictReader(f))
    col = lambda c: np.array([float(x[c]) if x[c] not in ("", "nan") else np.nan for x in filas])
    t = (col("t_exp_ns") - float(filas[0]["t_exp_ns"])) * 1e-9
    sel = np.array([x["estado"] == "SEGUIMIENTO" and x["valido"] == "1" for x in filas]) & (t >= desde)
    real = -(col("w_real") - geo.cy) * geo.ifov_y_deg
    return {"t": t[sel], "medido": col("dtilt_deg")[sel], "real": real[sel], "w": col("w")[sel],
            "w_real": col("w_real")[sel], "w_cmd": col("esp32_w_tilt")[sel],
            "dpan": col("dpan_deg")[sel], "ifov": geo.ifov_y_deg}


def rapido(x, n=40):
    """Parte rápida (> ~2 Hz): la señal menos su media móvil de 0,5 s (40 cuadros a 80 fps)."""
    return x - np.convolve(x, np.ones(n) / n, mode="same")


def main():
    P = Parametros()
    geo = Geometria.desde_parametros(P)
    carpeta = os.path.join(RAIZ, "registros", "comp_ganancias")
    salida = os.path.normpath(os.path.join(RAIZ, "..", "..", "docs", "figuras"))
    D = {c: cargar(c, carpeta, geo) for c, _ in CASOS}
    px = D["A_sin_ff"]["ifov"]

    print(f"Referencias: 1 px = {coma(px)}° | 1 cuenta AS5600 = {coma(LSB_AS5600)}° | micropaso 1/16 = 0,1125° | paso completo = 1,8°\n")
    print(f"{'caso':<26s}{'medido RMS':>12s}{'real RMS':>10s}{'medición RMS':>14s}{'rápido medido':>15s}{'rápido real':>13s}{'pegado ±1 px':>14s}{'ω_cmd rápido':>14s}")
    for c, nombre in CASOS:
        d = D[c]
        pegado = 100 * np.mean(np.abs(np.abs(d["medido"]) - px) < 0.004)
        print(f"{nombre:<26s}{coma(rms(d['medido'])):>11s}°{coma(rms(d['real'])):>9s}°{coma(rms(d['medido'] - d['real'])):>13s}°"
              f"{coma(rms(rapido(d['medido']))):>14s}°{coma(rms(rapido(d['real']))):>12s}°{coma(pegado, 1):>12s} %{coma(rms(rapido(d['w_cmd'])), 2):>10s} °/s")
    print("\nVariación cuadro a cuadro del error medido (RMS), tilt contra pan:")
    for c, nombre in CASOS:
        print(f"  {nombre:<26s} tilt {coma(rms(np.diff(D[c]['medido'])))}°   pan {coma(rms(np.diff(D[c]['dpan'])))}°")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.ticker import FuncFormatter
    except ImportError:
        print("matplotlib no está instalado: se omite la figura.")
        return
    TINTA, TINTA2, GRILLA, EJE = "#0b0b0b", "#52514e", "#e1e0d9", "#c3c2b7"
    MEDIDO, REAL = "#2a78d6", "#0b0b0b"
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
        "axes.edgecolor": EJE, "axes.linewidth": 0.8, "axes.labelcolor": TINTA2, "text.color": TINTA,
        "xtick.color": TINTA2, "ytick.color": TINTA2, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRILLA,
        "grid.linewidth": 0.6, "axes.axisbelow": True, "legend.frameon": False, "legend.fontsize": 8.5,
        "savefig.dpi": 220, "savefig.bbox": "tight", "savefig.pad_inches": 0.06,
    })
    fmt = FuncFormatter(lambda x, _: f"{x:g}".replace(".", ",").replace("-", "−"))
    fig, axs = plt.subplots(1, 3, figsize=(10.4, 3.3), gridspec_kw={"width_ratios": [1.25, 1.25, 1]})

    for ax, caso, titulo in ((axs[0], "A_sin_ff", "Grupo A, sin feedforward"),
                             (axs[1], "B_con_ff", "Grupo B, con feedforward")):
        d = D[caso]
        m = (d["t"] >= 14) & (d["t"] <= 22)
        ax.plot(d["t"][m], d["medido"][m], color=MEDIDO, linewidth=1.2, label="medido por el detector")
        ax.plot(d["t"][m], d["real"][m], color=REAL, linewidth=1.2, label="real")
        for y in (px, -px):
            ax.axhline(y, color=TINTA2, linewidth=0.8, linestyle=(0, (4, 3)))
        ax.text(22.05, px, "+1 px", va="center", fontsize=8, color=TINTA2)
        ax.text(22.05, -px, "−1 px", va="center", fontsize=8, color=TINTA2)
        ax.set_ylim(-0.24, 0.24)
        ax.set_xlim(14, 22)
        ax.set_title(titulo, loc="left")
        ax.set_xlabel("tiempo de la corrida [s]")
        ax.yaxis.set_major_formatter(fmt)
    axs[0].set_ylabel("error de centrado en tilt [°]")
    axs[0].legend(loc="lower left", ncol=2, bbox_to_anchor=(0.0, -0.36))

    #* Curva del detector: error de medición según dónde cae el punto respecto de la grilla del
    #* plano V (una muestra V cada 2 px; su centro está en w = 2j + 0,5).
    w = np.concatenate([D[c]["w"] for c, _ in CASOS])
    wr = np.concatenate([D[c]["w_real"] for c, _ in CASOS])
    fase = np.mod(wr - 0.5 + 1.0, 2.0) - 1.0          # −1..+1 px desde el centro de la muestra V más cercana
    axs[2].scatter(fase, w - wr, s=3, color=MEDIDO, alpha=0.25, linewidths=0)
    axs[2].axhline(0, color=EJE, linewidth=0.8)
    axs[2].set_title("Error del detector según la posición", loc="left")
    axs[2].set_xlabel("posición real respecto del centro\nde la muestra V más cercana [px]")
    axs[2].set_ylabel("medido − real [px]")
    axs[2].xaxis.set_major_formatter(fmt)
    axs[2].yaxis.set_major_formatter(fmt)
    fig.tight_layout(w_pad=1.8)
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(salida, f"ruido_tilt.{ext}"))
    print(f"\nFigura: {os.path.join(salida, 'ruido_tilt')}.png (.pdf)")


if __name__ == "__main__":
    main()
