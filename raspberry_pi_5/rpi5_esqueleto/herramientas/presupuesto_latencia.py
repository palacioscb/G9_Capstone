#!/usr/bin/env python3
"""
presupuesto_latencia.py — Estimación de la latencia "captura -> consigna aplicada en el ESP32"
(Monte Carlo) combinando componentes MEDIDOS con componentes ESTIMADOS.

  Componente                               Fuente
  ---------------------------------------  ---------------------------------------------
  1 Exposición (centro -> fin, T_exp/2)    configuración (exposición fija)
  2 Lectura del sensor + ISP + entrega     MEDIDO: bench_camara.py, por cuadro
  3 Procesamiento en la RPi (PI, EP, CS-1) ESTIMADO: U(0,1; 0,5) ms  -> medir con main.py
  4 USB (host -> puente USB-UART)          ESTIMADO: U(0; 1) ms, trama USB full-speed de 1 ms
  5 Transmisión UART                       CALCULADO: bytes · 10 / baudios
  6 ESP32 (parser + tic de rampa de 1 ms)  ESTIMADO: 0,05 ms + U(0; 1) ms (updateRamp cada 1 ms)

Se evalúan dos escenarios del enlace: firmware PoC actual (texto, 115 200 baudios)
y diseño (trama binaria de 15 B, 921 600 baudios).

Además calcula el retardo EQUIVALENTE para el análisis de control:
    tau_eq = E[L] + T/2     (el muestreo de la cámara actúa como retenedor de orden cero)
y el error de seguimiento que produce ese retardo sin predictor: e = omega · tau_eq.

Uso:
  python3 herramientas/presupuesto_latencia.py \
      --cuadros resultados/bench_yuv/cuadros_m1_640x480.csv --fps 60 --exposicion 2000 \
      --salida resultados/latencia
"""
import argparse
import csv
import os

import numpy as np

COMPONENTES = [
    "Exposición (T_exp/2)",
    "Lectura sensor + ISP + entrega",
    "Procesamiento RPi",
    "USB host→puente",
    "UART",
    "ESP32 (parser + rampa)",
]
FUENTE = ["configuración", "medido", "estimado", "estimado", "calculado", "estimado"]
ESCENARIOS = {                    # nombre: (baudios, bytes por consigna)
    "PoC actual (texto, 115 200 Bd)": (115200, 9),       # "v 1234.5\n"
    "Diseño (trama, 921 600 Bd)": (921600, 15),           # AA55 + 3 + 8 + CRC16
}
# Paleta categórica validada (dataviz, modo claro): slots 1..6 en orden fijo
COLORES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
TINTA, TINTA2, SUPERFICIE = "#0b0b0b", "#52514e", "#ffffff"


def leer_latencias(ruta):
    with open(ruta, newline="") as f:
        return np.array([float(r["lat_lectura_ms"]) for r in csv.DictReader(f)])


def simular(lat_medida, exp_us, baud, nbytes, n, rng):
    c = np.zeros((n, 6))
    c[:, 0] = exp_us / 2000.0
    c[:, 1] = rng.choice(lat_medida, n, replace=True)
    c[:, 2] = rng.uniform(0.1, 0.5, n)
    c[:, 3] = rng.uniform(0.0, 1.0, n)
    c[:, 4] = nbytes * 10 / baud * 1e3
    c[:, 5] = 0.05 + rng.uniform(0.0, 1.0, n)
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cuadros", default="resultados/bench_yuv/cuadros_m1_640x480.csv")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--exposicion", type=float, default=2000.0, help="µs")
    ap.add_argument("--ifov", type=float, default=0.1154, help="°/px en el centro")
    ap.add_argument("-n", type=int, default=200000)
    ap.add_argument("--salida", default="resultados/latencia")
    a = ap.parse_args()
    os.makedirs(a.salida, exist_ok=True)
    rng = np.random.default_rng(1)

    lat = leer_latencias(a.cuadros)
    T = 1e3 / a.fps
    print(f"Datos medidos: {a.cuadros} (n={len(lat)}, mediana {np.median(lat):.2f} ms, "
          f"p95 {np.percentile(lat, 95):.2f} ms)\n")

    res = {}
    for nombre, (baud, nb) in ESCENARIOS.items():
        c = simular(lat, a.exposicion, baud, nb, a.n, rng)
        tot = c.sum(axis=1)
        res[nombre] = (c, tot)

    # ---------------- Tabla por componente ----------------
    filas = []
    print(f"{'Componente':34s} {'Fuente':13s}" + "".join(f"{k[:22]:>26s}" for k in res))
    for i, comp in enumerate(COMPONENTES):
        vals = [f"{res[k][0][:, i].mean():6.2f} (p95 {np.percentile(res[k][0][:, i], 95):5.2f})" for k in res]
        print(f"{comp:34s} {FUENTE[i]:13s}" + "".join(f"{v:>26s}" for v in vals))
        filas.append([comp, FUENTE[i]] + [f"{res[k][0][:, i].mean():.2f}" for k in res])
    print("-" * 120)
    resumen = {}
    for k, (c, tot) in res.items():
        m, p50, p95, p99 = tot.mean(), *np.percentile(tot, [50, 95, 99])
        tau = m + T / 2
        resumen[k] = (m, p50, p95, p99, tau)
        print(f"{k}: L media {m:.2f} ms | p50 {p50:.2f} | p95 {p95:.2f} | p99 {p99:.2f} ms "
              f"| tau_eq = L + T/2 = {tau:.2f} ms (T = {T:.2f} ms)")

    print("\nError de seguimiento por retardo, SIN predictor (e = ω·tau_eq):")
    for w in (5, 10, 20, 40):
        txt = " | ".join(f"{k.split(' (')[0]}: {w*r[4]/1e3:.2f}° = {w*r[4]/1e3/a.ifov:.1f} px"
                         for k, r in resumen.items())
        print(f"  ω = {w:>2} °/s -> {txt}")

    # ---------------- CSV + tabla LaTeX ----------------
    with open(os.path.join(a.salida, "presupuesto_latencia.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["componente", "fuente"] + [f"media_ms {k}" for k in res])
        w.writerows(filas)
        for k, r in resumen.items():
            w.writerow([f"TOTAL {k}", "", *[f"{x:.2f}" for x in r]])
    with open(os.path.join(a.salida, "tabla_latencia.tex"), "w") as f:
        f.write("\\begin{tabular}{llrr}\n\\hline\n")
        f.write("Componente & Fuente & PoC 115\\,200 Bd [ms] & Diseño 921\\,600 Bd [ms] \\\\\n\\hline\n")
        flecha = "$\\to$"
        for fi in filas:
            nombre = fi[0].replace("→", flecha).replace("T_exp/2", "$T_{exp}/2$")
            v1, v2 = fi[2].replace(".", ","), fi[3].replace(".", ",")
            f.write(nombre + " & " + fi[1] + " & " + v1 + " & " + v2 + " \\\\\n")
        f.write("\\hline\n")
        r1, r2 = list(resumen.values())
        f.write(f"Total, media (p95) & & {r1[0]:.1f} ({r1[2]:.1f}) & {r2[0]:.1f} ({r2[2]:.1f}) \\\\\n"
                .replace(".", ","))
        f.write(f"Retardo equivalente $\\tau_{{eq}}=\\bar L+T/2$ & & {r1[4]:.1f} & {r2[4]:.1f} \\\\\n"
                .replace(".", ","))
        f.write("\\hline\n\\end{tabular}\n")

    # ---------------- Figuras ----------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib no disponible: se omiten las figuras)")
        return
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": TINTA2, "axes.labelcolor": TINTA,
                         "xtick.color": TINTA2, "ytick.color": TINTA2, "text.color": TINTA})

    # Fig. 1: barras apiladas horizontales con la media de cada componente
    fig, ax = plt.subplots(figsize=(8.0, 2.8), dpi=200)
    nombres = list(res)
    for j, k in enumerate(nombres):
        izq = 0.0
        for i in range(6):
            v = res[k][0][:, i].mean()
            ax.barh(j, v, left=izq, height=0.5, color=COLORES[i], edgecolor=SUPERFICIE, linewidth=1.5,
                    label=COMPONENTES[i] if j == 0 else None)
            if v > 1.4:
                ax.text(izq + v / 2, j, f"{v:.1f}", ha="center", va="center", fontsize=8, color=TINTA)
            izq += v
        ax.text(izq + 0.3, j, f"{izq:.1f} ms", va="center", fontsize=9, color=TINTA)
    ax.set_yticks(range(len(nombres)), nombres)
    ax.invert_yaxis()
    ax.set_xlabel("Latencia media: centro de exposición → consigna aplicada en el ESP32 [ms]")
    ax.set_xlim(0, max(r[0] for r in resumen.values()) * 1.15)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="x", color="#e4e3dd", linewidth=0.6)
    ax.set_axisbelow(True)
    ax.legend(ncol=3, fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.32))
    fig.tight_layout()
    fig.savefig(os.path.join(a.salida, "presupuesto_latencia.png"), bbox_inches="tight")
    plt.close(fig)

    # Fig. 2: función de distribución acumulada de la latencia total
    fig, ax = plt.subplots(figsize=(6.0, 3.2), dpi=200)
    for j, k in enumerate(nombres):
        tot = np.sort(res[k][1])
        y = np.arange(1, len(tot) + 1) / len(tot)
        ax.plot(tot, y, color=COLORES[j], linewidth=2, label=k)
        p95 = np.percentile(tot, 95)
        ax.plot([p95], [0.95], "o", ms=6, color=COLORES[j], markeredgecolor=SUPERFICIE, markeredgewidth=1.5)
        ax.annotate(f"p95 = {p95:.1f} ms", (p95, 0.95), xytext=(8, -14 - 14 * j),
                    textcoords="offset points", fontsize=8, color=TINTA)
    ax.set_xlabel("Latencia total [ms]")
    ax.set_ylabel("Probabilidad acumulada")
    ax.set_ylim(0, 1.02)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(color="#e4e3dd", linewidth=0.6)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(a.salida, "cdf_latencia.png"), bbox_inches="tight")
    plt.close(fig)
    print(f"\nFiguras, CSV y tabla LaTeX en: {a.salida}")


if __name__ == "__main__":
    main()
