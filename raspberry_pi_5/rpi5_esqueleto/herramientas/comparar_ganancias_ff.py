#!/usr/bin/env python3
"""
comparar_ganancias_ff.py — tabla y figuras para la presentación: seguimiento CON y SIN
feedforward para cada grupo de ganancias del PI, en simulación y (después) en el laboratorio.

  Grupo A (arranque, config.h):  pan Kp = 3, Ki = 2,25   | tilt Kp = 10, Ki = 25
  Grupo B (pid_t_2 §8):          pan Kp = 5, Ki = 6,25   | tilt Kp = 20, Ki = 100

1) SIMULACIÓN (PC, ~1 min). Cuatro corridas de main.py en lazo cerrado simulado, con la misma
   escena sintética; sólo cambian las ganancias (--ganancias-sim) y el feedforward:
       python3 herramientas/comparar_ganancias_ff.py
       python3 herramientas/comparar_ganancias_ff.py --reusar      # no repite lo ya simulado

2) LABORATORIO (desde el 19-11-2026, todo desde el tablero y con main.py corriendo una vez).
   Por cada caso: poner las ganancias (tarjeta "Plataforma": Ganancias del PI), marcar o
   desmarcar "Feedforward" en los ajustes, escribir una etiqueta (A_sin_ff, A_con_ff, B_sin_ff,
   B_con_ff), "Grabar", mover el objetivo unos 40 s y "Detener". Cada grabación queda en
   registros/exp_<etiqueta>_<fecha>.csv y se baja desde el mismo tablero. Luego:
       python3 herramientas/comparar_ganancias_ff.py --reusar --lab registros/exp_*.csv
   El grupo (A o B) se reconoce por las ganancias anotadas en cada fila del CSV, y el caso con o
   sin feedforward por la columna ff: da lo mismo cuántos archivos sean o cómo se llamen.
   La tabla y las figuras quedan con las filas "simulado" y "medido" lado a lado.
   (Con registros anteriores, sin las columnas de ganancias: --lab-a ... y --lab-b ...)

Salidas (carpeta docs/figuras del repositorio, o la que indique --salida):
    comp_ganancias_ff.csv           todos los números, una fila por caso (para planilla)
    comp_ganancias_ff.md            tabla lista para pegar en la presentación o el informe
    comp_ganancias_ff_barras.*      figura: error de centrado y atraso, por grupo y feedforward
    comp_ganancias_ff_traza_sim.*   figura: error de centrado en el tiempo (simulado)
    comp_ganancias_ff_traza_lab.*   ídem con lo medido, si se entregó --lab (o --lab-a / --lab-b)
Los CSV crudos de la simulación quedan en registros/comp_ganancias/<grupo>_<sin|con>_ff/.
"""
#? La planta simulada es el modelo ideal de pid_t_2 con ζ = 0,05 SUPUESTO (pt/modelo_eje.py).
#? Los valores simulados sirven para comparar casos entre sí; no predicen el error real.
#* Qué columna comparar entre simulación y laboratorio: el ATRASO [ms]. El error en grados
#* depende de qué tan rápido se movió el objetivo (error ≈ atraso · velocidad), y el
#* movimiento a mano del laboratorio no es el de la escena sintética.
#* RÉGIMEN: de cada simulación se descartan los primeros 8 s (--regimen-desde). Ahí están el
#* enganche inicial (el objetivo parte a 8° del centro) y las dos oclusiones programadas de la
#* escena (3,0–3,5 s y 6,0–6,3 s); con ellos adentro, el transitorio tapa la diferencia entre
#* casos (tilt daba 0,62° RMS en los cuatro). De lo medido se descartan los primeros 2 s de
#* cada tramo (--descartar-lab).
#! En el ESP32 real el grupo B de pan (Kp = 5) queda a un 12 % del techo de estabilidad si
#! ζ ≈ 0,02 (techo 5,7 1/s): subir por pasos (k pan 4 4, luego k pan 5 6.25) y bajar si zumba.
import argparse
import csv
import glob
import os
import subprocess
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.normpath(os.path.join(AQUI, ".."))
sys.path.insert(0, AQUI)
import analizar_registro as ar                                    # noqa: E402

#* (Kp [1/s], Ki [1/s²]) por eje. Para comparar otro grupo, agregarlo aquí (p. ej. "C").
GRUPOS = {
    "A": {"nombre": "arranque (config.h)", "pan": (3.0, 2.25), "tilt": (10.0, 25.0)},
    "B": {"nombre": "pid_t_2 §8", "pan": (5.0, 6.25), "tilt": (20.0, 100.0)},
}
BASE = "comp_ganancias_ff"
coma = lambda x, d=2: "—" if not np.isfinite(x) else f"{round(x, d) + 0.0:.{d}f}".replace(".", ",").replace("-", "−")
grados = lambda x: coma(x, 2 if abs(x) >= 0.1 else 3)             # los errores chicos (tilt) con 3 decimales
gan = lambda g, eje: "Kp {} · Ki {}".format(*(f"{v:g}".replace(".", ",") for v in GRUPOS[g][eje]))


# ---------------------------------------------------------------------------
#* 1. Simulación: una corrida de main.py por caso
# ---------------------------------------------------------------------------
def simular(grupo, ff, a):
    """Corre (o reutiliza) el caso y devuelve la ruta de su CSV."""
    carpeta = os.path.join(a.registros, f"{grupo}_{'con' if ff else 'sin'}_ff")
    previos = sorted(glob.glob(os.path.join(carpeta, "*.csv")), key=os.path.getmtime)
    if a.reusar and previos:
        return previos[-1]
    (kp_pan, ki_pan), (kp_tilt, ki_tilt) = GRUPOS[grupo]["pan"], GRUPOS[grupo]["tilt"]
    cmd = [sys.executable, "main.py", "--enlace", "simulado", "--lampara", "--duracion", f"{a.duracion:g}",
           "--registro", carpeta, "--ganancias-sim", f"{kp_pan:g}", f"{ki_pan:g}", f"{kp_tilt:g}", f"{ki_tilt:g}"]
    if ff:
        cmd.append("--feedforward")
    print("  $ python3 " + " ".join(cmd[1:]), flush=True)
    r = subprocess.run(cmd, cwd=RAIZ, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"main.py falló en el caso {grupo}, ff={ff}:\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}")
    return sorted(glob.glob(os.path.join(carpeta, "*.csv")), key=os.path.getmtime)[-1]


# ---------------------------------------------------------------------------
#* 2. Análisis: mismo cálculo que analizar_registro.py (tramos de feedforward constante)
# ---------------------------------------------------------------------------
def grupo_de(fila, tol=0.02):
    """Grupo de ganancias (A, B...) de una fila del CSV, por sus columnas kp_pan..ki_tilt
    (registros desde el 19-11-2026). None si no las trae o no calzan con ningún grupo."""
    try:
        g = [float(fila[c]) for c in ("kp_pan", "ki_pan", "kp_tilt", "ki_tilt")]
    except (KeyError, TypeError, ValueError):
        return None
    for nombre, d in GRUPOS.items():
        if all(abs(a - b) <= tol * max(1.0, abs(b)) for a, b in zip(g, d["pan"] + d["tilt"])):
            return nombre
    return None


def tramos(ruta, detector=None, min_tramo=3.0, descartar=0.0):
    """Tramos del CSV con detector, feedforward y ganancias constantes:
    [{'res': resumen, 't', 'dpan', 'dtilt', 'grupo'}]. De cada tramo se descartan los primeros
    `descartar` segundos (transitorio). Los tramos con referencia manual no son seguimiento: se omiten."""
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 20:
        return []
    t = (np.array([ar.num(f, "t_exp_ns") for f in filas]) - ar.num(filas[0], "t_exp_ns")) * 1e-9
    salida = []
    for i0, i1, det, _ in ar.partir_en_tramos(filas):
        i0 = int(np.searchsorted(t, t[i0] + descartar))
        if (detector and det != detector) or i1 - i0 < 20 or t[i1 - 1] - t[i0] < min_tramo:
            continue
        if filas[i0].get("manual") == "1":
            continue
        ff, tt = filas[i0:i1], t[i0:i1]
        res = ar.analizar_tramo(ff, tt, os.path.basename(ruta), detalle=False)
        sel = np.array([f.get("estado") == "SEGUIMIENTO" and ar.num(f, "valido") == 1 for f in ff])
        serie = lambda c: np.where(sel, np.array([ar.num(f, c) for f in ff]), np.nan)
        salida.append({"res": res, "t": tt, "dpan": serie("dpan_deg"), "dtilt": serie("dtilt_deg"),
                       "archivo": ruta, "grupo": grupo_de(filas[i0]),
                       "gan": " ".join(str(filas[i0].get(c, "?")) for c in ("kp_pan", "ki_pan", "kp_tilt", "ki_tilt"))})
    return salida


def reunir(a):
    """casos[(origen, grupo, ff)] = tramo. Si hay varios tramos para un caso, queda el más largo."""
    casos = {}

    def agregar(origen, grupo, tr):
        clave = (origen, grupo, int(bool(tr["res"]["ff"])))
        if clave in casos:
            largo, corto = sorted((casos[clave], tr), key=lambda x: -x["res"]["dur"])
            print(f"  aviso: {origen} {grupo} {'con' if clave[2] else 'sin'} ff tiene más de un tramo; se usa el de "
                  f"{largo['res']['dur']:.0f} s ({os.path.basename(largo['archivo'])}) y se descarta el de {corto['res']['dur']:.0f} s")
            tr = largo
        casos[clave] = tr

    if not a.sin_simular:
        print(f"Simulación (lazo cerrado simulado, escena sintética de {a.duracion:g} s; se analiza desde {a.regimen_desde:g} s):")
        for grupo in GRUPOS:
            for ff in (0, 1):
                ruta = simular(grupo, ff, a)
                for tr in tramos(ruta, descartar=a.regimen_desde):
                    agregar("simulado", grupo, tr)
    for grupo, rutas in (("A", a.lab_a), ("B", a.lab_b)):
        for ruta in rutas or []:
            encontrados = tramos(ruta, a.detector, a.min_tramo, a.descartar_lab)
            if not encontrados:
                print(f"  aviso: {ruta} no tiene tramos útiles" + (f" con detector {a.detector}" if a.detector else ""))
            for tr in encontrados:
                agregar("medido", grupo, tr)
    #* --lab: el grupo sale de las ganancias anotadas en el propio CSV.
    for ruta in a.lab or []:
        encontrados = tramos(ruta, a.detector, a.min_tramo, a.descartar_lab)
        if not encontrados:
            print(f"  aviso: {ruta} no tiene tramos útiles" + (f" con detector {a.detector}" if a.detector else ""))
        for tr in encontrados:
            if tr["grupo"] is None:
                print(f"  aviso: {os.path.basename(ruta)}: tramo de {tr['res']['dur']:.0f} s con ganancias ({tr['gan']}) que no "
                      f"son de ningún grupo ({', '.join(GRUPOS)}), o sin esas columnas. Se omite (usar --lab-a / --lab-b).")
                continue
            agregar("medido", tr["grupo"], tr)
    return casos


# ---------------------------------------------------------------------------
#* 3. Tabla (CSV para planilla, Markdown para la presentación)
# ---------------------------------------------------------------------------
def ordenados(casos):
    return sorted(casos.items(), key=lambda kv: (kv[0][0] != "simulado", kv[0][1], kv[0][2]))


def mejora(casos, origen, grupo, clave):
    """Cuántas veces menor es el error con feedforward (None si falta alguno de los dos casos)."""
    s, c = casos.get((origen, grupo, 0)), casos.get((origen, grupo, 1))
    if not s or not c or not (c["res"][clave] > 0) or not np.isfinite(s["res"][clave]):
        return None
    return s["res"][clave] / c["res"][clave]


def escribir_tabla(casos, salida, a):
    ruta_csv, ruta_md = os.path.join(salida, BASE + ".csv"), os.path.join(salida, BASE + ".md")
    with open(ruta_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["origen", "grupo", "kp_pan", "ki_pan", "kp_tilt", "ki_tilt", "feedforward", "duracion_s",
                    "en_seguimiento_pct", "caidas_por_min", "error_pan_rms_deg", "error_pan_rms_px",
                    "error_tilt_rms_deg", "error_tilt_rms_px", "atraso_pan_ms", "atraso_tilt_ms",
                    "vel_objetivo_pan_rms_dps", "vel_objetivo_tilt_rms_dps", "archivo"])
        for (origen, grupo, ff), tr in ordenados(casos):
            r, g = tr["res"], GRUPOS[grupo]
            w.writerow([origen, grupo, *g["pan"], *g["tilt"], ff, f"{r['dur']:.1f}", f"{r['seguimiento']:.1f}",
                        f"{r['caidas_min']:.1f}", f"{r['e_pan']:.3f}", f"{r['e_pan'] / ar.IFOV:.1f}",
                        f"{r['e_tilt']:.3f}", f"{r['e_tilt'] / ar.IFOV:.1f}", f"{r['tau_pan']:.0f}".replace("-0", "0"),
                        f"{r['tau_tilt']:.0f}".replace("-0", "0"), f"{r['v_pan']:.1f}", f"{r['v_tilt']:.1f}",
                        os.path.relpath(tr["archivo"], RAIZ)])

    L = ["# Con y sin feedforward, por grupo de ganancias", "",
         "Error de centrado: distancia angular del objetivo al centro de la imagen, RMS, sólo en SEGUIMIENTO con "
         "detección válida. Atraso: tiempo que la plataforma va detrás del objetivo (regresión del error contra "
         "la velocidad del objetivo).", ""]
    for g, d in GRUPOS.items():
        L.append(f"- **Grupo {g}** ({d['nombre']}): pan {gan(g, 'pan')}; tilt {gan(g, 'tilt')}")
    L += ["", "| Origen | Grupo | Feedforward | Error pan RMS | Error tilt RMS | Atraso pan | Atraso tilt | Vel. objetivo pan (RMS) |",
          "|---|---|---|---|---|---|---|---|"]
    for (origen, grupo, ff), tr in ordenados(casos):
        r = tr["res"]
        e = lambda x: f"{grados(x)}° ({coma(x / ar.IFOV, 1)} px)" if np.isfinite(x) else "—"
        tau = lambda x: f"{coma(x, 0)} ms" if np.isfinite(x) else "—"
        L.append(f"| {origen} | {grupo} | {'sí' if ff else 'no'} | {e(r['e_pan'])} | {e(r['e_tilt'])} | "
                 f"{tau(r['tau_pan'])} | {tau(r['tau_tilt'])} | {coma(r['v_pan'], 1)} °/s |")
    L += ["", "| Origen | Grupo | Reducción del error en pan con feedforward | Reducción en tilt |", "|---|---|---|---|"]
    for origen in ("simulado", "medido"):
        for grupo in GRUPOS:
            m = [mejora(casos, origen, grupo, k) for k in ("e_pan", "e_tilt")]
            if any(x is not None for x in m):
                L.append(f"| {origen} | {grupo} | " + " | ".join(f"{coma(x, 1)} veces" if x else "—" for x in m) + " |")
    L += ["", f"Simulado: planta de pid_t_2 con ζ = 0,05 supuesto y escena sintética de {a.duracion:g} s, en régimen "
          f"(desde {a.regimen_desde:g} s: sin el enganche inicial ni las dos oclusiones programadas). Sirve para comparar "
          "casos, no como predicción del error real. Entre simulado y medido se compara el atraso (ms), que no "
          "depende de la velocidad con que se movió el objetivo.", ""]
    with open(ruta_md, "w") as f:
        f.write("\n".join(L))
    print("\n" + "\n".join(L[5 + len(GRUPOS):-2]))
    print(f"\nTabla: {ruta_csv}\n       {ruta_md}")


# ---------------------------------------------------------------------------
#* 4. Figuras
# ---------------------------------------------------------------------------
TINTA, TINTA2, GRILLA, EJE = "#0b0b0b", "#52514e", "#e1e0d9", "#c3c2b7"
#* Paleta de la presentación (18-11-2026): escala de azules, la misma de docs/figuras/figuras_feedforward.py.
SIN, CON = "#86b6ef", "#0d366b"          # sin feedforward = celeste; con feedforward = azul marino
ROTULO = {"simulado": "Simulado", "medido": "Medido en laboratorio"}


def figuras(casos, salida):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
        from matplotlib.ticker import FuncFormatter
    except ImportError:
        print("matplotlib no está instalado: se omiten las figuras (la tabla ya quedó guardada).")
        return
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
        "axes.edgecolor": EJE, "axes.linewidth": 0.8, "axes.labelcolor": TINTA2, "text.color": TINTA,
        "xtick.color": TINTA2, "ytick.color": TINTA2, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRILLA,
        "grid.linewidth": 0.6, "axes.axisbelow": True, "legend.frameon": False, "legend.fontsize": 8.5,
        "savefig.dpi": 220, "savefig.bbox": "tight", "savefig.pad_inches": 0.06,
    })
    formato = FuncFormatter(lambda x, _: f"{x:g}".replace(".", ","))
    leyenda = [Patch(color=SIN, label="Sin feedforward"), Patch(color=CON, label="Con feedforward")]
    origenes = [o for o in ("simulado", "medido") if any(k[0] == o for k in casos)]
    grupos = list(GRUPOS)

    def guardar(fig, nombre):
        for ext in ("png", "pdf"):
            fig.savefig(os.path.join(salida, f"{nombre}.{ext}"))
        plt.close(fig)
        print(f"Figura: {os.path.join(salida, nombre)}.png (.pdf)")

    # --- Barras: una fila por origen; columnas = error pan, error tilt, atraso pan ----------
    paneles = [("e_pan", "Error de centrado en pan [° RMS]", "pan", 2),
               ("e_tilt", "Error de centrado en tilt [° RMS]", "tilt", 3),
               ("tau_pan", "Atraso de pan [ms]", "pan", 0)]
    fig, axs = plt.subplots(len(origenes), 3, figsize=(9.6, 2.9 * len(origenes) + 0.5), squeeze=False)
    for i, origen in enumerate(origenes):
        for j, (clave, titulo, eje, dec) in enumerate(paneles):
            ax = axs[i][j]
            vals = [casos[k]["res"][clave] for k in casos if k[0] == origen and np.isfinite(casos[k]["res"][clave])]
            tope = max([abs(v) for v in vals] + [1e-9])
            for x, g in enumerate(grupos):
                for ff, dx, color in ((0, -0.19, SIN), (1, +0.19, CON)):
                    tr = casos.get((origen, g, ff))
                    v = tr["res"][clave] if tr else float("nan")
                    if not np.isfinite(v):
                        ax.text(x + dx, 0.03 * tope, "s/d", ha="center", va="bottom", color=TINTA2, fontsize=8)
                        continue
                    ax.bar(x + dx, v, width=0.34, color=color, linewidth=0)
                    ax.text(x + dx, max(v, 0) + 0.025 * tope, coma(v, dec), ha="center", va="bottom", fontsize=8.5)
            ax.axhline(0, color=EJE, linewidth=0.8)
            ax.set_xticks(range(len(grupos)))
            ax.set_xticklabels([f"Grupo {g}\n{gan(g, eje)}" for g in grupos])
            ax.set_xlim(-0.6, len(grupos) - 0.4)
            ax.set_ylim(min(0, min(vals + [0]) * 1.3), tope * 1.18)
            ax.set_title(titulo, loc="left", color=TINTA)
            ax.yaxis.set_major_formatter(formato)
            ax.grid(axis="x", visible=False)
            ax.tick_params(axis="x", length=0)
        axs[i][0].annotate(ROTULO[origen].upper(), xy=(0, 1.16), xycoords="axes fraction", fontsize=9,
                           fontweight="bold", color=TINTA2, va="bottom")
    fig.legend(handles=leyenda, loc="upper right", ncol=2, bbox_to_anchor=(0.99, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=2.2, w_pad=1.6)
    guardar(fig, BASE + "_barras")

    # --- Trazas: error de centrado en el tiempo, por origen ---------------------------------
    for origen in origenes:
        fig, axs = plt.subplots(2, len(grupos), figsize=(9.6, 4.6), squeeze=False, sharex="col", sharey="row")
        for j, g in enumerate(grupos):
            for i, (serie, eje) in enumerate((("dpan", "pan"), ("dtilt", "tilt"))):
                ax = axs[i][j]
                for ff, color in ((0, SIN), (1, CON)):
                    tr = casos.get((origen, g, ff))
                    if tr:
                        ax.plot(tr["t"], tr[serie], color=color, linewidth=1.5)
                ax.axhline(0, color=EJE, linewidth=0.8)
                ax.set_title(f"Grupo {g}: {eje}, {gan(g, eje)}", loc="left", color=TINTA)
                ax.yaxis.set_major_formatter(formato)
                if j == 0:
                    ax.set_ylabel(f"error de centrado en {eje} [°]")
                if i == 1:
                    ax.set_xlabel("tiempo de la corrida [s]")
        fig.legend(handles=leyenda, loc="upper right", ncol=2, bbox_to_anchor=(0.99, 1.0))
        fig.suptitle(ROTULO[origen].upper(), x=0.065, y=0.985, ha="left", fontsize=9, fontweight="bold", color=TINTA2)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        guardar(fig, BASE + ("_traza_sim" if origen == "simulado" else "_traza_lab"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duracion", type=float, default=40.0, help="segundos de cada simulación (por defecto 40)")
    ap.add_argument("--regimen-desde", type=float, default=8.0,
                    help="simulación: se analiza desde este segundo (por defecto 8: sin enganche ni oclusiones)")
    ap.add_argument("--descartar-lab", type=float, default=2.0,
                    help="en lo medido, segundos que se descartan al inicio de cada tramo (por defecto 2)")
    ap.add_argument("--reusar", action="store_true", help="no repetir las simulaciones que ya tienen CSV")
    ap.add_argument("--sin-simular", action="store_true", help="sólo lo medido (--lab, --lab-a, --lab-b)")
    ap.add_argument("--lab", nargs="+", metavar="CSV",
                    help="grabaciones del laboratorio (exp_*.csv del tablero): el grupo se reconoce por las ganancias del CSV")
    ap.add_argument("--lab-a", nargs="+", metavar="CSV", help="registros del laboratorio con las ganancias del grupo A")
    ap.add_argument("--lab-b", nargs="+", metavar="CSV", help="registros del laboratorio con las ganancias del grupo B")
    ap.add_argument("--detector", help="en lo medido, usar sólo los tramos con este detector (rojo, luz, avion, brillo)")
    ap.add_argument("--min-tramo", type=float, default=3.0, help="en lo medido, omitir tramos más cortos que esto [s]")
    ap.add_argument("--registros", default=os.path.join(RAIZ, "registros", "comp_ganancias"),
                    help="carpeta de los CSV crudos de la simulación")
    ap.add_argument("--salida", default=os.path.normpath(os.path.join(RAIZ, "..", "..", "docs", "figuras")),
                    help="carpeta de la tabla y las figuras")
    a = ap.parse_args()
    os.makedirs(a.salida, exist_ok=True)
    casos = reunir(a)
    if not casos:
        sys.exit("No hay casos que comparar.")
    escribir_tabla(casos, a.salida, a)
    figuras(casos, a.salida)


if __name__ == "__main__":
    main()
