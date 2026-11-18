#!/usr/bin/env python3
"""
analizar_registro.py — ¿dónde está el cuello de botella? Lee el CSV de una o más corridas de
main.py (carpeta registros/, o /registro.csv desde el tablero) y entrega el presupuesto MEDIDO:

  1. cuadros: tasa real, regularidad y cuadros perdidos
  2. latencias por etapa (captura, detección, total) y horizonte del predictor
  3. detección: porcentaje de cuadros válidos y motivos de rechazo
  4. supervisor: tiempo en cada estado y caídas a PÉRDIDA
  5. predictor: mediciones rechazadas por el gating
  6. error de centrado medido en SEGUIMIENTO (lo que pide CTL-5)
  7. atraso de la plataforma respecto del objetivo, en milisegundos
  8. lazo interno (ESP32): error θ_ref − θ_med, saturación de ω_cmd y fallas

y, al final, una tabla que COMPARA todo lo analizado (p. ej. con y sin feedforward).

TRAMOS: el detector y el feedforward se pueden cambiar desde el tablero con el lazo corriendo.
Cada CSV se parte solo en tramos donde esos dos ajustes no cambian, y cada tramo se analiza
por separado. Así no importa si la comparación se hizo en dos corridas o en una sola.

Uso:
    python3 herramientas/analizar_registro.py registros/sin_ff/*.csv registros/con_ff/*.csv
    python3 herramientas/analizar_registro.py corrida.csv --detector avion      # sólo los tramos con ese detector
    python3 herramientas/analizar_registro.py corrida.csv --resumen             # sólo la tabla comparativa
    python3 herramientas/analizar_registro.py corrida.csv --desde 5 --hasta 40
"""
#* Punto 7: en SEGUIMIENTO el error de centrado (dpan_deg) es "objetivo − eje óptico". Si la
#* plataforma va atrasada un tiempo τ, ese error vale ≈ τ · (velocidad del objetivo). τ se
#* obtiene por regresión de dpan_deg contra la velocidad de pan_obj. Con el PI solo se espera
#* τ ≈ 1/Kp (333 ms en pan, 100 ms en tilt con las ganancias de arranque); con feedforward, cerca de 0.
import argparse
import collections
import csv
import os

import numpy as np

IFOV = 0.1154        # °/px a 640 px con el modo 1640x1232 (pt/geometria.py)


def num(fila, clave):
    try:
        return float(fila[clave])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def pct(x, q):
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else float("nan")


def rms(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def linea(nombre, x, unidad="", dec=2):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if not x.size:
        print(f"  {nombre:<34s} sin datos")
        return
    print(f"  {nombre:<34s} mediana {np.median(x):8.{dec}f}   p95 {np.percentile(x, 95):8.{dec}f}   "
          f"p99 {np.percentile(x, 99):8.{dec}f}   máx {x.max():8.{dec}f}  {unidad}")


def ff_de_la_fila(fila):
    """1/0 según la columna ff. En registros anteriores al 17-11 no existe: se deduce de ω_ff."""
    if fila.get("ff") in ("0", "1"):
        return int(fila["ff"])
    return None


def partir_en_tramos(filas):
    """[(i0, i1, detector, ff)], con i1 exclusivo: tramos seguidos con el mismo detector y feedforward."""
    tramos, i0 = [], 0
    clave = lambda f: (f.get("detector", ""), ff_de_la_fila(f))
    for i in range(1, len(filas) + 1):
        if i == len(filas) or clave(filas[i]) != clave(filas[i0]):
            tramos.append((i0, i) + clave(filas[i0]))
            i0 = i
    return tramos


def atraso_ms(err, obj, t, sel):
    """τ [ms] tal que error ≈ τ · velocidad del objetivo (regresión), y cuánta varianza explica."""
    if len(t) < 60:
        return None
    v = np.gradient(obj, t)
    v = np.convolve(np.nan_to_num(v), np.ones(5) / 5, mode="same")
    ok = sel & np.isfinite(err) & np.isfinite(obj) & (np.abs(v) < 400)
    if ok.sum() < 50 or np.std(v[ok]) < 0.5:
        return None
    e, vv = err[ok] - err[ok].mean(), v[ok] - v[ok].mean()
    tau = float(e @ vv / (vv @ vv))
    r2 = float(1 - np.sum((e - tau * vv) ** 2) / np.sum(e ** 2)) if np.sum(e ** 2) > 0 else 0.0
    return tau * 1e3, r2, float(np.sqrt(np.mean(v[ok] ** 2)))


def analizar_tramo(filas, t, titulo, detalle=True):
    """Imprime el informe de un tramo y devuelve su resumen para la tabla comparativa."""
    col = lambda c: np.array([num(f, c) for f in filas])
    txt = lambda c: [f.get(c, "") for f in filas]
    n = len(filas)
    dur = t[-1] - t[0]
    est = txt("estado")
    seg = np.array([e == "SEGUIMIENTO" for e in est])
    val = col("valido") == 1
    sel = seg & val
    ff = ff_de_la_fila(filas[0])
    if ff is None:                                # registro antiguo: se deduce de ω_ff
        wff = np.abs(np.nan_to_num(col("wff_pan"))) + np.abs(np.nan_to_num(col("wff_tilt")))
        ff = int(seg.any() and (wff[seg] > 0).mean() > 0.2)
    caidas = sum(1 for a, b in zip(est, est[1:]) if a == "SEGUIMIENTO" and b != "SEGUIMIENTO")
    atrasos = {eje: atraso_ms(col(ce), col(co), t, sel)
               for eje, ce, co in (("pan", "dpan_deg", "pan_obj"), ("tilt", "dtilt_deg", "tilt_obj"))}
    exp, gan, lam = col("exp_us"), col("ganancia"), col("lam")
    res = {"titulo": titulo, "detector": filas[0].get("detector", "") or "?", "ff": ff, "dur": dur,
           "valido": 100 * val.mean(), "seguimiento": 100 * seg.mean(),
           "caidas_min": caidas / max(dur, 1e-9) * 60,
           "e_pan": rms(col("dpan_deg")[sel]), "e_tilt": rms(col("dtilt_deg")[sel]),
           "tau_pan": atrasos["pan"][0] if atrasos["pan"] else float("nan"),
           "tau_tilt": atrasos["tilt"][0] if atrasos["tilt"] else float("nan"),
           #* Velocidad RMS del objetivo: el error por atraso es ≈ atraso · velocidad, así que dos
           #* corridas con movimientos distintos se comparan por el atraso [ms], no por los grados.
           "v_pan": atrasos["pan"][2] if atrasos["pan"] else float("nan"),
           "v_tilt": atrasos["tilt"][2] if atrasos["tilt"] else float("nan"),
           "lat": pct(col("lat_total_ms"), 50)}
    if not detalle:
        return res

    print(f"\n=== {titulo} ===")
    ajustes = [f"detector {res['detector']}", f"feedforward {'ACTIVO' if ff else 'apagado'}"]
    if np.isfinite(exp).any():
        ajustes.append(f"exposición {np.nanmedian(exp):.0f} µs")
    if np.isfinite(gan).any():
        ajustes.append(f"ganancia {np.nanmedian(gan):.2f}")
    if np.isfinite(lam).any():
        ajustes.append(f"λ {np.nanmedian(lam):.2f}")
    print(f"  {n} cuadros, {dur:.1f} s | " + ", ".join(ajustes))

    print("\n1. Cuadros")
    dt = np.diff(t) * 1e3
    T = float(np.median(dt))
    print(f"  tasa real {1e3 / T:.1f} fps (período mediano {T:.2f} ms) | cuadros perdidos (hueco > 1,5 períodos): "
          f"{int(np.sum(dt > 1.5 * T))} de {n}")
    linea("período entre cuadros", dt, "ms")

    print("\n2. Latencias (desde el inicio de la exposición)")
    linea("captura: exposición -> matriz", col("lat_captura_ms"), "ms")
    linea("detección", col("t_det_ms"), "ms", 3)
    linea("total: exposición -> consigna", col("lat_total_ms"), "ms")
    linea("horizonte del predictor (h_ms)", col("h_ms"), "ms")

    print("\n3. Detección")
    print(f"  cuadros con objetivo válido: {res['valido']:.1f} %")
    for motivo, c in collections.Counter(mo for mo, v in zip(txt("motivo"), val) if not v).most_common(4):
        print(f"    {100 * c / n:5.1f} %  {motivo or '(sin motivo)'}")

    print("\n4. Supervisor")
    for e, c in collections.Counter(est).most_common():
        print(f"  {e:<16s} {100 * c / n:5.1f} % del tiempo")
    print(f"  salidas de SEGUIMIENTO: {caidas}  ({res['caidas_min']:.1f} por minuto)")

    print("\n5. Predictor (gating), sólo en SEGUIMIENTO")
    for eje, c in (("pan", "est_u"), ("tilt", "est_w")):
        r = np.array([x == "rechazada" for x in txt(c)])
        print(f"  {eje:<5s} mediciones rechazadas: {100 * r[seg].mean() if seg.any() else float('nan'):.2f} %")
    #* Reinicios del estimador: desde el 17-11 ya no pasan por PÉRDIDA, así que se cuentan aquí.
    rein = np.array([a == "reinicio" or b == "reinicio" for a, b in zip(txt("est_u"), txt("est_w"))])
    print(f"  reinicios del estimador (varios rechazos seguidos): {int(rein[seg].sum())}  ({rein[seg].sum() / max(dur, 1e-9) * 60:.1f} por minuto)")
    d = col("gating_d")[seg] if seg.any() else np.array([])
    if np.isfinite(d).sum() > 10:
        #* gating_d = innovación en veces el umbral k·max(σ, piso). Sirve para ajustar el PISO:
        #* con el objetivo bien detectado, d debería quedar lejos de 1.
        print(f"  innovación / umbral (gating_d): mediana {pct(d, 50):.2f}   p95 {pct(d, 95):.2f}   p99 {pct(d, 99):.2f}   "
              f"máx {np.nanmax(d):.2f}   (rechazo si > 1)")
        print(f"  σ de la innovación: pan mediana {pct(col('sigma_pan')[seg], 50):.3f}°  p95 {pct(col('sigma_pan')[seg], 95):.3f}°   |   "
              f"tilt mediana {pct(col('sigma_tilt')[seg], 50):.3f}°  p95 {pct(col('sigma_tilt')[seg], 95):.3f}°")
        if pct(d, 99) > 0.7:
            print("  AVISO: p99 de gating_d sobre 0,7: el umbral está apretado para este detector. Si los rechazos son "
                  "saltos reales del centroide, subir el piso del gating (tablero); si son detecciones falsas, dejarlo.")

    print("\n6. Error de centrado medido en SEGUIMIENTO (objetivo respecto del centro de la imagen)")
    if sel.sum() > 10:
        for eje, c in (("pan", "dpan_deg"), ("tilt", "dtilt_deg")):
            e = col(c)[sel]
            print(f"  {eje:<5s} RMS {rms(e):6.3f}°  ({rms(e) / IFOV:5.1f} px)   p95 {pct(np.abs(e), 95):6.3f}°   "
                  f"máx {np.nanmax(np.abs(e)):6.3f}°   sesgo {np.nanmean(e):+.3f}°")
    else:
        print("  sin cuadros en SEGUIMIENTO con detección válida")

    print("\n7. Atraso de la plataforma respecto del objetivo")
    for eje in ("pan", "tilt"):
        r = atrasos[eje]
        if r is None:
            print(f"  {eje:<5s} no se puede estimar (el objetivo casi no se movió, o hay pocos datos)")
        else:
            print(f"  {eje:<5s} atraso ≈ {r[0]:6.0f} ms   (explica el {100 * max(r[1], 0):.0f} % del error; "
                  f"velocidad RMS del objetivo {r[2]:.1f} °/s)")

    print("\n8. Lazo interno (telemetría del ESP32)")
    if np.isfinite(col("esp32_pan_med")).sum() < 10:
        print("  sin telemetría en este tramo")
    else:
        for eje in ("pan", "tilt"):
            e = col(f"esp32_{eje}_ref") - col(f"esp32_{eje}_med")
            w = np.abs(col(f"esp32_w_{eje}"))
            print(f"  {eje:<5s} |θ_ref − θ_med| RMS {rms(e[seg]):6.3f}°   |ω_cmd| p95 {pct(w, 95):6.1f} °/s   "
                  f"en saturación (> 170 °/s): {100 * np.nanmean(w > 170):.1f} % del tiempo")
        print("  estados del ESP32: " + ", ".join(f"{e or '?'} {100 * c / n:.0f} %" for e, c in collections.Counter(txt("esp32_estado")).most_common()))
        fallas = sorted(set(x for x in txt("esp32_fallas") if x not in ("", "0")))
        print(f"  fallas reportadas: {', '.join(fallas) if fallas else 'ninguna'}")
    return res


def analizar_archivo(ruta, a):
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 20:
        print(f"\n{ruta}: muy pocas filas ({len(filas)})")
        return []
    t = (np.array([num(f, "t_exp_ns") for f in filas]) - num(filas[0], "t_exp_ns")) * 1e-9
    nombre = os.path.join(os.path.basename(os.path.dirname(ruta)), os.path.basename(ruta)).lstrip(os.sep)
    resumenes, omitidos = [], []
    for i0, i1, det, ff in partir_en_tramos(filas):
        #* Recorte pedido por el usuario, dentro del tramo.
        j0 = max(i0, int(np.searchsorted(t, a.desde)))
        j1 = min(i1, int(np.searchsorted(t, a.hasta, side="right"))) if a.hasta else i1
        if j1 - j0 < 2:
            continue
        rango = f"{t[j0]:.1f}–{t[j1 - 1]:.1f} s"
        if a.detector and det != a.detector:
            omitidos.append(f"{rango} (detector {det or '?'})")
            continue
        if a.ff is not None and ff is not None and ff != a.ff:
            omitidos.append(f"{rango} (feedforward {'activo' if ff else 'apagado'})")
            continue
        if t[j1 - 1] - t[j0] < a.min_tramo:
            omitidos.append(f"{rango} (dura menos de {a.min_tramo:g} s)")
            continue
        resumenes.append(analizar_tramo(filas[j0:j1], t[j0:j1], f"{nombre}  [{rango}]", detalle=not a.resumen))
    if omitidos:
        print(f"\n{nombre}: tramos no analizados: " + "; ".join(omitidos))
    return resumenes


def tabla_comparativa(resumenes):
    print("\n" + "=" * 118)
    print("COMPARACIÓN (error de centrado y atraso: sólo cuadros en SEGUIMIENTO con detección válida)")
    for i, r in enumerate(resumenes, 1):
        print(f"  T{i} = {r['titulo']}")
    print(f"\n{'tramo':<6s}{'detector':>9s}{'feedforward':>13s}{'duración':>10s}{'válidos':>9s}{'error pan':>19s}{'error tilt':>19s}{'atraso pan':>12s}{'atraso tilt':>13s}")
    for i, r in enumerate(resumenes, 1):
        e = lambda x: f"{x:.2f}° ({x / IFOV:.1f} px)" if np.isfinite(x) else "—"
        tau = lambda x: f"{x:.0f} ms" if np.isfinite(x) else "—"
        print(f"{'T' + str(i):<6s}{r['detector']:>9s}{'sí' if r['ff'] else 'no':>13s}{r['dur']:8.0f} s{r['valido']:7.0f} %"
              f"{e(r['e_pan']):>19s}{e(r['e_tilt']):>19s}{tau(r['tau_pan']):>12s}{tau(r['tau_tilt']):>13s}")
    con = [r for r in resumenes if r["ff"]]
    sin = [r for r in resumenes if not r["ff"]]
    if len(con) == 1 and len(sin) == 1:
        s, c = sin[0], con[0]
        print("\nCon feedforward respecto de sin feedforward:")
        for eje in ("pan", "tilt"):
            es, ec = s[f"e_{eje}"], c[f"e_{eje}"]
            if np.isfinite(es) and np.isfinite(ec) and ec > 0:
                if ec < 0.85 * es:
                    juicio = f"{es / ec:.1f} veces menor"
                elif ec > 1.15 * es:
                    juicio = "EMPEORA: revisar escala (micropasos) y signo"
                else:
                    juicio = "sin cambio apreciable"
                print(f"  {eje:<5s} error de centrado RMS {es:.2f}° -> {ec:.2f}°  ({juicio})")
        if abs(s["dur"] - c["dur"]) > 0.5 * max(s["dur"], c["dur"]):
            print("  Aviso: los dos tramos duran muy distinto; la comparación vale si el objetivo se movió parecido en ambos.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="uno o más registros de main.py")
    ap.add_argument("--desde", type=float, default=0.0, help="segundo inicial a considerar en cada archivo")
    ap.add_argument("--hasta", type=float, default=0.0, help="segundo final (0 = hasta el final)")
    ap.add_argument("--detector", help="analizar sólo los tramos con este detector (rojo, luz, avion, brillo)")
    ap.add_argument("--ff", type=int, choices=(0, 1), help="analizar sólo los tramos sin (0) o con (1) feedforward")
    ap.add_argument("--min-tramo", type=float, default=3.0, help="se omiten los tramos más cortos que esto [s]")
    ap.add_argument("--resumen", action="store_true", help="sólo la tabla comparativa")
    a = ap.parse_args()
    resumenes = []
    for ruta in a.csv:
        resumenes += analizar_archivo(ruta, a)
    if resumenes:
        tabla_comparativa(resumenes)
    else:
        print("\nNingún tramo para analizar con esos filtros.")


if __name__ == "__main__":
    main()
