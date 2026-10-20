#!/usr/bin/env python3
"""
bench_camara.py — Caracterización de la cadena de adquisición (cámara → matriz NumPy).

Para cada modo del sensor IMX219 y cada tamaño de salida, mide:
  * fps del sensor, fps efectivamente entregados a Python y cuadros perdidos
  * variación del período entre cuadros (jitter)
  * latencia desde el sensor hasta tener la matriz en Python
  * costo de copiar la matriz y de un procesamiento mínimo (punto más brillante)
  * campo de visión (FOV) efectivo y resolución angular (°/px) según el recorte real

Salidas (carpeta --salida):
  resumen.csv                 una fila por configuración (tabla para el informe)
  cuadros_<id>.csv            una fila por cuadro (histogramas de latencia / jitter)
  muestra_<id>.jpg            un cuadro de ejemplo de cada configuración
  camara.json                 propiedades de la cámara y modos del sensor

Uso (en la Raspberry Pi):
    python3 bench_camara.py                        # todos los modos, RGB888, 300 cuadros
    python3 bench_camara.py --formato YUV420       # sólo luminancia (más liviano)
    python3 bench_camara.py --exposicion 2000      # exposición fija 2 ms (útil para el láser)
    python3 bench_camara.py --modos 0 1 --cuadros 600

Definiciones de tiempo (reloj CLOCK_MONOTONIC, el mismo que usa Picamera2 internamente):
  SensorTimestamp  ≈ fin de la exposición de la primera fila (Picamera2 calcula el inicio
                     de exposición como SensorTimestamp − ExposureTime).
  lat_lectura      = t_matriz − SensorTimestamp        (lectura + ISP + entrega + copia)
  lat_centro_exp   = lat_lectura + ExposureTime/2       (desde el centro de la exposición)

Proyecto IEE2913 — Grupo 9. Bloques: Adquisición visual / Procesamiento de imagen.
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Datos ópticos NOMINALES (Waveshare Pi5-IMX219-77). Reemplazar por calibración.
# ---------------------------------------------------------------------------
PIX_UM = 1.12          # tamaño de píxel del IMX219 [µm]
F_MM = 2.85            # distancia focal nominal del lente [mm]


# ---------------------------------------------------------------------------
# Utilidades de sistema
# ---------------------------------------------------------------------------
def temperatura_cpu():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000.0
    except OSError:
        return float("nan")


def estado_throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
        return out.stdout.strip().split("=")[-1]
    except Exception:
        return "n/d"


def pct(xs, p):
    if not xs:
        return float("nan")
    return float(np.percentile(np.asarray(xs, dtype=float), p))


# ---------------------------------------------------------------------------
# Geometría: FOV y resolución angular a partir del recorte real del sensor
# ---------------------------------------------------------------------------
def geometria(crop_w, crop_h, out_w, out_h):
    """crop_w/h en píxeles NATIVOS del sensor; out_w/h = tamaño de la matriz entregada."""
    pix_mm = PIX_UM / 1000.0
    hfov = 2 * math.degrees(math.atan(crop_w * pix_mm / 2 / F_MM))
    vfov = 2 * math.degrees(math.atan(crop_h * pix_mm / 2 / F_MM))
    f_px = (F_MM / pix_mm) * (out_w / crop_w)       # focal expresada en píxeles de SALIDA
    ifov_centro = math.degrees(math.atan(1.0 / f_px))  # °/px en el centro de la imagen
    return hfov, vfov, f_px, ifov_centro


# ---------------------------------------------------------------------------
# Construcción de la lista de configuraciones a probar
# ---------------------------------------------------------------------------
def par(x):
    return int(x) // 2 * 2


def configuraciones(modos_idx, anchos_extra, max_ancho_nativo):
    """modos_idx: lista de (índice, modo). Devuelve lista de (índice, modo, (ancho, alto))."""
    confs = []
    for i, m in modos_idx:
        mw, mh = m["size"]
        salidas = []
        if mw <= max_ancho_nativo:
            salidas.append((mw, mh))                     # salida = tamaño del modo (sin escalar)
        for w in anchos_extra:
            if w < mw:
                salidas.append((par(w), par(w * mh / mw)))  # escalada por el ISP, mismo aspecto
        vistos = set()
        for s in salidas:
            if s not in vistos:
                vistos.add(s)
                confs.append((i, m, s))
    return confs


# ---------------------------------------------------------------------------
# Medición de una configuración
# ---------------------------------------------------------------------------
def medir(picam2, idx_modo, modo, salida, args, carpeta, id_conf):
    fps_modo = modo["fps"]
    if args.fps:
        fps_modo = min(fps_modo, args.fps)
    fd_us = int(round(1e6 / fps_modo))
    controles = {"FrameDurationLimits": (fd_us, fd_us)}
    if args.exposicion:
        controles.update({"AeEnable": False, "ExposureTime": int(args.exposicion),
                          "AnalogueGain": float(args.ganancia)})

    cfg = picam2.create_video_configuration(
        main={"size": salida, "format": args.formato},
        sensor={"output_size": modo["size"], "bit_depth": modo["bit_depth"]},
        controls=controles,
        buffer_count=6,
        queue=False,          # capture_request() espera un cuadro NUEVO (no uno encolado)
    )
    picam2.configure(cfg)
    real = picam2.camera_configuration()
    out_w, out_h = real["main"]["size"]
    sensor_real = real["sensor"]["output_size"]

    picam2.start()
    temp_ini = temperatura_cpu()
    t_fin = time.monotonic() + args.calentamiento
    while time.monotonic() < t_fin:           # descarta cuadros mientras converge AE/AWB
        picam2.capture_request().release()

    filas = []
    forma = nbytes = None
    for k in range(args.cuadros):
        req = picam2.capture_request()
        t_rx = time.monotonic_ns()
        arr = req.make_array("main")          # copia -> ndarray
        t_arr = time.monotonic_ns()
        md = req.get_metadata()
        req.release()

        # Procesamiento mínimo de referencia: punto más brillante (candidato a láser)
        t0 = time.monotonic_ns()
        if args.formato == "YUV420":
            img = arr[:out_h, :out_w]         # plano Y (luminancia)
        else:
            img = arr[:, :, 2]                # canal R (orden BGR en memoria)
        v, u = np.unravel_index(int(np.argmax(img)), img.shape)
        t_proc = time.monotonic_ns() - t0

        if k == 0:
            forma, nbytes = arr.shape, arr.nbytes
        ts = md["SensorTimestamp"]
        filas.append({
            "k": k,
            "sensor_ts_ns": ts,
            "t_rx_ns": t_rx,
            "t_matriz_ns": t_arr,
            "lat_lectura_ms": (t_arr - ts) / 1e6,
            "copia_ms": (t_arr - t_rx) / 1e6,
            "proc_ms": t_proc / 1e6,
            "exposicion_us": md.get("ExposureTime"),
            "ganancia": md.get("AnalogueGain"),
            "frame_dur_us": md.get("FrameDuration"),
            "scaler_crop": md.get("ScalerCrop"),
            "u_max": int(u), "v_max": int(v), "valor_max": int(img[v, u]),
        })

    # Cuadro de muestra
    if not args.sin_imagen:
        req = picam2.capture_request()
        try:
            if args.formato == "YUV420":
                from PIL import Image
                Image.fromarray(req.make_array("main")[:out_h, :out_w]).save(
                    os.path.join(carpeta, f"muestra_{id_conf}.jpg"))
            else:
                req.save("main", os.path.join(carpeta, f"muestra_{id_conf}.jpg"))
        finally:
            req.release()

    picam2.stop()
    temp_fin = temperatura_cpu()

    # --- Registro por cuadro ------------------------------------------------
    with open(os.path.join(carpeta, f"cuadros_{id_conf}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader()
        w.writerows(filas)

    # --- Métricas -----------------------------------------------------------
    ts = [r["sensor_ts_ns"] for r in filas]
    dts_ms = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
    fd_med_us = st.median([r["frame_dur_us"] for r in filas if r["frame_dur_us"]])
    fps_sensor = 1e6 / fd_med_us
    fps_entregado = (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9)
    perdidos = sum(max(0, round(dt * 1000 / fd_med_us) - 1) for dt in dts_ms)
    lat = [r["lat_lectura_ms"] for r in filas]
    exp_med_us = st.median([r["exposicion_us"] for r in filas])
    crop = filas[-1]["scaler_crop"] or modo["crop_limits"]
    cx, cy, cw, ch = crop
    hfov, vfov, f_px, ifov = geometria(cw, ch, out_w, out_h)

    return {
        "id": id_conf,
        "modo": idx_modo,
        "formato_sensor": str(modo["format"]),
        "bits": modo["bit_depth"],
        "modo_px": f"{sensor_real[0]}x{sensor_real[1]}",
        "salida_px": f"{out_w}x{out_h}",
        "formato_salida": args.formato,
        "forma_matriz": "x".join(map(str, forma)),
        "MB_cuadro": round(nbytes / 1e6, 3),
        "recorte_sensor": f"({cx},{cy}) {cw}x{ch}",
        "HFOV_deg": round(hfov, 2),
        "VFOV_deg": round(vfov, 2),
        "f_px": round(f_px, 1),
        "IFOV_deg_px": round(ifov, 4),
        "fps_max_modo": modo["fps"],
        "fps_sensor": round(fps_sensor, 2),
        "fps_entregado": round(fps_entregado, 2),
        "cuadros_perdidos": perdidos,
        "N": len(filas),
        "dt_med_ms": round(st.median(dts_ms), 3),
        "jitter_std_ms": round(st.pstdev(dts_ms), 3),
        "dt_p99_ms": round(pct(dts_ms, 99), 3),
        "lat_med_ms": round(st.median(lat), 2),
        "lat_p95_ms": round(pct(lat, 95), 2),
        "lat_max_ms": round(max(lat), 2),
        "lat_centro_exp_med_ms": round(st.median(lat) + exp_med_us / 2000, 2),
        "copia_med_ms": round(st.median([r["copia_ms"] for r in filas]), 3),
        "proc_med_ms": round(st.median([r["proc_ms"] for r in filas]), 3),
        "exposicion_med_us": int(exp_med_us),
        "temp_ini_C": temp_ini,
        "temp_fin_C": temp_fin,
        "throttled": estado_throttled(),
    }


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Benchmark de modos/fps/latencia de la cámara")
    ap.add_argument("--formato", choices=["RGB888", "YUV420"], default="RGB888")
    ap.add_argument("--cuadros", type=int, default=300, help="cuadros medidos por configuración")
    ap.add_argument("--calentamiento", type=float, default=1.5, help="segundos descartados al inicio")
    ap.add_argument("--modos", type=int, nargs="*", help="índices de modos a probar (por defecto todos)")
    ap.add_argument("--anchos", type=int, nargs="*", default=[640, 320],
                    help="anchos de salida escalados por el ISP a probar además del nativo")
    ap.add_argument("--max-ancho-nativo", type=int, default=1920,
                    help="no probar salida sin escalar para modos más anchos que esto")
    ap.add_argument("--fps", type=float, help="limitar fps (por defecto: máximo de cada modo)")
    ap.add_argument("--exposicion", type=int, help="exposición fija en µs (desactiva AE)")
    ap.add_argument("--ganancia", type=float, default=1.0, help="ganancia analógica si --exposicion")
    ap.add_argument("--salida", default=None, help="carpeta de resultados")
    ap.add_argument("--sin-imagen", action="store_true", help="no guardar cuadros de muestra")
    args = ap.parse_args()

    carpeta = args.salida or datetime.now().strftime("bench_%Y%m%d_%H%M%S")
    os.makedirs(carpeta, exist_ok=True)

    picam2 = Picamera2()
    props = picam2.camera_properties
    print(f"Cámara: {props.get('Model')}  arreglo {props.get('PixelArraySize')}")
    print("Leyendo modos del sensor (reconfigura la cámara varias veces)...")
    modos = picam2.sensor_modes
    for i, m in enumerate(modos):
        print(f"  modo {i}: {m['size'][0]}x{m['size'][1]}  {m['bit_depth']} bits  "
              f"{m['fps']:.2f} fps máx  recorte {m['crop_limits']}")

    with open(os.path.join(carpeta, "camara.json"), "w") as f:
        json.dump({
            "fecha": datetime.now().isoformat(timespec="seconds"),
            "propiedades": {k: str(v) for k, v in props.items()},
            "modos": [{k: str(v) for k, v in m.items()} for m in modos],
            "optica_nominal": {"pix_um": PIX_UM, "f_mm": F_MM},
            "reloj": "CLOCK_MONOTONIC (time.monotonic_ns)",
            "diferencia_boottime_menos_monotonic_ns":
                time.clock_gettime_ns(time.CLOCK_BOOTTIME) - time.monotonic_ns(),
            "args": vars(args),
        }, f, indent=2, ensure_ascii=False)

    sel = [(i, m) for i, m in enumerate(modos) if args.modos is None or i in args.modos]
    confs = configuraciones(sel, args.anchos, args.max_ancho_nativo)

    resumen = []
    for n, (i, m, s) in enumerate(confs):
        id_conf = f"m{i}_{s[0]}x{s[1]}"
        print(f"\n[{n+1}/{len(confs)}] modo {i} {m['size'][0]}x{m['size'][1]} -> salida {s[0]}x{s[1]} ...",
              flush=True)
        try:
            r = medir(picam2, i, m, s, args, carpeta, id_conf)
        except Exception as e:  # una configuración fallida no detiene el resto
            print(f"   ERROR: {e}")
            try:
                picam2.stop()
            except Exception:
                pass
            continue
        resumen.append(r)
        print(f"   matriz {r['forma_matriz']} ({r['MB_cuadro']} MB) | FOV {r['HFOV_deg']}°x{r['VFOV_deg']}° "
              f"| {r['IFOV_deg_px']} °/px | fps sensor {r['fps_sensor']} entregado {r['fps_entregado']} "
              f"(perdidos {r['cuadros_perdidos']}) | lat med {r['lat_med_ms']} ms p95 {r['lat_p95_ms']} ms "
              f"| proc {r['proc_med_ms']} ms | {r['temp_fin_C']:.1f} °C thr={r['throttled']}")

    picam2.close()
    if not resumen:
        print("No se midió ninguna configuración.")
        sys.exit(1)

    ruta = os.path.join(carpeta, "resumen.csv")
    with open(ruta, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(resumen[0].keys()))
        w.writeheader()
        w.writerows(resumen)

    print(f"\nListo. Resultados en: {os.path.abspath(carpeta)}")
    print(f"{'config':>18} {'FOV H°':>7} {'°/px':>7} {'fps':>7} {'perd':>5} {'lat ms':>7} {'p95':>6} {'MB':>6}")
    for r in resumen:
        print(f"{r['id']:>18} {r['HFOV_deg']:>7} {r['IFOV_deg_px']:>7} {r['fps_entregado']:>7} "
              f"{r['cuadros_perdidos']:>5} {r['lat_med_ms']:>7} {r['lat_p95_ms']:>6} {r['MB_cuadro']:>6}")


if __name__ == "__main__":
    main()
