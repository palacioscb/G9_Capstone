#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
medir_avion.py - Recolector de métricas para diseñar el detector del AVIÓN ROJO de juguete.

Independiente del esqueleto (solo NumPy + Picamera2; OpenCV es opcional y solo para la
ventana). Pensado para correr en la Raspberry Pi con la cámara, sin ESP32 ni motores.

QUÉ HACE
  1. FONDO   : durante --calibrar-fondo segundos mide el plano V (rojo) SIN el avión en
               cuadro. De ahí sale el umbral de referencia (máximo del fondo + margen).
  2. REGISTRO: durante --duracion segundos mueves el avión (con el palo, en todas las
               orientaciones). Por cada cuadro se mide, con una segmentación permisiva
               ("todo lo rojo sobre el umbral"): área, caja, aspecto, relleno, ejes,
               elongación, ángulo, V/Y del objeto y del entorno, brillo saturado,
               desplazamiento entre cuadros, velocidad, cambio de área, pérdidas...
  3. A la vez corre una EMULACIÓN de DetectorLaserRojo (misma lógica que deteccion.py)
     con los parámetros --det-*, y la compara con la referencia: aciertos, falsos
     positivos, motivos de rechazo, error de centroide y tiempo de proceso.
  4. Al final escribe: cuadros.csv, resumen.txt (con parámetros sugeridos listos para
     pegar en parametros.py) y cuadros.npz (Y y V de una muestra de cuadros, para seguir
     afinando offline en el PC).

ETIQUETAS (para separar estadísticas por orientación). Cámbialas en cualquier momento:
  ventana abierta : teclas 0-5          sin ventana (SSH): escribe el número y Enter
  0 sin_avion  1 frente  2 lado  3 arriba  4 atras  5 libre
  Empieza en "sin_avion" tras el fondo: sirve para medir falsos positivos. Cuando el
  avión esté en cuadro, pulsa 1..5. Recomendación: 20-30 s por orientación y luego
  movimiento "libre" aleatorio; termina con unos segundos de "sin_avion".

USO
  python3 medir_avion.py                                   # cámara real, 60 s
  python3 medir_avion.py --sin-ventana --duracion 90       # por SSH
  python3 medir_avion.py --exposicion 4000 --ganancia 2    # probar otra exposición
  python3 medir_avion.py --fuente sintetica --sin-ventana  # prueba en PC, sin cámara

IMPORTANTE: mantén el avión FUERA del cuadro durante la fase de fondo y no cambies la
iluminación después; si lo haces, repite la medición.
"""
import argparse
import csv
import json
import math
import re
import select
import sys
import time
from collections import Counter, namedtuple
from pathlib import Path

import numpy as np

V_NEUTRO = 128
ETIQUETAS = {"0": "sin_avion", "1": "frente", "2": "lado", "3": "arriba",
             "4": "atras", "5": "libre"}
Res = namedtuple("Res", "valido u w pico area motivo t_ms")

COLUMNAS = [
    "i", "t_s", "dt_ms", "etiqueta",
    # referencia (segmentación permisiva sobre el plano V)
    "ref_ok", "ref_u", "ref_w", "npix_v", "area_y", "bbox_w_y", "bbox_h_y", "aspecto",
    "relleno", "eje_mayor_y", "eje_menor_y", "elongacion", "angulo_deg",
    "v_pico", "v_medio", "v_anillo", "u_medio", "y_medio", "y_max", "y_max7", "y_anillo",
    "contraste_y", "sat_frac",
    # movimiento
    "salto_px", "frames_desde", "vel_px_s", "ratio_area", "perdidos_previos",
    # detector emulado
    "det_ok", "det_u", "det_w", "det_area", "det_pico", "det_motivo", "det_t_ms",
    "err_det_ref_px", "clase",
]


# =============================================================================
# Fuentes de imagen: devuelven (Y, U, V, t_ns); U y V a media resolución
# =============================================================================
class FuenteCamara:
    def __init__(self, a):
        from picamera2 import Picamera2, MappedArray
        from libcamera import controls as lc
        self.MappedArray = MappedArray
        self.W, self.H = a.ancho, a.alto
        dur_us = int(1e6 / a.fps)
        if a.exposicion > dur_us:
            print(f"AVISO: exposición {a.exposicion} us > periodo de cuadro {dur_us} us; "
                  f"el fps real será menor.")
        ctrl = {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": int(a.exposicion),
            "AnalogueGain": float(a.ganancia),
            "FrameDurationLimits": (dur_us, dur_us),
            "NoiseReductionMode": lc.draft.NoiseReductionModeEnum.Off,
        }
        if a.ganancias_color:
            ctrl["ColourGains"] = tuple(a.ganancias_color)    # (rojo, azul)
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(
            main={"size": (self.W, self.H), "format": "YUV420"},
            buffer_count=4, queue=False, controls=ctrl)
        self.cam.configure(cfg)
        self.cam.start()
        print("Modo de sensor:", self.cam.camera_configuration()["sensor"])

    def leer(self, avion=True):
        W, H = self.W, self.H
        req = self.cam.capture_request()
        try:
            ts = req.get_metadata().get("SensorTimestamp")
            with self.MappedArray(req, "main") as ma:
                arr = ma.array
                if arr.shape != (H * 3 // 2, W):
                    raise RuntimeError(
                        f"Forma YUV420 inesperada {arr.shape}; use un ancho múltiplo de 64 "
                        f"(hay relleno por fila).")
                y = arr[:H].copy()
                u = arr[H:H + H // 4].reshape(H // 2, W // 2).copy()
                v = arr[H + H // 4:].reshape(H // 2, W // 2).copy()
        finally:
            req.release()
        return y, u, v, int(ts) if ts else time.monotonic_ns()

    def cerrar(self):
        try:
            self.cam.stop()
        except Exception:
            pass


class FuenteSintetica:
    """Avión rojo elíptico con movimiento aleatorio suave (Ornstein-Uhlenbeck) y forma que
    cambia con la 'orientación'. Sirve para probar este script en el PC."""

    def __init__(self, a):
        self.W, self.H, self.fps = a.ancho, a.alto, a.fps
        self.rng = np.random.default_rng(1)
        self.pos = np.array([self.W / 2, self.H / 2])
        self.vel = np.zeros(2)
        self.fase = 0.0
        self.ang = 0.0
        self.a_ = 25.0
        self.b_ = 10.0
        self.n = 0

    def _mascara(self, h, w, esc):
        cx, cy = self.pos
        r = max(self.a_, self.b_) + 2
        j0 = max(int(math.floor((cx - r) / esc)), 0)
        j1 = min(int(math.floor((cx + r) / esc)) + 2, w)
        i0 = max(int(math.floor((cy - r) / esc)), 0)
        i1 = min(int(math.floor((cy + r) / esc)) + 2, h)
        if j0 >= j1 or i0 >= i1:
            return i0, i1, j0, j1, np.zeros((0, 0), bool)
        jj, ii = np.meshgrid(np.arange(j0, j1), np.arange(i0, i1))
        x = esc * jj + (esc - 1) / 2 - cx
        yy = esc * ii + (esc - 1) / 2 - cy
        c, s = math.cos(self.ang), math.sin(self.ang)
        xr, yr = x * c + yy * s, -x * s + yy * c
        return i0, i1, j0, j1, (xr / self.a_) ** 2 + (yr / self.b_) ** 2 <= 1.0

    def leer(self, avion=True):
        W, H, dt, rng = self.W, self.H, 1.0 / self.fps, self.rng
        self.vel += (-self.vel / 0.5) * dt + 300.0 * math.sqrt(dt) * rng.standard_normal(2)
        self.pos += self.vel * dt
        for k, lim in ((0, W), (1, H)):                   # rebote suave en los bordes
            if self.pos[k] < 40 or self.pos[k] > lim - 40:
                self.vel[k] = -self.vel[k]
                self.pos[k] = min(max(self.pos[k], 40), lim - 40)
        self.fase += 1.5 * dt * rng.standard_normal() * 6
        self.ang += 2.0 * dt * rng.standard_normal() * 3
        self.a_ = 26 + 12 * math.cos(self.fase)
        self.b_ = 9 + 7 * abs(math.sin(self.fase))

        y = (100 + 2 * rng.standard_normal((H, W))).clip(0, 255).astype(np.uint8)
        u = (128 + 1.5 * rng.standard_normal((H // 2, W // 2))).clip(0, 255).astype(np.uint8)
        v = (128 + 1.5 * rng.standard_normal((H // 2, W // 2))).clip(0, 255).astype(np.uint8)
        v[200:210, 30:45] = 142                           # un "distractor" rojizo fijo
        if avion:
            i0, i1, j0, j1, m = self._mascara(H, W, 1)
            if m.size:
                y[i0:i1, j0:j1][m] = 70
            i0, i1, j0, j1, m = self._mascara(H // 2, W // 2, 2)
            if m.size:
                v[i0:i1, j0:j1][m] = (190 + 3 * rng.standard_normal(m.sum())).clip(0, 255)
                u[i0:i1, j0:j1][m] = 100
        self.n += 1
        return y, u, v, int(self.n * 1e9 / self.fps)

    def cerrar(self):
        pass


# =============================================================================
# Referencia: segmentación permisiva "todo lo rojo sobre el umbral" en el plano V
# =============================================================================
def segmentar_ref(y, u, v, y2, thr, min_pix):
    mask = v > thr
    n = int(np.count_nonzero(mask))
    if n < min_pix:
        return None
    ys, xs = np.nonzero(mask)
    vals = v[ys, xs].astype(np.float32)
    wgt = vals - V_NEUTRO
    sw = float(wgt.sum())
    mx = float((wgt * xs).sum() / sw)
    my = float((wgt * ys).sum() / sw)
    x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
    wv, hv = x1 - x0 + 1, y1 - y0 + 1

    # forma por momentos (en px de V, +1/12 por el tamaño del píxel)
    dx, dy = xs - xs.mean(), ys - ys.mean()
    cxx, cyy, cxy = float((dx * dx).mean()) + 1 / 12, float((dy * dy).mean()) + 1 / 12, float((dx * dy).mean())
    tr, det = cxx + cyy, cxx * cyy - cxy * cxy
    disc = math.sqrt(max(tr * tr / 4 - det, 0.0))
    l1, l2 = tr / 2 + disc, max(tr / 2 - disc, 1e-6)
    ang = 0.5 * math.degrees(math.atan2(2 * cxy, cxx - cyy))

    # entorno inmediato (anillo de 6 px de V alrededor de la caja)
    m = 6
    a0, a1 = max(x0 - m, 0), min(x1 + m + 1, v.shape[1])
    b0, b1 = max(y0 - m, 0), min(y1 + m + 1, v.shape[0])
    anillo = ~mask[b0:b1, a0:a1]
    v_ring = float(v[b0:b1, a0:a1][anillo].mean()) if anillo.any() else float("nan")
    y_ring = float(y2[b0:b1, a0:a1][anillo].mean()) if anillo.any() else float("nan")

    yv = y2[ys, xs]
    u_real, w_real = 2.0 * mx + 0.5, 2.0 * my + 0.5       # a px del plano Y
    ui, wi = int(u_real), int(w_real)
    y_max7 = int(y[max(wi - 3, 0): wi + 4, max(ui - 3, 0): ui + 4].max())
    return dict(
        u=u_real, w=w_real, npix_v=n, area_y=4 * n, bbox_w_y=2 * wv, bbox_h_y=2 * hv,
        aspecto=wv / hv, relleno=n / (wv * hv), eje_mayor_y=8 * math.sqrt(l1),
        eje_menor_y=8 * math.sqrt(l2), elongacion=math.sqrt(l1 / l2), angulo_deg=ang,
        v_pico=int(vals.max()), v_medio=float(vals.mean()), v_anillo=v_ring,
        u_medio=float(u[ys, xs].mean()), y_medio=float(yv.mean()), y_max=int(yv.max()),
        y_max7=y_max7, y_anillo=y_ring, contraste_y=float(yv.mean()) - y_ring,
        sat_frac=float((yv >= 250).mean()),
    )


# =============================================================================
# Emulación de DetectorLaserRojo (misma lógica que deteccion.py)
# =============================================================================
class DetectorRojoEmulado:
    def __init__(self, p, paciencia=5):
        self.p = p
        self.ultimo = None
        self.paciencia = paciencia      # cuadros inválidos antes de volver a búsqueda global
        self.fallos = 0

    def procesar(self, y, v):
        t0 = time.perf_counter()
        r = self._procesar(y, v)
        t_ms = (time.perf_counter() - t0) * 1e3
        if r[0]:
            self.ultimo, self.fallos = (r[1], r[2]), 0
        else:
            self.fallos += 1
            if self.fallos >= self.paciencia:
                self.ultimo, self.fallos = None, 0
        return Res(*r, t_ms)

    def _procesar(self, y, v):
        p = self.p
        h, w = v.shape
        x0, y0, x1, y1 = 0, 0, w, h
        if self.ultimo is not None:
            m = max(int(p.ventana) // 4, int(p.bloque))
            uc = int(round((self.ultimo[0] - 0.5) / 2))
            wc = int(round((self.ultimo[1] - 0.5) / 2))
            x0, x1 = max(0, uc - m), min(w, uc + m)
            y0, y1 = max(0, wc - m), min(h, wc + m)
        roi = v[y0:y1, x0:x1]
        nan = float("nan")
        if roi.size == 0:
            return (False, nan, nan, 0, 0, "ventana vacía")
        pico_v = int(roi.max())
        if pico_v < p.v_umbral:
            return (False, nan, nan, pico_v, 0, "sin rojo sobre umbral")
        b = int(p.bloque)
        hb, wb = roi.shape[0] // b, roi.shape[1] // b
        if hb == 0 or wb == 0:
            cy, cx = divmod(int(np.argmax(roi)), roi.shape[1])
        else:
            suma = roi[:hb * b, :wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.uint32)
            by, bx = divmod(int(suma.argmax()), wb)
            cx, cy = bx * b + b // 2, by * b + b // 2
        rad = (3 * b) // 2
        a0, a1 = max(0, cx - rad), min(roi.shape[1], cx + rad)
        c0, c1 = max(0, cy - rad), min(roi.shape[0], cy + rad)
        ven = roi[c0:c1, a0:a1]
        pico = int(ven.max())
        if pico < p.v_umbral:
            return (False, nan, nan, pico, 0, "bloque rojo sin pico")
        corte = V_NEUTRO + p.frac_corte * (pico - V_NEUTRO)
        peso = ven.astype(np.float32) - corte
        peso[peso < 0] = 0.0
        area_v = int(np.count_nonzero(peso))
        s = float(peso.sum())
        if s <= 0:
            return (False, nan, nan, pico, 0, "mancha vacía")
        vv, uu = np.mgrid[c0:c1, a0:a1]
        mx = float((peso * uu).sum() / s) + x0
        my = float((peso * vv).sum() / s) + y0
        u_real, w_real, area = 2.0 * mx + 0.5, 2.0 * my + 0.5, 4 * area_v
        if p.y_min > 0:
            ui, wi = int(u_real), int(w_real)
            brillo = int(y[max(wi - 3, 0): wi + 4, max(ui - 3, 0): ui + 4].max())
            if brillo < p.y_min:
                return (False, u_real, w_real, pico, area, f"rojo sin brillo (Y={brillo})")
        if not (p.area_min <= area <= p.area_max):
            return (False, u_real, w_real, pico, area, f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = float(np.hypot(u_real - self.ultimo[0], w_real - self.ultimo[1]))
            if salto > p.salto_max:
                return (False, u_real, w_real, pico, area, f"salto {salto:.0f} px")
        return (True, u_real, w_real, pico, area, "")


# =============================================================================
# Utilidades de entrada (teclas) y estadística
# =============================================================================
class EntradaEtiquetas:
    def __init__(self):
        self.ok = True

    def leer(self):
        if not self.ok:
            return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                lin = sys.stdin.readline()
                if lin == "":
                    self.ok = False
                    return None
                return ETIQUETAS.get(lin.strip())
        except Exception:
            self.ok = False
        return None


def col(rows, k, cond=None):
    return np.array([r[k] for r in rows if cond is None or cond(r)], dtype=float)


def pc(a, qs=(5, 50, 95), fmt="{:.1f}"):
    a = a[np.isfinite(a)]
    if a.size == 0:
        return "n/d"
    return " / ".join(fmt.format(np.percentile(a, q)) for q in qs)


def perc(a, q, defecto=float("nan")):
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else defecto


# =============================================================================
# Resumen y parámetros sugeridos
# =============================================================================
def generar_resumen(rows, bg, args, thr_ref, pd, t_nominal_ms):
    L = []
    add = L.append
    add("=" * 78)
    add("RESUMEN DE MEDICIÓN - AVIÓN ROJO")
    add("=" * 78)
    add(f"Cuadros registrados: {len(rows)}   fuente: {args.fuente}   "
        f"exposición: {args.exposicion} us   ganancia: {args.ganancia}   "
        f"ganancias de color: {args.ganancias_color}")
    dts = col(rows, "dt_ms")
    dts = dts[np.isfinite(dts)]
    if dts.size:
        perdidos = int((dts > 1.5 * t_nominal_ms).sum())
        add(f"Periodo entre cuadros: mediana {np.median(dts):.2f} ms (nominal "
            f"{t_nominal_ms:.2f}); cuadros con salto > 1,5x nominal: {perdidos}")
    add("")
    add("--- FONDO (sin avión) -------------------------------------------------------")
    add(f"Plano V: máximo {bg['vmax']}, p99.9 {bg['v_p999']}, medio {bg['v_medio']:.1f}, "
        f"desv. {bg['v_std']:.2f}  ({bg['n_cuadros']} cuadros)")
    add(f"Plano Y: máximo {bg['ymax']}, p99.9 {bg['y_p999']}, medio {bg['y_medio']:.1f}")
    add(f"Umbral de referencia usado (V): {thr_ref:.0f}  (máximo del fondo + margen)")

    etiquetas = [e for e in dict.fromkeys(r["etiqueta"] for r in rows)]
    add("")
    add("--- POR ORIENTACIÓN (p5 / mediana / p95) -------------------------------------")
    for e in etiquetas:
        sub = [r for r in rows if r["etiqueta"] == e]
        con = [r for r in sub if r["ref_ok"] == 1]
        add(f"[{e}] cuadros: {len(sub)}   con avión detectado por la referencia: "
            f"{len(con)} ({100 * len(con) / max(len(sub), 1):.0f} %)")
        if e == "sin_avion" or not con:
            continue
        g = lambda k: col(con, k)
        add(f"   área (px Y)      {pc(g('area_y'), fmt='{:.0f}')}")
        add(f"   eje mayor/menor  {pc(g('eje_mayor_y'))}  |  {pc(g('eje_menor_y'))}  (px Y)")
        add(f"   aspecto caja     {pc(g('aspecto'), fmt='{:.2f}')}   relleno {pc(g('relleno'), fmt='{:.2f}')}"
            f"   elongación {pc(g('elongacion'), fmt='{:.2f}')}")
        add(f"   V pico           {pc(g('v_pico'), fmt='{:.0f}')}   V medio {pc(g('v_medio'))}"
            f"   V entorno {pc(g('v_anillo'))}")
        add(f"   Y medio objeto   {pc(g('y_medio'))}   Y entorno {pc(g('y_anillo'))}"
            f"   contraste Y {pc(g('contraste_y'))}")
        add(f"   Y máx 7x7        {pc(g('y_max7'), fmt='{:.0f}')}   fracción saturada "
            f"{pc(g('sat_frac'), fmt='{:.2f}')}")

    # ---------------- movimiento ----------------
    pl = [r for r in rows if r["etiqueta"] != "sin_avion" and r["ref_ok"] == 1]
    add("")
    add("--- MOVIMIENTO (todas las orientaciones con avión) ---------------------------")
    d1 = col(pl, "salto_px", lambda r: r["frames_desde"] == 1)
    dg = col(pl, "salto_px", lambda r: r["frames_desde"] > 1)
    add(f"Desplazamiento entre cuadros consecutivos [px]: p50 {perc(d1, 50):.1f}  p95 {perc(d1, 95):.1f}  "
        f"p99 {perc(d1, 99):.1f}  máx {perc(d1, 100):.1f}")
    add(f"Velocidad [px/s]: p50 {perc(col(pl, 'vel_px_s'), 50):.0f}  p95 {perc(col(pl, 'vel_px_s'), 95):.0f}  "
        f"p99 {perc(col(pl, 'vel_px_s'), 99):.0f}")
    if dg.size:
        add(f"Salto tras pérdida de cuadros [px]: p50 {perc(dg, 50):.1f}  p95 {perc(dg, 95):.1f}  máx {perc(dg, 100):.1f}")
    ra = col(pl, "ratio_area")
    add(f"Cambio de área entre cuadros (área / área previa): p1 {perc(ra, 1):.2f}  p99 {perc(ra, 99):.2f}")
    pp = col(pl, "perdidos_previos")
    pp = pp[np.isfinite(pp) & (pp > 0)]
    add(f"Pérdidas de la referencia en medio del movimiento: {pp.size} episodios, "
        f"racha máxima {int(pp.max()) if pp.size else 0} cuadros")

    # ---------------- detector emulado ----------------
    add("")
    add("--- DETECTOR EMULADO (DetectorLaserRojo con los parámetros --det-*) ---------")
    add(f"Parámetros: bloque={pd.bloque} v_umbral={pd.v_umbral} frac_corte={pd.frac_corte} "
        f"y_min={pd.y_min} area=[{pd.area_min},{pd.area_max}] salto_max={pd.salto_max} "
        f"ventana={pd.ventana} (paciencia {args.det_paciencia} cuadros)")
    cnt = Counter(r["clase"] for r in rows if r["clase"])
    tp = cnt.get("TP", 0) + cnt.get("DESPLAZADO", 0)
    fn = cnt.get("FN", 0)
    fp = cnt.get("FP", 0)
    tn = cnt.get("TN", 0)
    add(f"Con avión: detectado {tp} / {tp + fn} ({100 * tp / max(tp + fn, 1):.1f} %)  "
        f"[de ellos desplazados del centro de la referencia: {cnt.get('DESPLAZADO', 0)}]")
    add(f"Sin avión: falsos positivos {fp} / {fp + tn} ({100 * fp / max(fp + tn, 1):.1f} %)")
    mot = Counter(re.sub(r"\d+(\.\d+)?", "N", r["det_motivo"])
                  for r in rows if r["det_ok"] == 0 and r["ref_ok"] == 1 and r["etiqueta"] != "sin_avion")
    if mot:
        add("Motivos de rechazo con avión presente: " +
            ", ".join(f"'{k}' {v}" for k, v in mot.most_common(5)))
    err = col(rows, "err_det_ref_px")
    add(f"Error de centroide detector vs referencia [px]: p50 {perc(err, 50):.1f}  p95 {perc(err, 95):.1f}  "
        f"máx {perc(err, 100):.1f}")
    tdet = col(rows, "det_t_ms")
    add(f"Tiempo del detector [ms]: p50 {perc(tdet, 50):.3f}  p99 {perc(tdet, 99):.3f}  "
        f"(periodo de cuadro {t_nominal_ms:.1f} ms)")

    # ---------------- parámetros sugeridos ----------------
    add("")
    add("=" * 78)
    add("PARÁMETROS SUGERIDOS (punto de partida; validar con --det-* y volver a medir)")
    add("=" * 78)
    if len(pl) < 30:
        add(f"Hay solo {len(pl)} cuadros con avión: grábalo más tiempo (y con etiquetas 1-5) "
            f"para obtener sugerencias fiables.")
        return "\n".join(L)

    vp5 = perc(col(pl, "v_pico"), 5)
    margen = vp5 - bg["vmax"]
    if margen >= 4:
        v_umbral = int(round(bg["vmax"] + 0.5 * margen))
        add(f"v_umbral: {v_umbral}   (fondo máx {bg['vmax']}, V pico del avión p5 {vp5:.0f}, "
            f"margen {margen:.0f})")
    else:
        v_umbral = int(bg["vmax"] + 2)
        add(f"AVISO v_umbral: el V del avión (p5 {vp5:.0f}) casi no supera el máximo del fondo "
            f"({bg['vmax']}). Con {v_umbral} habrá pérdidas o falsos positivos. Mejoras: más "
            f"luz, más ganancia de rojo (--ganancias-color), otra exposición, o quitar objetos "
            f"rojos del fondo.")
    a1, a99 = perc(col(pl, "area_y"), 1), perc(col(pl, "area_y"), 99)
    area_min = max(4, int(0.5 * a1))
    area_max = int(1.3 * a99)
    eje99 = perc(col(pl, "eje_mayor_y"), 99)
    eje95_v = perc(col(pl, "eje_mayor_y"), 95) / 2.0
    bloque = int(min(max(math.ceil(1.1 * eje95_v / 3.0), 2), 16))
    alcanzable = 4 * (2 * ((3 * bloque) // 2)) ** 2
    add(f"bloque: {bloque}   (la ventana del centroide mide ~3·bloque px de V; debe cubrir el "
        f"eje mayor p95 = {eje95_v:.0f} px de V)")
    if area_max > alcanzable:
        add(f"   AVISO: area_max {area_max} supera el área que ve el centroide con bloque={bloque} "
            f"({alcanzable}); se limita.")
        area_max = alcanzable
    add(f"area_min: {area_min}   area_max: {area_max}   (px del plano Y; área p1 {a1:.0f}, p99 {a99:.0f})")
    d99 = perc(d1, 99, 0.0)
    dg95 = perc(dg, 95, 0.0)
    salto_max = int(max(20, math.ceil(max(1.5 * d99, dg95))))
    add(f"salto_max_px: {salto_max}   (mayor entre 1,5 x p99 del desplazamiento entre cuadros = "
        f"{d99:.1f} px y p95 del salto tras pérdida = {dg95:.1f} px; mínimo 20)")
    ventana = int(max(64, 8 * math.ceil((2 * (1.5 * d99 + 0.5 * eje99) + 16) / 8)))
    add(f"ventana_seguimiento: {ventana}   (lado en px Y: 2·(1,5·p99 salto + medio eje mayor p99) + 16)")
    y7 = perc(col(pl, "y_max7"), 5)
    add(f"y_min: 0   (el avión es mate: Y máx 7x7 p5 = {y7:.0f}; si algún día confirmas por brillo, "
        f"no pases de {int(0.7 * y7)})")
    add("frac_corte: 0.5   (revisar con cuadros.npz si el centroide se sesga con el avión de canto)")
    add("")
    add("# --- pegar en parametros.py (configuración del avión) ---")
    add(f"bloque = {bloque}")
    add(f"v_umbral = {v_umbral}")
    add("frac_corte = 0.5")
    add("y_min = 0")
    add(f"area_min = {area_min}")
    add(f"area_max = {area_max}")
    add(f"salto_max_px = {salto_max}")
    add(f"ventana_seguimiento = {ventana}")
    add("")
    add("NOTA: la 'referencia' supone que lo único rojo sobre el umbral es el avión. Si en "
        "el cuadro hay otras cosas rojas más intensas que el fondo medido, aparecerán como "
        "avión; sácalas de la escena o repite la fase de fondo con ellas presentes.")
    return "\n".join(L)


# =============================================================================
# Programa principal
# =============================================================================
class ParamsDet:
    pass


def parse():
    ap = argparse.ArgumentParser(description="Mide métricas del avión rojo para diseñar su detector.")
    ap.add_argument("--fuente", choices=["camara", "sintetica"], default="camara")
    ap.add_argument("--ancho", type=int, default=640)
    ap.add_argument("--alto", type=int, default=480)
    ap.add_argument("--fps", type=float, default=80.0)
    ap.add_argument("--exposicion", type=int, default=8000, help="us (más alta que 2000 para objeto mate)")
    ap.add_argument("--ganancia", type=float, default=1.0)
    ap.add_argument("--ganancias-color", type=float, nargs=2, metavar=("ROJO", "AZUL"), default=None)
    ap.add_argument("--calibrar-fondo", type=float, default=5.0, help="s sin avión en cuadro")
    ap.add_argument("--duracion", type=float, default=60.0, help="s de registro con avión")
    ap.add_argument("--margen-v", type=float, default=8.0, help="margen sobre el máximo del fondo")
    ap.add_argument("--v-umbral-ref", type=float, default=None, help="forzar umbral de referencia")
    ap.add_argument("--min-pix-ref", type=int, default=4, help="px de V mínimos para dar por visto el avión")
    ap.add_argument("--etiqueta", default="sin_avion", choices=list(ETIQUETAS.values()))
    ap.add_argument("--max-cuadros", type=int, default=200, help="cuadros a guardar en cuadros.npz (0 = ninguno)")
    ap.add_argument("--guardar-cada", type=int, default=6)
    ap.add_argument("--salida", default="medicion_avion")
    ap.add_argument("--sin-ventana", action="store_true")
    g = ap.add_argument_group("detector emulado (candidatos a parámetros del avión)")
    g.add_argument("--det-bloque", type=int, default=8)
    g.add_argument("--det-v-umbral", type=float, default=145)
    g.add_argument("--det-frac-corte", type=float, default=0.5)
    g.add_argument("--det-y-min", type=int, default=0)
    g.add_argument("--det-area-min", type=int, default=150)
    g.add_argument("--det-area-max", type=int, default=5000)
    g.add_argument("--det-salto-max", type=float, default=80)
    g.add_argument("--det-ventana", type=int, default=160)
    g.add_argument("--det-paciencia", type=int, default=5)
    return ap.parse_args()


def main():
    a = parse()
    out = Path(a.salida)
    out.mkdir(parents=True, exist_ok=True)
    W, H = a.ancho, a.alto

    pd = ParamsDet()
    pd.bloque, pd.v_umbral, pd.frac_corte, pd.y_min = a.det_bloque, a.det_v_umbral, a.det_frac_corte, a.det_y_min
    pd.area_min, pd.area_max, pd.salto_max, pd.ventana = a.det_area_min, a.det_area_max, a.det_salto_max, a.det_ventana
    det = DetectorRojoEmulado(pd, a.det_paciencia)

    cv2 = None
    if not a.sin_ventana:
        try:
            import cv2 as _cv2
            cv2 = _cv2
            cv2.namedWindow("medir_avion", cv2.WINDOW_NORMAL)
        except Exception as e:
            print(f"AVISO: sin ventana ({e}).")
    fuente = FuenteCamara(a) if a.fuente == "camara" else FuenteSintetica(a)
    entrada = EntradaEtiquetas()
    t_nominal_ms = 1e3 / a.fps

    print("\nFASE 1/2 - FONDO: mantén el AVIÓN FUERA del cuadro "
          f"({a.calibrar_fondo:.0f} s)...")
    hv = np.zeros(256, np.int64)
    hy = np.zeros(256, np.int64)
    vmaxs, vmeans, vstds, n_bg = [], [], [], 0
    fase = "fondo"
    t0_ns = None
    t_reg0 = None
    etiqueta = a.etiqueta
    thr_ref = None
    rows, fr_y, fr_v, fr_meta = [], [], [], []
    ult_ref = None            # (u, w, t_ns) última detección de la referencia
    faltan = 0
    i = 0
    t_prev_ns = None
    t_print = time.perf_counter()
    sint_ciclo = ["frente", "lado", "arriba", "libre", "sin_avion"]

    try:
        while True:
            avion_visible = (etiqueta != "sin_avion") if fase == "registro" else False
            y, u, v, t_ns = fuente.leer(avion=avion_visible)
            if t0_ns is None:
                t0_ns = t_ns

            # ------------------------------ FONDO ------------------------------
            if fase == "fondo":
                hv += np.bincount(v.ravel(), minlength=256)
                hy += np.bincount(y.ravel(), minlength=256)
                vmaxs.append(int(v.max()))
                vmeans.append(float(v.mean()))
                vstds.append(float(v.std()))
                n_bg += 1
                if (t_ns - t0_ns) / 1e9 >= a.calibrar_fondo:
                    cv = hv.cumsum() / hv.sum()
                    cy_ = hy.cumsum() / hy.sum()
                    bg = dict(vmax=int(max(vmaxs)), v_p999=int(np.searchsorted(cv, 0.999)),
                              v_medio=float(np.mean(vmeans)), v_std=float(np.mean(vstds)),
                              ymax=int(np.nonzero(hy)[0].max()), y_p999=int(np.searchsorted(cy_, 0.999)),
                              y_medio=float((hy * np.arange(256)).sum() / hy.sum()), n_cuadros=n_bg)
                    thr_ref = max(a.v_umbral_ref if a.v_umbral_ref is not None
                                  else bg["vmax"] + a.margen_v, V_NEUTRO + 1)
                    fase, t_reg0 = "registro", t_ns
                    if a.fuente == "sintetica":
                        etiqueta = "sin_avion"
                    print(f"Fondo medido: V máx {bg['vmax']}, p99.9 {bg['v_p999']}; umbral de "
                          f"referencia = {thr_ref:.0f}.")
                    print(f"\nFASE 2/2 - REGISTRO ({a.duracion:.0f} s). Etiqueta inicial: '{etiqueta}'.")
                    print("Etiquetas: " + "  ".join(f"{k}={v_}" for k, v_ in ETIQUETAS.items()) +
                          "   (ventana: tecla; SSH: número + Enter; q = terminar)")
                    print("Mete el avión al cuadro y pulsa 1-5 según la orientación. Termina con "
                          "unos segundos en 0 (sin avión).")
                    t_prev_ns = t_ns
                    t_print = time.perf_counter()
                continue

            # ---------------------------- REGISTRO -----------------------------
            t_rel = (t_ns - t_reg0) / 1e9
            if a.fuente == "sintetica":
                etiqueta = sint_ciclo[min(int(t_rel // (a.duracion / len(sint_ciclo))), len(sint_ciclo) - 1)]
            nueva = entrada.leer()
            if nueva:
                etiqueta = nueva
                ult_ref = None

            y2 = y[0::2, 0::2]
            ref = segmentar_ref(y, u, v, y2, thr_ref, a.min_pix_ref)
            rs = det.procesar(y, v)

            row = dict.fromkeys(COLUMNAS, float("nan"))
            row.update(i=i, t_s=round(t_rel, 4), dt_ms=(t_ns - t_prev_ns) / 1e6, etiqueta=etiqueta,
                       ref_ok=0, det_ok=int(rs.valido), det_motivo=rs.motivo, det_t_ms=rs.t_ms,
                       det_u=rs.u, det_w=rs.w, det_area=rs.area, det_pico=rs.pico, clase="")
            t_prev_ns = t_ns
            err = float("nan")
            if ref is not None:
                row["ref_ok"] = 1
                row.update(ref_u=ref["u"], ref_w=ref["w"], **{k: ref[k] for k in COLUMNAS if k in ref})
                row["perdidos_previos"] = faltan
                if ult_ref is not None and (t_ns - ult_ref[2]) < 1.0e9:
                    row["salto_px"] = math.hypot(ref["u"] - ult_ref[0], ref["w"] - ult_ref[1])
                    row["frames_desde"] = faltan + 1
                    row["vel_px_s"] = row["salto_px"] / max((t_ns - ult_ref[2]) / 1e9, 1e-6)
                    row["ratio_area"] = ref["area_y"] / max(ult_ref[3], 1)
                ult_ref = (ref["u"], ref["w"], t_ns, ref["area_y"])
                faltan = 0
                if rs.valido:
                    err = math.hypot(rs.u - ref["u"], rs.w - ref["w"])
            else:
                faltan += 1
            row["err_det_ref_px"] = err

            # clasificación detector vs verdad (verdad = referencia, salvo 'sin_avion')
            verdad = (etiqueta != "sin_avion") and ref is not None
            if verdad and rs.valido:
                tam = max(ref["eje_mayor_y"], 10.0)
                row["clase"] = "TP" if err <= tam else "DESPLAZADO"
            elif verdad:
                row["clase"] = "FN"
            elif rs.valido:
                row["clase"] = "FP"
            else:
                row["clase"] = "TN"
            if etiqueta != "sin_avion" and ref is None and not rs.valido:
                row["clase"] = ""        # avión fuera de cuadro: no cuenta como acierto ni error
            rows.append(row)

            if a.max_cuadros and len(fr_y) < a.max_cuadros and i % max(a.guardar_cada, 1) == 0:
                fr_y.append(y)
                fr_v.append(v)
                fr_meta.append((t_ns, etiqueta))
            i += 1

            ahora = time.perf_counter()
            if ahora - t_print >= 1.0:
                t_print = ahora
                est = (f"avión A={ref['area_y']} V={ref['v_pico']} elong={ref['elongacion']:.1f}"
                       if ref else "sin avión")
                print(f"t={t_rel:5.1f}s [{etiqueta:9s}] ref: {est:38s} det: "
                      f"{'OK ' if rs.valido else 'NO '}{rs.motivo}")

            if cv2 is not None and i % 3 == 0:
                img = cv2.cvtColor(y, cv2.COLOR_GRAY2BGR)
                if ref:
                    cv2.circle(img, (int(ref["u"]), int(ref["w"])), int(max(ref["eje_mayor_y"] / 2, 6)),
                               (0, 255, 0), 1)
                if rs.valido:
                    cv2.drawMarker(img, (int(rs.u), int(rs.w)), (255, 255, 0), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(img, f"[{etiqueta}] t={t_rel:.0f}s  verde=ref  cian=detector",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("medir_avion", img)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if chr(k) in ETIQUETAS:
                    etiqueta = ETIQUETAS[chr(k)]
                    ult_ref = None
            if t_rel >= a.duracion:
                break
    except KeyboardInterrupt:
        print("\nInterrumpido: se guardan los datos obtenidos.")
    finally:
        fuente.cerrar()
        if cv2 is not None:
            cv2.destroyAllWindows()

    if thr_ref is None or not rows:
        print("No se completó la fase de fondo; no hay datos que guardar.")
        return

    # ------------------------------- SALIDAS -------------------------------
    with open(out / "cuadros.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=COLUMNAS)
        wr.writeheader()
        wr.writerows(rows)
    if fr_y:
        np.savez_compressed(out / "cuadros.npz", y=np.stack(fr_y), v=np.stack(fr_v),
                            t_ns=np.array([m[0] for m in fr_meta], np.int64),
                            etiqueta=np.array([m[1] for m in fr_meta]), thr_ref=thr_ref)
    texto = generar_resumen(rows, bg, a, thr_ref, pd, t_nominal_ms)
    (out / "resumen.txt").write_text(texto, encoding="utf-8")
    (out / "fondo.json").write_text(json.dumps(bg, indent=2), encoding="utf-8")
    print("\n" + texto)
    print(f"\nArchivos en '{out}/': cuadros.csv, resumen.txt, fondo.json" +
          (", cuadros.npz" if fr_y else ""))


if __name__ == "__main__":
    main()
