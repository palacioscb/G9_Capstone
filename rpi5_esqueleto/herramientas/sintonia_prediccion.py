#!/usr/bin/env python3
"""
sintonia_prediccion.py — Barrido de λ y orden del predictor RLS sobre la fuente sintética.

Compara el error de posición en t + h contra la referencia "sin predictor"
(usar la última medición). Si el predictor no le gana a esa referencia, no
aporta: sirve para escoger λ y orden, y como figura para el informe (CTL-3).

Uso (PC o RPi):  python3 herramientas/sintonia_prediccion.py [--fps 60] [--h 0.05]
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pt.parametros import Parametros            # noqa: E402
from pt.camara import FuenteSintetica           # noqa: E402
from pt.deteccion import DetectorLaser          # noqa: E402
from pt.prediccion import Estimador2D           # noqa: E402


def evaluar(P, fps, dur, semilla=0):
    W, H = P.captura.salida_px
    fu = FuenteSintetica(W, H, fps, dur, semilla=semilla, oclusiones=(), falsos=())
    det, est = DetectorLaser(P.deteccion), Estimador2D(P.prediccion)
    e_pred, e_base = [], []
    for c in fu:
        r = det.procesar(c.img, usar_ventana=True)
        e = est.actualizar(c.t_exp_ns * 1e-9, r)
        if r.valido and not math.isnan(e.u) and e.confianza > 0 and c.k > 20:
            ts = c.t_exp_ns * 1e-9 + P.prediccion.horizonte_s - fu.t0_ns * 1e-9
            uh, wh = fu.trayectoria(ts)
            e_pred.append(math.hypot(e.u - uh, e.w - wh))
            e_base.append(math.hypot(r.u - uh, r.w - wh))
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return rms(e_pred), float(np.max(e_pred)), rms(e_base)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--h", type=float, default=0.05, help="horizonte [s]")
    ap.add_argument("--dur", type=float, default=15.0)
    a = ap.parse_args()
    print(f"fps={a.fps}  h={a.h*1e3:.0f} ms  (error de posición en t+h, px)")
    print(f"{'orden':>5} {'λ':>5} {'N_ef[ms]':>8} {'RMS pred':>9} {'máx pred':>9} {'RMS sin pred':>12}")
    for orden in (2, 3):
        for lam in (0.60, 0.70, 0.80, 0.85, 0.90, 0.95):
            P = Parametros()
            P.prediccion.orden, P.prediccion.lam, P.prediccion.horizonte_s = orden, lam, a.h
            rp, mp, rb = evaluar(P, a.fps, a.dur)
            n_ef = 1 / (1 - lam) / a.fps * 1e3
            print(f"{orden:>5} {lam:>5.2f} {n_ef:>8.0f} {rp:>9.2f} {mp:>9.2f} {rb:>12.2f}")


if __name__ == "__main__":
    main()
