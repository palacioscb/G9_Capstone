#!/usr/bin/env python3
"""
barrido_rls_ruido.py — Barrido del predictor RLS (λ, orden) con ruido de detección añadido.

Extiende sintonia_prediccion.py para la sección 5 del informe del 25 %:
  * las detecciones se calculan UNA vez por semilla (el detector no depende de λ);
  * se evalúan dos tasas (60 y 80 fps) y dos horizontes (50 ms del informe del 5 %
    y el retardo equivalente medido tau_eq = L + T/2 de presupuesto_latencia.py);
  * a 80 fps se suma ruido gaussiano σ ∈ {0; 0,25; 0,5; 1} px al centroide, porque la
    fuente sintética casi no tiene ruido (RMS 0,05 px) y eso favorece λ pequeños.

Métrica: error RMS entre la posición predicha para t+h y la posición real en t+h,
comparado con "sin predictor" (usar la última medición). Promedio de 3 semillas.

Uso:  python3 herramientas/barrido_rls_ruido.py --salida resultados/barrido_rls
"""
import argparse
import csv
import dataclasses
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pt.parametros import Parametros            # noqa: E402
from pt.camara import FuenteSintetica           # noqa: E402
from pt.deteccion import DetectorLaser          # noqa: E402
from pt.prediccion import Estimador2D           # noqa: E402

LAMS = [0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95]
TAU_EQ = {60: 0.02694, 80: 0.02486}   # [s] presupuesto_latencia.py, escenario PoC


def detecciones(fps, dur, semilla):
    P = Parametros()
    W, H = P.captura.salida_px
    fu = FuenteSintetica(W, H, fps, dur, semilla=semilla, oclusiones=(), falsos=())
    det = DetectorLaser(P.deteccion)
    return fu, [(c.k, c.t_exp_ns, det.procesar(c.img, usar_ventana=True)) for c in fu]


def con_ruido(dets, sigma, rng):
    if sigma == 0:
        return dets
    return [(k, t, dataclasses.replace(r, u=r.u + rng.normal(0, sigma), w=r.w + rng.normal(0, sigma))
             if r.valido else r) for k, t, r in dets]


def evaluar(fu, dets, orden, lam, h):
    P = Parametros()
    P.prediccion.orden, P.prediccion.lam, P.prediccion.horizonte_s = orden, lam, h
    est = Estimador2D(P.prediccion)
    e_pred, e_sin = [], []
    for k, t_ns, r in dets:
        e = est.actualizar(t_ns * 1e-9, r)
        if r.valido and not math.isnan(e.u) and e.confianza > 0 and k > 20:
            uh, wh = fu.trayectoria(t_ns * 1e-9 + h - fu.t0_ns * 1e-9)   # verdad en t + h
            e_pred.append(math.hypot(e.u - uh, e.w - wh))
            e_sin.append(math.hypot(r.u - uh, r.w - wh))                 # sin predictor
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return rms(e_pred), float(np.percentile(e_pred, 95)), rms(e_sin)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=15.0)
    ap.add_argument("--salida", default="resultados/barrido_rls")
    a = ap.parse_args()
    os.makedirs(a.salida, exist_ok=True)
    f1, f2 = [], []
    for fps in (60, 80):
        for sem in (0, 1, 2):
            fu, d = detecciones(fps, a.dur, sem)
            for h in (0.050, TAU_EQ[fps]):                       # barrido 1: sin ruido añadido
                for orden in (2, 3):
                    for lam in LAMS:
                        rp, p95, rs = evaluar(fu, d, orden, lam, h)
                        f1.append(dict(fps=fps, h_ms=round(h * 1e3, 2), semilla=sem, orden=orden, lam=lam,
                                       Nef_muestras=round(1 / (1 - lam), 1),
                                       Nef_ms=round(1 / (1 - lam) / fps * 1e3, 1),
                                       rms_pred=rp, p95_pred=p95, rms_sin=rs))
            if fps == 80:                                        # barrido 2: ruido de detección
                rng = np.random.default_rng(100 + sem)
                for sigma in (0.0, 0.25, 0.5, 1.0):
                    dn = con_ruido(d, sigma, rng)
                    for orden in (2, 3):
                        for lam in LAMS:
                            rp, p95, rs = evaluar(fu, dn, orden, lam, TAU_EQ[80])
                            f2.append(dict(fps=80, h_ms=round(TAU_EQ[80] * 1e3, 2), semilla=sem,
                                           sigma_px=sigma, orden=orden, lam=lam,
                                           rms_pred=rp, p95_pred=p95, rms_sin=rs))
            print(f"fps {fps} semilla {sem} listo", flush=True)
    for nombre, filas in (("datos_barrido_rls.csv", f1), ("datos_barrido_ruido.csv", f2)):
        with open(os.path.join(a.salida, nombre), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(filas[0]))
            w.writeheader()
            w.writerows(filas)
    # resumen en pantalla: RMS medio [px] a 80 fps, h = tau_eq
    print(f"\n80 fps, h = {TAU_EQ[80]*1e3:.1f} ms — RMS [px] (promedio 3 semillas)")
    print("σ[px] orden " + " ".join(f"{l:>5.2f}" for l in LAMS) + "   sin pred")
    for sigma in (0.0, 0.25, 0.5, 1.0):
        for orden in (2, 3):
            v = [np.mean([r["rms_pred"] for r in f2 if r["sigma_px"] == sigma and r["orden"] == orden
                          and r["lam"] == l]) for l in LAMS]
            s = np.mean([r["rms_sin"] for r in f2 if r["sigma_px"] == sigma])
            print(f"{sigma:>5} {orden:>5} " + " ".join(f"{x:>5.2f}" for x in v) + f"   {s:>6.2f}")


if __name__ == "__main__":
    main()
