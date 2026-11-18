#!/usr/bin/env python3
"""
barrido_lazo_cerrado.py — barrido de λ y orden del RLS en LAZO CERRADO SIMULADO, con y sin
feedforward, y con ruido agregado al centroide. Es la fuente de la figura 5 y de la tabla 3
de docs/prediccion_rls.pdf.

Cada punto es una corrida completa de main.py (--enlace simulado --lampara, 10 s a 80 fps):
la misma escena, el mismo modelo de planta de pid_t_2 y el mismo PI del firmware. Se mide el
error de centrado (distancia del objetivo al centro de la imagen) en SEGUIMIENTO.

Uso (PC):
    python3 herramientas/barrido_lazo_cerrado.py                 # 48 corridas + 1, ~10 s cada una
    python3 herramientas/barrido_lazo_cerrado.py --procesos 4    # en paralelo
    python3 herramientas/barrido_lazo_cerrado.py --rapido        # 2 corridas, para probar
Escribe docs/figuras/datos_barrido_lazo_cerrado.json (lo lee docs/figuras/figuras_prediccion.py).
"""
#? La planta es el modelo ideal de pid_t_2 con ζ = 0,05 supuesto: los valores sirven para
#? comparar configuraciones entre sí, no como predicción del error de la plataforma real.
import argparse
import contextlib
import csv
import dataclasses
import io
import itertools
import json
import math
import os
import sys
import tempfile
from multiprocessing import Pool

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SALIDA = os.path.join(RAIZ, "..", "..", "docs", "figuras", "datos_barrido_lazo_cerrado.json")


def correr(cfg):
    """Una corrida de main.py. cfg = (orden, lam, feedforward, ruido_px, sin_predictor)."""
    orden, lam, ff, ruido_px, sin_predictor = cfg
    sys.path.insert(0, RAIZ)
    os.chdir(RAIZ)
    import numpy as np
    import main as programa
    from pt import control, deteccion
    from pt.parametros import Parametros as ParametrosBase

    def parametros():
        p = ParametrosBase()
        p.prediccion.orden, p.prediccion.lam = orden, lam
        return p
    programa.Parametros = parametros

    #* Ruido gaussiano sobre el centroide ya detectado (imita un detector más ruidoso).
    rng = np.random.default_rng(7)

    class DetectorConRuido(deteccion.DetectorLaserRojo):
        def procesar(self, img, usar_ventana=True, v=None):
            r = super().procesar(img, usar_ventana, v)
            if r.valido and ruido_px > 0:
                r.u += rng.normal(0, ruido_px)
                r.w += rng.normal(0, ruido_px)
            return r
    programa.DetectorLaserRojo = DetectorConRuido

    if sin_predictor:
        #* Referencia "sin predictor": la consigna es la última medición válida, sin extrapolar.
        original = control.ReferenciaVisual.actualizar

        def actualizar(self, t_exp_ns, det, pose, t_aplic_ns):
            med, e = original(self, t_exp_ns, det, pose, t_aplic_ns)
            if med.valida:
                self._ultima = (med.pan, med.tilt)
            if getattr(self, "_ultima", None) and e.confianza > 0:
                e = dataclasses.replace(e, u=self._ultima[0], w=self._ultima[1])
            return med, e
        control.ReferenciaVisual.actualizar = actualizar

    carpeta = tempfile.mkdtemp(prefix="barrido_")
    sys.argv = ["main.py", "--enlace", "simulado", "--lampara", "--registro", carpeta] + (["--feedforward"] if ff else [])
    salida = io.StringIO()
    with contextlib.redirect_stdout(salida):
        programa.main()
    ruta = os.path.join(carpeta, os.listdir(carpeta)[0])
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    t0 = int(filas[0]["t_exp_ns"])

    def rms(a, b):
        e = [float(f["err_centro_px"]) for f in filas
             if a <= (int(f["t_exp_ns"]) - t0) * 1e-9 < b and f["estado"] == "SEGUIMIENTO" and f["err_centro_px"] not in ("", "nan")]
        e = [x for x in e if not math.isnan(x)]
        return round((sum(x * x for x in e) / len(e)) ** 0.5, 2) if e else None

    perdidas = sum(1 for a, b in zip(filas, filas[1:]) if a["estado"] == "SEGUIMIENTO" and b["estado"] == "PERDIDA")
    #* Tramos sin oclusión de la fuente sintética: 1–2,9 s y 7–10 s (oclusiones en 3,0–3,5 y 6,0–6,3 s).
    return {"orden": orden, "lam": lam, "feedforward": ff, "ruido_px": ruido_px, "sin_predictor": bool(sin_predictor),
            "rms_todo_px": rms(0, 99), "rms_1a2.9s_px": rms(1.0, 2.9), "rms_7a10s_px": rms(7.0, 10.0), "perdidas": perdidas}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--procesos", type=int, default=2)
    ap.add_argument("--rapido", action="store_true", help="sólo 2 corridas (prueba del script; no sobrescribe los datos)")
    ap.add_argument("--salida", default=SALIDA)
    a = ap.parse_args()
    if a.rapido:
        cfgs = [(3, 0.8, 0, 0.0, 0), (3, 0.8, 1, 0.0, 0)]
    else:
        cfgs = [c + (0,) for c in itertools.product((2, 3), (0.6, 0.7, 0.8, 0.85, 0.9, 0.95), (0, 1), (0.0, 1.0))]
        cfgs.append((3, 0.8, 0, 0.0, 1))
    with Pool(a.procesos) as pool:
        res = []
        for r in pool.imap_unordered(correr, cfgs, chunksize=1):
            res.append(r)
            print(f"{len(res):2d}/{len(cfgs)}  orden {r['orden']}  λ {r['lam']:.2f}  ff {r['feedforward']}  ruido {r['ruido_px']} px"
                  f"{'  SIN PREDICTOR' if r['sin_predictor'] else ''}:  todo {r['rms_todo_px']} px | 1–2,9 s {r['rms_1a2.9s_px']} px | 7–10 s {r['rms_7a10s_px']} px",
                  flush=True)
    if a.rapido:
        return
    sp = [r for r in res if r["sin_predictor"]]
    doc = {"descripcion": "Lazo cerrado simulado (main.py --enlace simulado --lampara, 10 s, 80 fps): error de centrado RMS "
                          "en SEGUIMIENTO segun orden y lambda del RLS, con y sin feedforward, y con ruido gaussiano agregado "
                          "al centroide (ruido_px). Generado con herramientas/barrido_lazo_cerrado.py.",
           "sin_predictor_sin_ff_rms_todo_px": sp[0]["rms_todo_px"] if sp else None,
           "filas": sorted((r for r in res if not r["sin_predictor"]),
                           key=lambda r: (r["ruido_px"], r["feedforward"], r["orden"], r["lam"]))}
    with open(a.salida, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    print("Escrito:", os.path.normpath(a.salida))


if __name__ == "__main__":
    main()
