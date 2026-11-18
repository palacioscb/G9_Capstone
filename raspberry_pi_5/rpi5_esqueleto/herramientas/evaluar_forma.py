#!/usr/bin/env python3
"""
evaluar_forma.py — ¿cuánto acierta el detector por forma (pt/deteccion.py, DetectorForma)?

Dibuja escenas SINTÉTICAS de 640x480 en escala de grises y pasa cada una por el detector
completo. Mide: aciertos (detección válida a menos de 5 px del objetivo), falsos positivos
(detección válida en otro lugar), error del centroide y tiempo de proceso.

Cada escena tiene fondo con degradado de iluminación y ruido, y un objetivo con tamaño,
giro, inclinación y posición al azar. Según el caso se agregan: figuras de distracción
(rectángulos, barras, manchas, una lámpara redonda), el palo que sostiene al avión, o un
avión de otro modelo que el de la plantilla.

Uso (PC):
    python3 herramientas/evaluar_forma.py              # ~1 min; imprime la tabla
    python3 herramientas/evaluar_forma.py --n 1000     # más escenas por caso
    python3 herramientas/evaluar_forma.py --guardar /tmp/escenas   # además guarda ejemplos en PNG
"""
#? Son escenas dibujadas: bordes nítidos, objetivo de un solo tono y fondo sin textura real.
#? Los porcentajes sirven para comparar casos y fijar valores de partida, no como desempeño
#? esperado con la cámara. Lo que falta medir: el avión real, con su fondo y su iluminación.
import argparse
import os
import sys

import cv2
import numpy as np

RAIZ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, RAIZ)
from pt import formas as fm                                       # noqa: E402
from pt.deteccion import DetectorForma                            # noqa: E402
from pt.parametros import Parametros                              # noqa: E402

W, H, S = 640, 480, 3            # S: se dibuja a 3x y se reduce, para que los bordes queden suaves

#* Avión de OTRO modelo (ala recta, hélice): no está en el banco. Mide cuánto generaliza la
#* plantilla genérica "avion_planta".
AVION_OTRO = fm._espejo([(0.0, 1.0), (0.10, 0.92), (0.12, 0.45), (1.0, 0.40), (1.0, 0.12), (0.12, 0.08),
                         (0.09, -0.62), (0.42, -0.70), (0.42, -0.90), (0.06, -0.88), (0.0, -1.0)])


def fondo(rng, claro, real=False):
    base = rng.uniform(140, 190) if claro else rng.uniform(35, 70)
    gx, gy = rng.uniform(-25, 25, 2)
    if real:                                             # como la grabación del 18-11: fondo de 64 a 99
        base, ang = 81.0, rng.uniform(0, 2 * np.pi)
        gx, gy = 35 * np.cos(ang), 35 * np.sin(ang)
    yy, xx = np.mgrid[0:H * S, 0:W * S].astype(np.float32)
    return base + gx * (xx / (W * S) - 0.5) + gy * (yy / (H * S) - 0.5)


def poner(img, pol_px, nivel):
    cv2.fillPoly(img, [np.round(np.asarray(pol_px) * S).astype(np.int32)], float(nivel))


def distractores(img, rng, claro, lampara, evitar):
    """Figuras que no son el objetivo, lejos de él (a más de 110 px de `evitar`)."""
    def lugar():
        for _ in range(50):
            c = np.array([rng.uniform(50, W - 50), rng.uniform(50, H - 50)])
            if evitar is None or np.hypot(*(c - evitar)) > 110:
                return c
        return np.array([60.0, 60.0])
    tono = lambda: rng.uniform(30, 80) if claro else rng.uniform(150, 220)
    for _ in range(int(rng.integers(2, 5))):
        c, tipo = lugar(), rng.choice(["rect", "barra", "mancha"])
        if tipo == "rect":
            box = cv2.boxPoints(((float(c[0]), float(c[1])), (float(rng.uniform(25, 80)), float(rng.uniform(25, 80))), float(rng.uniform(0, 180))))
            poner(img, box, tono())
        elif tipo == "barra":
            box = cv2.boxPoints(((float(c[0]), float(c[1])), (float(rng.uniform(80, 200)), float(rng.uniform(4, 10))), float(rng.uniform(0, 180))))
            poner(img, box, tono())
        else:
            a = np.linspace(0, 2 * np.pi, 40, endpoint=False)
            r = rng.uniform(18, 40) * (1 + 0.4 * np.convolve(rng.normal(0, 1, 40), np.ones(5) / 5, mode="same"))
            poner(img, np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)], tono())
    if lampara:
        c = lugar()
        poner(img, fm.transformar(fm.poligono("circulo"), rng.uniform(30, 70), centro=c), 250)


def escena(rng, objetivo, claro=True, distraer=False, lampara=False, palo=False, tam=(45, 120), inclinar=0.5, real=False):
    """Devuelve (imagen uint8, centro verdadero o None). real=True copia los niveles medidos en
    la grabación del avión del 18-11: fondo 81 con degradado de 64 a 99, avión en 50, ruido 2,4."""
    img = fondo(rng, claro, real)
    centro = None
    if objetivo is not None:
        pol = AVION_OTRO if objetivo == "avion_otro" else fm.poligono(objetivo)
        t = rng.uniform(*tam)
        ap = (rng.uniform(inclinar, 1.0), 1.0) if rng.random() < 0.5 else (1.0, rng.uniform(inclinar, 1.0))
        c0 = (rng.uniform(90, W - 90), rng.uniform(90, H - 90))
        px = fm.transformar(pol, t, rng.uniform(0, 2 * np.pi), c0, ap)
        nivel = rng.uniform(25, 70) if claro else rng.uniform(170, 230)
        if real:
            nivel = rng.uniform(46, 54)
        centro = np.array(fm.momentos(px)[1])
        if palo:                                         # palo delgado desde el avión hasta fuera del cuadro
            a = rng.uniform(0, 2 * np.pi)
            fin = centro + 700 * np.array([np.cos(a), np.sin(a)])
            cv2.line(img, tuple(np.round(centro * S).astype(int)), tuple(np.round(fin * S).astype(int)), float(nivel),
                     int(round(rng.uniform(2, 4) * S)))
        poner(img, px, nivel)
    if distraer or lampara:
        distractores(img, rng, claro, lampara, centro)
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    img = cv2.GaussianBlur(img, (0, 0), 0.8) + rng.normal(0, 2.4 if real else 3.0, (H, W)).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8), centro


CASOS = [
    # (nombre, detector, argumentos de escena)
    ("avión de planta, fondo liso claro", "avion", dict(objetivo="avion_planta")),
    ("avión claro sobre fondo oscuro (polaridad claro)", "avion_claro", dict(objetivo="avion_planta", claro=False)),
    ("avión de planta chico (25–45 px)", "avion", dict(objetivo="avion_planta", tam=(25, 45))),
    ("avión de planta con distractores", "avion", dict(objetivo="avion_planta", distraer=True, lampara=True)),
    ("avión, niveles de la grabación real (50 sobre 64–99)", "avion", dict(objetivo="avion_planta", real=True)),
    ("sin avión, niveles de la grabación real", "avion", dict(objetivo=None, real=True)),
    ("avión de planta con palo de 2–4 px", "avion", dict(objetivo="avion_planta", palo=True)),
    ("avión de OTRO modelo (ala recta)", "avion", dict(objetivo="avion_otro")),
    ("sin avión, con distractores", "avion", dict(objetivo=None, distraer=True, lampara=True)),
    ("disco (círculo o elipse), fondo liso", "ovni", dict(objetivo="circulo", tam=(30, 90), inclinar=0.4)),
    ("disco con distractores, sin lámpara", "ovni", dict(objetivo="circulo", tam=(30, 90), inclinar=0.4, distraer=True)),
    ("disco con una lámpara redonda cerca", "ovni", dict(objetivo="circulo", tam=(30, 90), inclinar=0.4, lampara=True)),
    ("sin disco, con distractores, sin lámpara", "ovni", dict(objetivo=None, distraer=True)),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=300, help="escenas por caso")
    ap.add_argument("--guardar", help="carpeta donde dejar 3 escenas de ejemplo por caso (PNG con la detección marcada)")
    a = ap.parse_args()
    P = Parametros()
    import dataclasses
    params = {"avion": P.forma_avion, "ovni": P.forma_ovni,
              "avion_claro": dataclasses.replace(P.forma_avion, polaridad="claro")}
    if a.guardar:
        os.makedirs(a.guardar, exist_ok=True)
    print(f"{a.n} escenas por caso. Acierto: detección válida a menos de 5 px del objetivo.\n")
    print(f"{'caso':<50s}{'acierto':>9s}{'falso +':>9s}{'sin det.':>10s}{'error centroide':>17s}{'dist. mediana/p95':>19s}{'tiempo':>10s}")
    for i, (nombre, cual, kw) in enumerate(CASOS):
        rng = np.random.default_rng(100 + i)
        det = DetectorForma(params[cual])
        ok = fp = nada = 0
        err, dist, tms = [], [], []
        for k in range(a.n):
            img, c = escena(rng, **kw)
            det.reiniciar()                              # cada escena es independiente: búsqueda en cuadro completo
            r = det.procesar(img, usar_ventana=False)
            tms.append(r.t_proc_ms)
            if r.valido and c is not None and np.hypot(r.u - c[0], r.w - c[1]) < 5.0:
                ok += 1
                err.append(np.hypot(r.u - c[0], r.w - c[1]))
                dist.append(r.dist)
            elif r.valido:
                fp += 1
            else:
                nada += 1
            if a.guardar and k < 3:
                bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                if r.valido:
                    cv2.circle(bgr, (int(r.u), int(r.w)), 14, (0, 200, 0), 2)
                    cv2.putText(bgr, f"{r.forma} d={r.dist:.2f}", (int(r.u) + 18, int(r.w)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
                else:
                    cv2.putText(bgr, r.motivo, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
                cv2.imwrite(os.path.join(a.guardar, f"caso{i:02d}_{k}.png"), bgr)
        e = f"{np.sqrt(np.mean(np.square(err))):.2f} px RMS" if err else "—"
        d = f"{np.median(dist):.2f} / {np.percentile(dist, 95):.2f}" if dist else "—"
        print(f"{nombre:<50s}{100 * ok / a.n:7.1f} %{100 * fp / a.n:7.1f} %{100 * nada / a.n:8.1f} %{e:>17s}{d:>19s}{np.median(tms):7.2f} ms")

    #* SEGUIMIENTO: el mismo avión se mueve 4 px por cuadro y gira; el detector trabaja en la
    #* ventana alrededor de la detección anterior. Es el caso que corre a 80 cuadros por segundo.
    print("\nSecuencia de seguimiento (400 cuadros, objetivo de 80 px que se mueve y gira):")
    for cual, con_distractores in (("avion", False), ("avion", True), ("ovni", False), ("ovni", True)):
        rng, det = np.random.default_rng(7), DetectorForma(params[cual])
        pol = fm.poligono("avion_planta" if cual == "avion" else "circulo")
        ok, tv, tb, err, sin = 0, [], [], [], 0
        lienzo = fondo(rng, True)
        if con_distractores:
            #! Los distractores quedan fijos y el objetivo les pasa por delante: mientras se
            #! tocan, las dos figuras se funden en una mancha y el objetivo no se reconoce.
            distractores(lienzo, rng, True, False, np.array([320.0, 240.0]))
        for k in range(400):
            c0 = (320 + 200 * np.sin(0.02 * k), 240 + 120 * np.sin(0.031 * k + 1.0))
            px = fm.transformar(pol, 80, 0.01 * k, c0, (1.0, 0.75))
            img = lienzo.copy()
            poner(img, px, 45)
            img = cv2.GaussianBlur(cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA), (0, 0), 0.8)
            img = np.clip(img + rng.normal(0, 3.0, (H, W)), 0, 255).astype(np.uint8)
            hubo = det.ultimo is not None
            r = det.procesar(img, usar_ventana=True)
            (tv if hubo else tb).append(r.t_proc_ms)
            c = np.array(fm.momentos(px)[1])
            if r.valido and np.hypot(r.u - c[0], r.w - c[1]) < 5:
                ok += 1
                err.append(np.hypot(r.u - c[0], r.w - c[1]))
            sin = 0 if r.valido else sin + 1
            if sin >= 24:                                # como el supervisor: 0,3 s sin detección -> PÉRDIDA
                det.reiniciar()                          # y main.py reinicia la ventana del detector
        print(f"  {cual:<6s}{'con' if con_distractores else 'sin'} distractores: acierto {100 * ok / 400:5.1f} %   "
              f"error del centroide {np.sqrt(np.mean(np.square(err))):.2f} px RMS   tiempo siguiendo: mediana {np.median(tv):.2f} ms, "
              f"p95 {np.percentile(tv, 95):.2f} ms, máx {np.max(tv):.2f} ms")


if __name__ == "__main__":
    main()
