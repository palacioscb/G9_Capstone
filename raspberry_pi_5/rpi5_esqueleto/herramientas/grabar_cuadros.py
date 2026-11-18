"""
grabar_cuadros.py — Graba cuadros de la cámara (plano Y) para probar la detección SIN la torreta.

Sirve cuando no se puede montar la torreta en el lugar ideal: basta apuntar la cámara de la RPi a la
escena (en la mano, sobre un trípode, apoyada), poner el avión a una distancia conocida y grabar. Los
cuadros se llevan al PC y allí se prueba el detector cuantas veces haga falta, con
herramientas/reproducir_forma.py; también sirven para enseñar las vistas del avión
(herramientas/aprender_forma.py --imagen).

Uso (en la RPi, desde rpi5_esqueleto/):
  python3 herramientas/grabar_cuadros.py --etiqueta avion_lado_2m --distancia 2 --exposicion 6000 --ganancia 2
  python3 herramientas/grabar_cuadros.py --etiqueta avion_arriba_3m --distancia 3 --cuadros 60 --cada 0.2
  python3 herramientas/grabar_cuadros.py --etiqueta sin_avion --cuadros 30      # escena vacía: falsos positivos

Deja en registros/capturas/<etiqueta>/: 000.png, 001.png, ... (luminancia, sin pérdida), meta.json y
contacto.jpg (miniaturas, para ver a ojo qué quedó grabado). Las etiquetas que empiezan por "sin_" se
toman como escenas sin avión. Para llevarlas al PC:
  scp -r usuario@g9-cap.local:<ruta>/rpi5_esqueleto/registros/capturas .
Mover el avión durante la grabación (o grabar varias veces) da más variedad que un solo cuadro.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

RAIZ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, RAIZ)
from pt.parametros import Parametros                              # noqa: E402


def hoja_de_contacto(miniaturas, ruta, columnas=5):
    if not miniaturas:
        return
    h, w = miniaturas[0].shape
    filas = (len(miniaturas) + columnas - 1) // columnas
    hoja = np.zeros((filas * h, columnas * w), np.uint8)
    for i, m in enumerate(miniaturas):
        r, c = divmod(i, columnas)
        hoja[r * h:(r + 1) * h, c * w:(c + 1) * w] = m
    cv2.imwrite(ruta, hoja, [int(cv2.IMWRITE_JPEG_QUALITY), 85])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--etiqueta", required=True, help="nombre de la carpeta, p. ej. avion_lado_2m; 'sin_...' = sin avión")
    ap.add_argument("--distancia", type=float, help="distancia cámara-avión [m] (queda en meta.json)")
    ap.add_argument("--cuadros", type=int, default=40, help="cuadros a guardar (por defecto 40)")
    ap.add_argument("--cada", type=float, default=0.25, help="separación entre cuadros guardados [s] (por defecto 0,25)")
    ap.add_argument("--espera", type=int, default=5, help="cuenta regresiva antes de grabar [s], para poner el avión (por defecto 5)")
    ap.add_argument("--exposicion", type=int, help="µs; usar la misma que en la demostración")
    ap.add_argument("--ganancia", type=float, help="ganancia analógica; usar la misma que en la demostración")
    ap.add_argument("--nota", default="", help="texto libre para meta.json (fondo, luz, cara del avión...)")
    ap.add_argument("--salida", default=os.path.join(RAIZ, "registros", "capturas"))
    a = ap.parse_args()

    P = Parametros()
    if a.exposicion:
        P.captura.exposicion_us = a.exposicion
    if a.ganancia:
        P.captura.ganancia = a.ganancia
    carpeta = os.path.join(a.salida, a.etiqueta)
    os.makedirs(carpeta, exist_ok=True)

    for s in range(a.espera, 0, -1):
        print(f"Grabando '{a.etiqueta}' en {s} s ... (poner el avión)   ", end="\r", flush=True)
        time.sleep(1)
    from pt.camara import FuenteCamara                  # tardío: sólo existe en la RPi
    fuente = FuenteCamara(P.captura, None)
    miniaturas, sat, osc, meta_cam = [], [], [], {}
    ultimo_t = None
    try:
        for k, c in enumerate(fuente):
            if k < 20:                                  # los primeros cuadros: la exposición se asienta
                continue
            t = c.t_entrega_ns / 1e9
            if ultimo_t is not None and t - ultimo_t < a.cada:
                continue
            ultimo_t = t
            y = np.ascontiguousarray(c.img)
            n = len(miniaturas)
            cv2.imwrite(os.path.join(carpeta, f"{n:03d}.png"), y)
            miniaturas.append(cv2.resize(y, (192, 144), interpolation=cv2.INTER_AREA))
            sat.append(float((y >= 250).mean()))
            osc.append(float((y <= 10).mean()))
            meta_cam = {kk: (c.meta or {}).get(kk) for kk in ("ExposureTime", "AnalogueGain") if (c.meta or {}).get(kk) is not None}
            print(f"  cuadro {n + 1}/{a.cuadros}      ", end="\r", flush=True)
            if n + 1 >= a.cuadros:
                break
    except KeyboardInterrupt:
        print("\nInterrumpido: se conserva lo grabado.")
    finally:
        fuente.cerrar()

    meta = {"etiqueta": a.etiqueta, "distancia_m": a.distancia, "cuadros": len(miniaturas), "cada_s": a.cada,
            "exposicion_us": P.captura.exposicion_us, "ganancia": P.captura.ganancia, "salida_px": list(P.captura.salida_px),
            "aplicado_por_la_camara": meta_cam, "fecha": datetime.now().isoformat(timespec="seconds"), "nota": a.nota}
    with open(os.path.join(carpeta, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    hoja_de_contacto(miniaturas, os.path.join(carpeta, "contacto.jpg"))
    if not miniaturas:
        sys.exit("\nNo se grabó ningún cuadro.")
    print(f"\nGrabados {len(miniaturas)} cuadros en {carpeta}")
    print(f"  píxeles saturados (>=250): {100 * np.mean(sat):.1f} %   muy oscuros (<=10): {100 * np.mean(osc):.1f} %")
    if np.mean(sat) > 0.5:
        print("  AVISO: más de la mitad de la imagen está saturada: bajar la exposición o la ganancia.")
    if np.mean(osc) > 0.5:
        print("  AVISO: más de la mitad de la imagen está negra: subir la exposición o la ganancia.")
    print(f"  Ver a ojo: {os.path.join(carpeta, 'contacto.jpg')}")


if __name__ == "__main__":
    main()
