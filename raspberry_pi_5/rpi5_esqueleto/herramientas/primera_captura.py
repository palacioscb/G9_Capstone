#!/usr/bin/env python3
"""
primera_captura.py — Primer contacto con la cámara desde Python (Picamera2).

Captura UN cuadro, lo convierte en una matriz NumPy y muestra qué hay dentro:
dimensiones, tipo de dato, memoria, valores de un píxel y el recorte del sensor
que se usó. Guarda la imagen (.jpg) y la matriz cruda (.npy).

Uso (en la Raspberry Pi):
    python3 primera_captura.py                 # 640x480 por defecto
    python3 primera_captura.py --ancho 1640 --alto 1232

Proyecto IEE2913 — Grupo 9. Bloque: Adquisición visual / Procesamiento de imagen.
"""
import argparse
import time

import numpy as np
from picamera2 import Picamera2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ancho", type=int, default=640)
    ap.add_argument("--alto", type=int, default=480)
    ap.add_argument("--nombre", default="primera")
    ap.add_argument("--modo-sensor", default="1640x1232",
                    help="modo del sensor (AnchoxAlto). 1640x1232 = campo completo; "
                         "'auto' deja que libcamera elija (a 640x480 elige el modo RECORTADO de 28°)")
    args = ap.parse_args()

    picam2 = Picamera2()

    # --- Qué cámara detectó libcamera ---------------------------------------
    props = picam2.camera_properties
    print("Modelo del sensor     :", props.get("Model"))
    print("Arreglo de píxeles    :", props.get("PixelArraySize"), "(ancho, alto) en píxeles nativos")
    print("Tamaño de celda       :", props.get("UnitCellSize"), "(nm)")

    # --- Configuración: flujo 'main' en RGB888 (3 bytes por píxel) ----------
    # OJO: en Picamera2, "RGB888" deja los bytes en orden [B, G, R] (igual que OpenCV).
    extra = {}
    if args.modo_sensor != "auto":
        extra["sensor"] = {"output_size": tuple(int(x) for x in args.modo_sensor.split("x"))}
    cfg = picam2.create_still_configuration(
        main={"size": (args.ancho, args.alto), "format": "RGB888"}, **extra
    )
    picam2.configure(cfg)
    real = picam2.camera_configuration()
    print("Modo del sensor usado :", real["sensor"])
    print("Tamaño de salida real :", real["main"]["size"])

    picam2.start()
    time.sleep(1.5)  # deja converger exposición automática y balance de blancos

    req = picam2.capture_request()
    matriz = req.make_array("main")   # copia a un ndarray de NumPy
    md = req.get_metadata()
    req.save("main", f"{args.nombre}.jpg")
    req.release()
    picam2.stop()

    np.save(f"{args.nombre}.npy", matriz)

    # --- Qué es la "matriz" -------------------------------------------------
    alto, ancho, canales = matriz.shape
    print("\n=== La imagen como matriz ===")
    print(f"shape  = {matriz.shape}  -> (filas v, columnas u, canales)")
    print(f"dtype  = {matriz.dtype}  -> cada canal es un entero 0..255")
    print(f"memoria= {matriz.nbytes:,} bytes = {matriz.nbytes/1e6:.2f} MB por cuadro")
    uc, vc = ancho // 2, alto // 2
    b, g, r = matriz[vc, uc]
    print(f"píxel central (u={uc}, v={vc}) -> B={b} G={g} R={r}")
    print("mín/máx por canal (B,G,R):",
          [(int(matriz[..., c].min()), int(matriz[..., c].max())) for c in range(canales)])
    # Punto más brillante del canal rojo (anticipo de la detección del láser)
    rojo = matriz[..., 2]
    v, u = np.unravel_index(np.argmax(rojo), rojo.shape)
    print(f"píxel más rojo: (u={u}, v={v}) valor={rojo[v, u]}")
    print(f"error respecto al centro: e_u={u-uc:+d} px, e_v={v-vc:+d} px")

    print("\n=== Metadatos del cuadro ===")
    for k in ("SensorTimestamp", "ExposureTime", "AnalogueGain", "FrameDuration", "ScalerCrop", "Lux"):
        print(f"{k:16s}: {md.get(k)}")
    print(f"\nGuardado: {args.nombre}.jpg y {args.nombre}.npy")


if __name__ == "__main__":
    main()
