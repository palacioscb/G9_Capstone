#!/usr/bin/env python3
"""
Torreta - detección de PUNTO LÁSER ROJO sobre pared, mínima latencia (Raspberry Pi + Picamera2)

Base: torreta_rapida.py (misma estructura y MISMA métrica de latencia sensor -> controlador).

Qué cambia respecto a la versión de "luz brillante":
  * El láser se distingue por COLOR, no solo por brillo. Se usa el plano V (Cr) del YUV420
    que entrega el ISP: V alto = rojo. Una lámpara blanca da V ~ 128 (neutro) y NO dispara.
        - Costo cero: el ISP ya lo calculó, es un plano más del mismo buffer (sin copia).
        - Mitad de resolución (320x240 para 640x480): 4x menos píxeles que recorrer.
  * Compuerta de brillo con el plano Y (ventana de 7x7 alrededor del candidato): descarta
    objetos rojos oscuros (ropa, carteles) cuyo V es alto pero que no brillan como un láser.
  * Exposición muy corta y ganancia baja: la pared (iluminada por la sala) queda oscura y
    el punto láser, que es mucho más intenso, destaca solo. Esto además baja el tiempo de
    exposición que forma parte de la latencia.
  * Igual que antes: búsqueda global -> ROI una vez enganchado, centroide subpíxel,
    queue=False, GC desactivado, afinidad a un núcleo, ventana opcional cada N frames.

Uso:
    python3 torreta_laser_rojo.py                          # con ventana
    python3 torreta_laser_rojo.py --sin-ventana            # solo terminal (mide lo más puro)
    python3 torreta_laser_rojo.py --exposicion 500 --ganancia 1.0

Calibración rápida: mira la columna "pico V" en la terminal. Con el láser apagado debe
quedar cerca de 128-150; con el láser encendido sobre la pared debe superar V_UMBRAL.
Si no sube, baja la exposición (pared demasiado clara) o sube la ganancia de rojo
(COLOR_GAINS[0]).
"""
import argparse
import gc
import os
import time
from datetime import datetime

import numpy as np

# ----------------------------- PARÁMETROS -----------------------------
W, H = 640, 480            # resolución de trabajo (múltiplo de 32 en ancho evita padding)
FPS_OBJETIVO = 120         # el sensor lo limita si no llega
EXPOSICION_US = 1000       # exposición fija MUY corta (bajar hasta que la pared se vea casi negra)
GANANCIA = 1.0             # ganancia analógica fija
COLOR_GAINS = (1.0, 1.0)   # (rojo, azul) fijos, sin AWB. Subir el rojo si el pico V es bajo
V_UMBRAL = 170             # pico mínimo del plano V (128 = neutro, 255 = rojo puro)
Y_MIN = 60                 # brillo mínimo (0-255) del punto en el plano Y. 0 = desactivar
MIN_PIX = 2                # píxeles mínimos (en el plano V) de la zona roja
B_GLOBAL = 4               # tamaño de bloque en búsqueda global (plano V -> 8 px reales)
B_ROI = 4                  # tamaño de bloque dentro de la ROI
ROI_R = 24                 # radio de la ventana de seguimiento en el plano V (=48 px reales)
MOSTRAR_CADA = 4           # refrescar ventana cada N frames
DIR_CAPTURAS = "capturas"

RELOJ = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)
_IDX = np.arange(max(W, H) + 8, dtype=np.int64)
V_NEUTRO = 128


# ----------------------------- DETECCIÓN -----------------------------
def centroide_local(v, cx, cy, r, umbral=V_UMBRAL, min_pix=MIN_PIX):
    """Centroide ponderado (subpíxel) de la zona roja alrededor de (cx, cy) en el plano V."""
    h, w = v.shape
    x0, x1 = max(cx - r, 0), min(cx + r, w)
    y0, y1 = max(cy - r, 0), min(cy + r, h)
    ven = v[y0:y1, x0:x1]
    pico = int(ven.max())
    if pico < umbral:
        return None
    corte = V_NEUTRO + ((pico - V_NEUTRO) * 7) // 10   # pesos: solo lo cercano al pico
    p = ven.astype(np.int16) - corte
    np.maximum(p, 0, out=p)
    if int(np.count_nonzero(p)) < min_pix:
        return None
    s = int(p.sum())
    mx = x0 + int(p.sum(axis=0) @ _IDX[: x1 - x0]) / s
    my = y0 + int(p.sum(axis=1) @ _IDX[: y1 - y0]) / s
    return mx, my, pico


def buscar_zona(v, b, umbral=V_UMBRAL):
    """Encuentra la zona (bloque b x b) más roja y refina con centroide."""
    if int(v.max()) < umbral:            # salida rápida: no hay nada rojo que buscar
        return None
    h, w = v.shape
    hb, wb = h // b, w // b
    suma = v[: hb * b, : wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.uint16)
    by, bx = divmod(int(suma.argmax()), wb)
    return centroide_local(v, bx * b + b // 2, by * b + b // 2, (3 * b) // 2, umbral)


def confirma_brillo(y, x, yy):
    """El láser es rojo Y brillante: verifica el plano Y en una ventana 7x7 (costo ~ nulo)."""
    if Y_MIN <= 0:
        return True
    xi, yi = int(x), int(yy)
    return int(y[max(yi - 3, 0): yi + 4, max(xi - 3, 0): xi + 4].max()) >= Y_MIN


class Detector:
    """Búsqueda global al inicio / al perder el objetivo; ROI cuando está enganchado.
    Trabaja en el plano V (mitad de resolución) y devuelve coordenadas en píxeles del plano Y."""

    def __init__(self):
        self.ultimo = None  # (x, y) entero en coordenadas del plano V

    def _roi(self, v):
        h, w = v.shape
        ux, uy = self.ultimo
        x0, x1 = max(ux - ROI_R, 0), min(ux + ROI_R, w)
        y0, y1 = max(uy - ROI_R, 0), min(uy + ROI_R, h)
        res = buscar_zona(v[y0:y1, x0:x1], B_ROI)
        if res is None:
            return None
        return res[0] + x0, res[1] + y0, res[2]

    def detectar(self, v, y):
        res = None
        if self.ultimo is not None:
            res = self._roi(v)
            if res is not None and not confirma_brillo(y, 2 * res[0] + 0.5, 2 * res[1] + 0.5):
                res = None
        if res is None:
            res = buscar_zona(v, B_GLOBAL)
            if res is not None and not confirma_brillo(y, 2 * res[0] + 0.5, 2 * res[1] + 0.5):
                res = None
        if res is None:
            self.ultimo = None
            return None
        self.ultimo = (int(res[0]), int(res[1]))
        # muestra V (i, j) cubre los píxeles reales 2j..2j+1 -> centro en 2j + 0.5
        return 2 * res[0] + 0.5, 2 * res[1] + 0.5, res[2]


# ----------------------------- CONTROLADOR -----------------------------
def enviar_al_controlador(ex, ey):
    """Punto de salida del error hacia el controlador.
    Reemplazar por tu canal real. Ejemplo por serial (pyserial, abrir el puerto una sola vez):
        ser.write(struct.pack('<ff', ex, ey))
    Se mide su duración, así que queda incluido en la latencia total.
    """
    return


# ----------------------------- UTILIDADES -----------------------------
class Metrica:
    """Promedio móvil exponencial + máximo de la ventana de 1 s."""

    def __init__(self, alpha=0.05):
        self.a, self.v, self.max, self._max = alpha, None, 0.0, 0.0

    def add(self, x):
        self.v = x if self.v is None else self.v + self.a * (x - self.v)
        self._max = max(self._max, x)

    def cerrar_ventana(self):
        self.max, self._max = self._max, 0.0


def dibujar(cv2, gris, res, m, fps, pico_v):
    img = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
    cx, cy = W // 2, H // 2
    cv2.drawMarker(img, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 25, 2)
    if res is not None:
        tx, ty = int(res[0]), int(res[1])
        cv2.circle(img, (tx, ty), 12, (0, 0, 255), 2)
        cv2.line(img, (cx, cy), (tx, ty), (0, 255, 255), 2)
        estado = f"dX:{res[0]-cx:+.1f} dY:{res[1]-cy:+.1f}"
    else:
        estado = "Buscando laser..."
    f = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, f"FPS: {fps:.0f}   {estado}   pico V: {pico_v}", (10, 25), f, 0.6, (0, 255, 0), 2)
    cv2.putText(img, f"Captura {m['cap'].v:.2f} | Deteccion {m['det'].v:.3f} | Envio {m['env'].v:.3f} ms",
                (10, 50), f, 0.55, (0, 255, 255), 1)
    cv2.putText(img, f"TOTAL sensor->controlador: {m['tot'].v:.2f} ms (max 1s: {m['tot'].max:.2f})",
                (10, 75), f, 0.6, (0, 200, 255), 2)
    cv2.putText(img, "[S] Foto | [Q] Salir", (10, H - 12), f, 0.5, (255, 255, 255), 1)
    return img


# ----------------------------- PRINCIPAL -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sin-ventana", action="store_true", help="no abrir ventana HDMI")
    ap.add_argument("--exposicion", type=int, default=EXPOSICION_US, help="exposición en µs")
    ap.add_argument("--ganancia", type=float, default=GANANCIA, help="ganancia analógica")
    args = ap.parse_args()

    from picamera2 import Picamera2, MappedArray
    from libcamera import controls as lc

    cv2 = None
    if not args.sin_ventana:
        import cv2 as _cv2
        cv2 = _cv2
        cv2.namedWindow("Torreta - Laser rojo", cv2.WINDOW_NORMAL)
        os.makedirs(DIR_CAPTURAS, exist_ok=True)

    dur_us = int(1e6 / FPS_OBJETIVO)
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (W, H), "format": "YUV420"},
        buffer_count=3,
        queue=False,  # siempre un frame nuevo; probar queue=True si el sensor va más rápido que el loop
        controls={
            "AeEnable": False,
            "AwbEnable": False,
            "ColourGains": COLOR_GAINS,
            "ExposureTime": args.exposicion,
            "AnalogueGain": args.ganancia,
            "FrameDurationLimits": (dur_us, dur_us),
            "NoiseReductionMode": lc.draft.NoiseReductionModeEnum.Off,
        },
    )
    picam2.configure(cfg)
    picam2.start()
    print("Modo de sensor:", picam2.camera_configuration()["sensor"])

    try:  # opcional: fijar el proceso a un núcleo (reduce jitter)
        os.sched_setaffinity(0, {os.cpu_count() - 1})
    except Exception:
        pass

    det = Detector()
    m = {k: Metrica() for k in ("cap", "det", "env", "tot")}
    fps, t_prev = 0.0, time.perf_counter()
    t_win, n, fotos = t_prev, 0, 0
    ts_avisado = False
    pico_v = 0

    gc.collect()
    gc.disable()  # evita pausas del recolector en el loop crítico
    print("Corriendo. Ctrl+C o 'q' para salir." if cv2 else "Corriendo sin ventana. Ctrl+C para salir.")

    try:
        while True:
            req = picam2.capture_request()
            t_cap = time.clock_gettime_ns(RELOJ)
            ts = req.get_metadata().get("SensorTimestamp")
            mostrar = cv2 is not None and (n % MOSTRAR_CADA == 0)
            with MappedArray(req, "main") as ma:
                arr = ma.array                          # (H*3/2, stride): Y | U | V planares
                stride = arr.shape[1]
                y = arr[:H, :W]                         # plano Y (brillo), sin copia
                v = arr[H + H // 4: H + H // 2].reshape(H // 2, stride // 2)[:, : W // 2]  # plano V (rojo), sin copia

                t0 = time.perf_counter_ns()
                res = det.detectar(v, y)
                t1 = time.perf_counter_ns()
                if res is not None:
                    enviar_al_controlador(res[0] - W / 2, res[1] - H / 2)
                t2 = time.perf_counter_ns()
                t_fin = time.clock_gettime_ns(RELOJ)    # dato listo para el controlador

                gris = y.copy() if mostrar else None
            req.release()
            pico_v = 0 if res is None else res[2]

            # --- métricas (ms) ---
            m["det"].add((t1 - t0) / 1e6)
            m["env"].add((t2 - t1) / 1e6)
            if ts:
                cap, tot = (t_cap - ts) / 1e6, (t_fin - ts) / 1e6
                if not ts_avisado and not (0 <= tot < 500):
                    print("AVISO: SensorTimestamp no es comparable con el reloj usado; "
                          "probar RELOJ = time.CLOCK_MONOTONIC.")
                    ts_avisado = True
            else:  # sin timestamp: solo costo de procesamiento propio
                cap, tot = 0.0, (t_fin - t_cap) / 1e6
            m["cap"].add(cap)
            m["tot"].add(tot)

            n += 1
            ahora = time.perf_counter()
            fps = 0.95 * fps + 0.05 / (ahora - t_prev) if fps else 1.0 / (ahora - t_prev)
            t_prev = ahora
            if ahora - t_win >= 1.0:
                for mm in m.values():
                    mm.cerrar_ventana()
                t_win = ahora
                pos = "sin laser" if res is None else f"dX={res[0]-W/2:+.1f} dY={res[1]-H/2:+.1f}"
                print(f"FPS {fps:5.1f} | captura {m['cap'].v:5.2f} | deteccion {m['det'].v:6.3f} | "
                      f"envio {m['env'].v:5.3f} | TOTAL {m['tot'].v:5.2f} ms (max {m['tot'].max:5.2f}) | "
                      f"pico V {pico_v:3d} | {pos}")

            # --- ventana (fuera de la ruta crítica) ---
            if mostrar:
                img = dibujar(cv2, gris, res, m, fps, pico_v)
                cv2.imshow("Torreta - Laser rojo", img)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("s"):
                    fotos += 1
                    fn = os.path.join(DIR_CAPTURAS, f"laser_{fotos}_{datetime.now():%Y%m%d_%H%M%S}.jpg")
                    cv2.imwrite(fn, img)
                    print(f"[FOTO {fotos}] {fn}")
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        if cv2:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
