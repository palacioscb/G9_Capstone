#!/usr/bin/env python3
"""
Torreta - detector de avión rojo, mínima latencia (Raspberry Pi + Picamera2)

Idea: en YUV420 el rojo tiene el plano V (Cr) alto y el fondo blanco/gris queda en ~128.
Se umbraliza SOLO el plano V (mitad de resolución -> 4x menos píxeles que Y) y se
calcula el centroide. Búsqueda global al inicio / al perder el objetivo; ROI al seguir.

Uso:
    python3 avion_rojo.py                # con ventana
    python3 avion_rojo.py --sin-ventana  # mínima latencia (sin HDMI)
    python3 avion_rojo.py --calibrar     # ayuda para fijar UMBRAL_V
"""
import argparse
import gc
import os
import time
from datetime import datetime

import numpy as np

# ----------------------------- PARÁMETROS -----------------------------
W, H = 640, 480            # resolución de trabajo (el plano V es de W/2 x H/2)
FPS_OBJETIVO = 120         # el sensor lo limita si no llega
EXPOSICION_MAX_US = 4000   # tope de exposición (menos desenfoque); si AE pide más, sube la ganancia
AUTO_FRAMES = 40           # frames para que AE/AWB converjan antes de congelarlos
UMBRAL_V = 155             # V mínimo para considerar "rojo" (pared ~128). Ajustar con --calibrar
MIN_PIX = 12               # píxeles (del plano V) mínimos para aceptar la detección
B_GLOBAL = 4               # bloque de la búsqueda global (en píxeles del plano V)
ROI_R = 40                 # radio de la ventana de seguimiento (px del plano V = 2x en imagen completa)
MOSTRAR_CADA = 4           # refrescar ventana cada N frames
DIR_CAPTURAS = "capturas"

# --- Cámara (para convertir error en píxeles a ángulos) ---
FOV_H = 62.2               # grados (Camera Module v2). Ajustar si usas otro lente
FOV_V = 48.8

RELOJ = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)
_IDX = np.arange(W, dtype=np.int64)


# ----------------------------- DETECCIÓN -----------------------------
def centroide_rojo(v, cx, cy, r, umbral=UMBRAL_V, min_pix=MIN_PIX):
    """Centroide ponderado (subpíxel) de lo 'rojo' en una ventana de radio r. Coords del plano V."""
    h, w = v.shape
    x0, x1 = max(cx - r, 0), min(cx + r, w)
    y0, y1 = max(cy - r, 0), min(cy + r, h)
    win = v[y0:y1, x0:x1]
    if int(win.max()) < umbral:                 # salida rápida
        return None
    p = win.astype(np.int16) - umbral           # peso = qué tan rojo está sobre el umbral
    np.maximum(p, 0, out=p)
    n = int(np.count_nonzero(p))
    if n < min_pix:
        return None
    s = int(p.sum())
    mx = x0 + int(p.sum(axis=0) @ _IDX[: x1 - x0]) / s
    my = y0 + int(p.sum(axis=1) @ _IDX[: y1 - y0]) / s
    return mx, my, n


def buscar_global(v, b=B_GLOBAL, umbral=UMBRAL_V):
    """Bloque con más 'rojo' en todo el cuadro, refinado con centroide (2 pasadas)."""
    if int(v.max()) < umbral:
        return None
    h, w = v.shape
    hb, wb = h // b, w // b
    p = v.astype(np.int16) - umbral
    np.maximum(p, 0, out=p)
    suma = p[: hb * b, : wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.int32)
    by, bx = divmod(int(suma.argmax()), wb)
    res = centroide_rojo(v, bx * b + b // 2, by * b + b // 2, ROI_R)
    if res is None:
        return None
    return centroide_rojo(v, int(res[0]), int(res[1]), ROI_R)   # recentra (la ventana inicial pudo quedar descentrada)


class Detector:
    """Devuelve (x, y, n) en coordenadas de la imagen completa (W x H)."""

    def __init__(self):
        self.ultimo = None  # (x, y) entero en el plano V

    def detectar(self, v):
        res = None
        if self.ultimo is not None:
            res = centroide_rojo(v, self.ultimo[0], self.ultimo[1], ROI_R)
        if res is None:
            res = buscar_global(v)
        if res is None:
            self.ultimo = None
            return None
        self.ultimo = (int(res[0]), int(res[1]))
        return 2 * res[0] + 0.5, 2 * res[1] + 0.5, res[2]   # plano V -> imagen completa


# ----------------------------- SALIDA (protocolo por definir) -----------------------------
def calcular_angulos(ex, ey):
    """Error en píxeles (respecto al centro) -> ángulos en grados.
    Aquí se enchufará después el envío (UART/ESP32) de la rama principal."""
    return ex * (FOV_H / W), ey * (FOV_V / H)


# ----------------------------- CÁMARA -----------------------------
def fijar_exposicion(picam2):
    """Deja converger AE/AWB (sin el avión en cuadro), luego congela exposición, ganancia y balance de blancos."""
    md = {}
    for _ in range(AUTO_FRAMES):
        md = picam2.capture_metadata()
    exp, gan, cg = md["ExposureTime"], md["AnalogueGain"], md["ColourGains"]
    if exp > EXPOSICION_MAX_US:
        gan = min(gan * exp / EXPOSICION_MAX_US, 16.0)
        exp = EXPOSICION_MAX_US
    picam2.set_controls({
        "AeEnable": False, "AwbEnable": False,
        "ExposureTime": int(exp), "AnalogueGain": float(gan), "ColourGains": cg,
    })
    for _ in range(6):                      # descarta frames hasta que se apliquen los controles
        picam2.capture_metadata()
    print(f"Fijado: exposición {int(exp)} us | ganancia {gan:.2f} | ColourGains {tuple(round(c, 2) for c in cg)}")


def planos(arr):
    """Y (H x W) y V (H/2 x W/2) del YUV420, sin copia."""
    y = arr[:H, :W]
    v = arr[H + H // 4: H + H // 2, :W].reshape(H // 2, W // 2)
    return y, v


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


def dibujar(cv2, gris, res, m, fps):
    img = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
    cx, cy = W // 2, H // 2
    cv2.drawMarker(img, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 25, 2)
    if res is not None:
        tx, ty = int(res[0]), int(res[1])
        cv2.circle(img, (tx, ty), 14, (0, 255, 0), 2)
        cv2.line(img, (cx, cy), (tx, ty), (0, 255, 255), 2)
        estado = f"dX:{res[0]-cx:+.1f} dY:{res[1]-cy:+.1f} ({int(res[2])} px)"
    else:
        estado = "Buscando avion..."
    f = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, f"FPS: {fps:.0f}   {estado}", (10, 25), f, 0.6, (0, 255, 0), 2)
    cv2.putText(img, f"Captura {m['cap'].v:.2f} | Deteccion {m['det'].v:.3f} | Angulos {m['ang'].v:.3f} ms",
                (10, 50), f, 0.55, (0, 255, 255), 1)
    cv2.putText(img, f"TOTAL sensor->controlador: {m['tot'].v:.2f} ms (max 1s: {m['tot'].max:.2f})",
                (10, 75), f, 0.6, (0, 200, 255), 2)
    cv2.putText(img, "[S] Foto | [Q] Salir", (10, H - 12), f, 0.5, (255, 255, 255), 1)
    return img


def modo_calibracion(picam2, cv2, MappedArray):
    """1) Sin el avión en cuadro, pulsa P -> guarda el V máximo del fondo (ruido).
       2) Cubre el recuadro verde con el avión, pulsa O -> guarda el V del avión.
       Se sugiere UMBRAL_V a mitad de camino."""
    pared, obj = None, None
    print("CALIBRACION: [P] fondo sin avion | [O] avion dentro del recuadro | [Q] salir")
    while True:
        req = picam2.capture_request()
        with MappedArray(req, "main") as ma:
            a = ma.array
            gris = a[:H, :W].copy()
            u = a[H: H + H // 4, :W].reshape(H // 2, W // 2)
            v = a[H + H // 4: H + H // 2, :W].reshape(H // 2, W // 2)
            cy, cx = H // 4, W // 4
            vc = float(v[cy - 4: cy + 4, cx - 4: cx + 4].mean())
            uc = float(u[cy - 4: cy + 4, cx - 4: cx + 4].mean())
            vmax = int(v.max())
        req.release()

        img = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(img, (W // 2 - 8, H // 2 - 8), (W // 2 + 8, H // 2 + 8), (0, 255, 0), 1)
        f = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"Centro: V={vc:.0f} U={uc:.0f} | V max cuadro={vmax}", (10, 25), f, 0.6, (0, 255, 0), 2)
        cv2.putText(img, f"Fondo(Vmax)={pared} | Avion(V)={obj}", (10, 50), f, 0.6, (0, 255, 255), 2)
        if pared is not None and obj is not None:
            txt = (f"UMBRAL_V sugerido: {(pared + obj) / 2:.0f}" if obj > pared
                   else "El avion no supera al fondo: revisar luz/exposicion")
            cv2.putText(img, txt, (10, 75), f, 0.6, (0, 200, 255), 2)
        cv2.imshow("Calibracion", img)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("p"):
            pared = vmax
        elif k == ord("o"):
            obj = round(vc)
        elif k == ord("q"):
            break
    cv2.destroyAllWindows()


# ----------------------------- PRINCIPAL -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sin-ventana", action="store_true", help="no abrir ventana HDMI")
    ap.add_argument("--calibrar", action="store_true", help="ayuda para fijar UMBRAL_V")
    args = ap.parse_args()

    from picamera2 import Picamera2, MappedArray
    from libcamera import controls as lc

    cv2 = None
    if not args.sin_ventana or args.calibrar:
        import cv2 as _cv2
        cv2 = _cv2
        cv2.namedWindow("Torreta - Avion", cv2.WINDOW_NORMAL)
        os.makedirs(DIR_CAPTURAS, exist_ok=True)

    dur_us = int(1e6 / FPS_OBJETIVO)
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (W, H), "format": "YUV420"},
        buffer_count=3,
        queue=False,
        controls={
            "AeEnable": True,           # solo durante el arranque; luego se congela
            "AwbEnable": True,
            "FrameDurationLimits": (dur_us, dur_us),
            "NoiseReductionMode": lc.draft.NoiseReductionModeEnum.Off,
        },
    )
    picam2.configure(cfg)
    picam2.start()
    print("Modo de sensor:", picam2.camera_configuration()["sensor"])

    stride = picam2.stream_configuration("main")["stride"]
    if stride != W:
        print(f"AVISO: stride={stride} != {W}; el plano V no se extrae bien. Usa un ancho multiplo de 32.")

    print("Ajustando exposicion y balance de blancos (no pongas el avion en cuadro)...")
    fijar_exposicion(picam2)

    if args.calibrar:
        modo_calibracion(picam2, cv2, MappedArray)
        picam2.stop()
        return

    try:
        os.sched_setaffinity(0, {os.cpu_count() - 1})
    except Exception:
        pass

    det = Detector()
    m = {k: Metrica() for k in ("cap", "det", "ang", "tot")}
    fps, t_prev = 0.0, time.perf_counter()
    t_win, n, fotos = t_prev, 0, 0
    ts_avisado = False

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
                y, v = planos(ma.array)                 # vistas, sin copia

                t0 = time.perf_counter_ns()
                res = det.detectar(v)
                t1 = time.perf_counter_ns()
                if res is not None:
                    ang_x, ang_y = calcular_angulos(res[0] - W / 2, res[1] - H / 2)
                    # TODO: aquí va el envío del protocolo (ang_x, ang_y)
                t2 = time.perf_counter_ns()
                t_fin = time.clock_gettime_ns(RELOJ)    # dato listo para el controlador

                gris = y.copy() if mostrar else None
            req.release()

            # --- métricas (ms) ---
            m["det"].add((t1 - t0) / 1e6)
            m["ang"].add((t2 - t1) / 1e6)
            if ts:
                cap, tot = (t_cap - ts) / 1e6, (t_fin - ts) / 1e6
                if not ts_avisado and not (0 <= tot < 500):
                    print("AVISO: SensorTimestamp no es comparable con el reloj usado; "
                          "probar RELOJ = time.CLOCK_MONOTONIC.")
                    ts_avisado = True
            else:
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
                pos = "sin objetivo" if res is None else f"dX={res[0]-W/2:+.1f} dY={res[1]-H/2:+.1f}"
                print(f"FPS {fps:5.1f} | captura {m['cap'].v:5.2f} | deteccion {m['det'].v:6.3f} | "
                      f"angulos {m['ang'].v:5.3f} | TOTAL {m['tot'].v:5.2f} ms (max {m['tot'].max:5.2f}) | {pos}")

            # --- ventana (fuera de la ruta crítica) ---
            if mostrar:
                img = dibujar(cv2, gris, res, m, fps)
                cv2.imshow("Torreta - Avion", img)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("s"):
                    fotos += 1
                    fn = os.path.join(DIR_CAPTURAS, f"avion_{fotos}_{datetime.now():%Y%m%d_%H%M%S}.jpg")
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
