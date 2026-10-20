"""
camara.py — PI-1: captura y marca de tiempo. Dos fuentes con la misma interfaz:

  FuenteCamara     cámara real (Picamera2, sólo en la Raspberry Pi)
  FuenteSintetica  punto láser simulado (PC o RPi sin cámara), con verdad conocida

Ambas entregan objetos Cuadro con la luminancia (Y8) como matriz 2-D uint8.
Marca de tiempo: INICIO de exposición, en ns de CLOCK_MONOTONIC
(Picamera2 calcula el inicio como SensorTimestamp - ExposureTime).
"""
import math
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Cuadro:
    img: np.ndarray                 # alto x ancho, uint8 (luminancia)
    t_exp_ns: int                   # inicio de exposición [ns, CLOCK_MONOTONIC]
    t_entrega_ns: int               # cuando la matriz quedó disponible en Python
    k: int                          # número de cuadro
    meta: dict = field(default_factory=dict)
    verdad: tuple = None            # (u, w) real — sólo fuente sintética


# ===========================================================================
class FuenteSintetica:
    """Punto láser gaussiano sobre fondo con ruido. Trayectoria suave + maniobra +
    oclusiones, muestreo con jitter. Sirve para probar todo el flujo sin hardware."""

    def __init__(self, ancho=640, alto=480, fps=40.0, duracion_s=10.0, semilla=0,
                 sigma_px=1.6, pico=235, fondo=35, ruido=4.0, jitter_ms=0.8,
                 latencia_ms=12.0, oclusiones=((3.0, 3.5), (6.0, 6.3)),
                 falsos=((4.5, (80, 60)),), tiempo_real=False):
        self.W, self.H, self.fps, self.dur = ancho, alto, fps, duracion_s
        self.rng = np.random.default_rng(semilla)
        self.sigma, self.pico, self.fondo, self.ruido = sigma_px, pico, fondo, ruido
        self.jitter, self.lat = jitter_ms, latencia_ms
        self.ocl, self.falsos, self.tiempo_real = oclusiones, falsos, tiempo_real
        self.vv, self.uu = np.mgrid[0:alto, 0:ancho].astype(np.float32)

    def trayectoria(self, t):
        u = self.W / 2 + 0.30 * self.W * math.sin(0.7 * t) + 0.04 * self.W * math.sin(3.1 * t)
        w = self.H / 2 + 0.25 * self.H * math.sin(0.45 * t + 0.6)
        return u, w

    def _render(self, u, w, visible, extra=None):
        img = self.fondo + self.rng.normal(0, self.ruido, (self.H, self.W)).astype(np.float32)
        for (pu, pw, amp) in ([(u, w, self.pico)] if visible else []) + (extra or []):
            m = int(4 * self.sigma) + 1
            x0, x1 = max(0, int(pu) - m), min(self.W, int(pu) + m + 1)
            y0, y1 = max(0, int(pw) - m), min(self.H, int(pw) + m + 1)
            if x0 < x1 and y0 < y1:
                d2 = (self.uu[y0:y1, x0:x1] - pu) ** 2 + (self.vv[y0:y1, x0:x1] - pw) ** 2
                img[y0:y1, x0:x1] += amp * np.exp(-d2 / (2 * self.sigma ** 2))
        return np.clip(img, 0, 255).astype(np.uint8)

    def __iter__(self):
        t0 = time.monotonic_ns()
        self.t0_ns = t0                      # para convertir tiempo absoluto -> tiempo simulado
        n = int(self.dur * self.fps)
        t = 0.0
        for k in range(n):
            t += 1.0 / self.fps + self.rng.normal(0, self.jitter * 1e-3)
            u, w = self.trayectoria(t)
            visible = not any(a <= t < b for a, b in self.ocl)
            extra = [(fu, fw, 250) for (tf, (fu, fw)) in self.falsos if abs(t - tf) < 0.5 / self.fps]
            img = self._render(u, w, visible, extra)
            t_exp = t0 + int(t * 1e9)
            if self.tiempo_real:
                espera = (t_exp + int(self.lat * 1e6) - time.monotonic_ns()) / 1e9
                if espera > 0:
                    time.sleep(espera)
                t_ent = time.monotonic_ns()
            else:
                t_ent = t_exp + int(self.lat * 1e6)
            yield Cuadro(img, t_exp, t_ent, k, {"visible": visible}, (u, w) if visible else None)

    def cerrar(self):
        pass


# ===========================================================================
class FuenteCamara:
    """Cámara real vía Picamera2. Entrega el plano Y de YUV420 (luminancia)."""

    def __init__(self, p_cap, duracion_s=None):
        from picamera2 import Picamera2        # import tardío: no existe en el PC
        self.pc = p_cap
        self.dur = duracion_s
        self.cam = Picamera2()
        fd = int(round(1e6 / p_cap.fps))
        controles = {"FrameDurationLimits": (fd, fd)}
        if p_cap.exposicion_us:
            controles.update({"AeEnable": False, "ExposureTime": int(p_cap.exposicion_us),
                              "AnalogueGain": float(p_cap.ganancia)})
        cfg = self.cam.create_video_configuration(
            main={"size": tuple(p_cap.salida_px), "format": "YUV420"},
            sensor={"output_size": tuple(p_cap.modo_sensor_px)},
            controls=controles, buffer_count=6, queue=False)
        self.cam.configure(cfg)
        real = self.cam.camera_configuration()
        self.W, self.H = real["main"]["size"]
        self.info = {"modo_sensor": real["sensor"], "salida": (self.W, self.H)}
        self.cam.start()
        time.sleep(1.0)                          # convergencia de AE/AWB
        # recorte real del sensor (para la geometría)
        md = self.cam.capture_metadata()
        self.info["ScalerCrop"] = md.get("ScalerCrop")

    def __iter__(self):
        t_fin = None if self.dur is None else time.monotonic() + self.dur
        k = 0
        while t_fin is None or time.monotonic() < t_fin:
            req = self.cam.capture_request()
            arr = req.make_array("main")
            t_ent = time.monotonic_ns()
            md = req.get_metadata()
            req.release()
            y = arr[:self.H, :self.W]           # plano Y
            t_exp = md["SensorTimestamp"] - 1000 * md.get("ExposureTime", 0)
            yield Cuadro(y, t_exp, t_ent, k, {"ExposureTime": md.get("ExposureTime"),
                                             "ScalerCrop": md.get("ScalerCrop"),
                                             "SensorTimestamp": md["SensorTimestamp"]})
            k += 1

    def cerrar(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass
