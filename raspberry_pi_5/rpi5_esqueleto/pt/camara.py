"""
camara.py — PI-1: captura y marca de tiempo. Dos fuentes con la misma interfaz:

  FuenteCamara     cámara real (Picamera2, sólo en la Raspberry Pi)
  FuenteSintetica  punto láser simulado (PC o RPi sin cámara), con verdad conocida

Ambas entregan objetos Cuadro con la luminancia (Y8) como matriz 2-D uint8 y, además,
el plano V (crominancia roja, a media resolución) para el detector de láser rojo.
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
    v: np.ndarray = None            # plano V (alto/2 x ancho/2, uint8, 128 = neutro)
    exp_ns: int = 0                 # tiempo de exposición [ns]
    lectura_ns: int = 0             # tiempo de lectura de todas las filas (persiana rodante) [ns]
    alto: int = 0

    def instante(self, w=None):
        """Instante que representa la fila w: centro de SU exposición [ns, CLOCK_MONOTONIC].
        El IMX219 lee fila por fila (persiana rodante): la fila w empieza a exponerse
        (w/alto)·t_lectura después de la primera. Sin w, se usa la fila central."""
        if not self.lectura_ns or not self.alto:
            return self.t_exp_ns + self.exp_ns // 2
        frac = 0.5 if w is None or w != w else min(max(w / self.alto, 0.0), 1.0)
        return self.t_exp_ns + self.exp_ns // 2 + int(frac * self.lectura_ns)


# ===========================================================================
class FuenteSintetica:
    """Punto láser gaussiano sobre fondo con ruido. Trayectoria suave + maniobra +
    oclusiones, muestreo con jitter. Sirve para probar todo el flujo sin hardware."""

    def __init__(self, ancho=640, alto=480, fps=40.0, duracion_s=10.0, semilla=0,
                 sigma_px=1.6, pico=235, fondo=35, ruido=4.0, jitter_ms=0.8,
                 latencia_ms=12.0, oclusiones=((3.0, 3.5), (6.0, 6.3)),
                 falsos=((4.5, (80, 60)),), tiempo_real=False, pico_v=100, lampara=None,
                 geometria=None, pose_en=None):
        self.W, self.H, self.fps, self.dur = ancho, alto, fps, duracion_s
        self.rng = np.random.default_rng(semilla)
        self.sigma, self.pico, self.fondo, self.ruido = sigma_px, pico, fondo, ruido
        self.jitter, self.lat = jitter_ms, latencia_ms
        self.ocl, self.falsos, self.tiempo_real = oclusiones, falsos, tiempo_real
        self.pico_v = pico_v                 # amplitud del láser en el plano V (sobre 128)
        #* lampara = (u, w): una fuente BLANCA fija (brillo alto, V neutro). El detector por
        #* brillo se engancha en ella; el detector rojo debe ignorarla.
        self.lampara = lampara
        #* Lazo cerrado simulado: con geometria y pose_en(t_ns) -> (pan, tilt), la trayectoria
        #* se interpreta como dirección en el MUNDO (la que tendría con la cámara en 0°, 0°) y el
        #* punto se dibuja donde lo vería la cámara con la pose de la plataforma en ese instante.
        self.geo, self.pose_en = geometria, pose_en
        self.vv, self.uu = np.mgrid[0:alto, 0:ancho].astype(np.float32)
        self.vv2, self.uu2 = np.mgrid[0:alto // 2, 0:ancho // 2].astype(np.float32)

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

    def _render_v(self, u, w, visible):
        """Plano V a media resolución: la muestra j cubre los píxeles 2j y 2j+1 (centro 2j+0,5)."""
        v = 128 + self.rng.normal(0, 1.5, (self.H // 2, self.W // 2)).astype(np.float32)
        if visible:
            pu, pw, s = (u - 0.5) / 2, (w - 0.5) / 2, max(self.sigma / 2, 0.7)
            m = int(4 * s) + 1
            x0, x1 = max(0, int(pu) - m), min(self.W // 2, int(pu) + m + 1)
            y0, y1 = max(0, int(pw) - m), min(self.H // 2, int(pw) + m + 1)
            if x0 < x1 and y0 < y1:
                d2 = (self.uu2[y0:y1, x0:x1] - pu) ** 2 + (self.vv2[y0:y1, x0:x1] - pw) ** 2
                v[y0:y1, x0:x1] += self.pico_v * np.exp(-d2 / (2 * s ** 2))
        return np.clip(v, 0, 255).astype(np.uint8)

    def __iter__(self):
        t0 = time.monotonic_ns()
        self.t0_ns = t0                      # para convertir tiempo absoluto -> tiempo simulado
        n = int(self.dur * self.fps)
        t = 0.0
        for k in range(n):
            t += 1.0 / self.fps + self.rng.normal(0, self.jitter * 1e-3)
            u, w = self.trayectoria(t)
            visible = not any(a <= t < b for a, b in self.ocl)
            if self.geo is not None and self.pose_en is not None:
                pan_obj, tilt_obj = self.geo.direccion_mundo(u, w, 0.0, 0.0)
                pose = self.pose_en(t0 + int(t * 1e9))
                u, w = self.geo.mundo_a_pix(pan_obj, tilt_obj, pose[0], pose[1])
                visible = visible and (0 <= u < self.W) and (0 <= w < self.H)
            extra = [(fu, fw, 250) for (tf, (fu, fw)) in self.falsos if abs(t - tf) < 0.5 / self.fps]
            if self.lampara:
                extra.append((self.lampara[0], self.lampara[1], 255))
            img = self._render(u, w, visible, extra)
            v = self._render_v(u, w, visible)
            t_exp = t0 + int(t * 1e9)
            if self.tiempo_real:
                espera = (t_exp + int(self.lat * 1e6) - time.monotonic_ns()) / 1e9
                if espera > 0:
                    time.sleep(espera)
                t_ent = time.monotonic_ns()
            else:
                t_ent = t_exp + int(self.lat * 1e6)
            yield Cuadro(img, t_exp, t_ent, k, {"visible": visible}, (u, w) if visible else None, v)

    def cerrar(self):
        pass


# ===========================================================================
class FuenteCamara:
    """Cámara real vía Picamera2. Entrega los planos Y (luminancia) y V (rojo) de YUV420."""

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
        #* Balance de blancos fijo (de laser_op.py): el umbral del plano V no debe moverse
        #* con la escena.
        if p_cap.ganancias_color:
            controles.update({"AwbEnable": False, "ColourGains": tuple(p_cap.ganancias_color)})
        if not p_cap.reduccion_ruido:
            try:
                from libcamera import controls as lc
                controles["NoiseReductionMode"] = lc.draft.NoiseReductionModeEnum.Off
            except Exception:                    # versiones de libcamera sin el control 'draft'
                pass
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
            #* YUV420 planar (I420): H filas de Y, luego H/4 filas de U y H/4 de V, cada una
            #* con dos filas de croma de ancho stride/2. Mismo corte que laser_op.py.
            H, W = self.H, self.W
            stride = arr.shape[1]
            y = arr[:H, :W]                                                       # plano Y
            v = arr[H + H // 4: H + H // 2].reshape(H // 2, stride // 2)[:, :W // 2]  # plano V
            t_exp = md["SensorTimestamp"] - 1000 * md.get("ExposureTime", 0)
            yield Cuadro(y, t_exp, t_ent, k, {"ExposureTime": md.get("ExposureTime"),
                                             "ScalerCrop": md.get("ScalerCrop"),
                                             "SensorTimestamp": md["SensorTimestamp"]}, None, v,
                         exp_ns=1000 * int(md.get("ExposureTime", 0)),
                         lectura_ns=int(self.pc.t_lectura_ms * 1e6), alto=H)
            k += 1

    def cerrar(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass
