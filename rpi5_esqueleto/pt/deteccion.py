"""
deteccion.py — Procesamiento de imagen, detector de LÁSER (PI-2 a PI-6).

Sólo NumPy, para que corra igual en el PC y en la Raspberry Pi.

  PI-2  recorte a la ventana de interés (128x128 en seguimiento, cuadro completo si no)
  PI-3  candidato = píxel más brillante de la ventana, si supera umbral_abs
  PI-4  la ventana se centra en la detección anterior (asociación trivial: 1 candidato)
  PI-5  centroide subpíxel ponderado por intensidad en una ventana alrededor del pico
  PI-6  validación: área de la mancha y salto máximo respecto del cuadro anterior

Pendiente (fuera de la PoC): detectores de aeronave y de patrón desconocido,
corrección de distorsión aplicada al punto, múltiples candidatos.
"""
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Resultado:
    valido: bool
    u: float = float("nan")     # centroide subpíxel [px]
    w: float = float("nan")
    pico: int = 0               # nivel máximo en la ventana [0..255]
    area: int = 0               # píxeles de la mancha
    confianza: float = 0.0      # [0, 1]
    motivo: str = ""            # por qué se rechazó (diagnóstico PI-8)
    t_proc_ms: float = 0.0


class DetectorLaser:
    def __init__(self, p_det):
        self.p = p_det
        self.ultimo = None      # (u, w) de la última detección válida

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True):
        """img: matriz 2-D uint8 (luminancia, alto x ancho). Devuelve Resultado."""
        t0 = time.perf_counter()
        r = self._procesar(img, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    # ------------------------------------------------------------------
    def _procesar(self, img, usar_ventana):
        p = self.p
        H, W = img.shape

        # PI-2 / PI-4: ventana de interés
        x0, y0, x1, y1 = 0, 0, W, H
        if usar_ventana and self.ultimo is not None:
            m = p.ventana_seguimiento // 2
            uc, wc = int(round(self.ultimo[0])), int(round(self.ultimo[1]))
            x0, x1 = max(0, uc - m), min(W, uc + m)
            y0, y1 = max(0, wc - m), min(H, wc + m)
        roi = img[y0:y1, x0:x1]

        # PI-3: candidato = pico de brillo
        idx = int(np.argmax(roi))
        wp, up = divmod(idx, roi.shape[1])
        pico = int(roi[wp, up])
        if pico < p.umbral_abs:
            return Resultado(False, pico=pico, motivo="sin pico sobre umbral")

        # PI-5: centroide ponderado en una ventana alrededor del pico
        n = p.medio_lado_mancha
        a0, a1 = max(0, up - n), min(roi.shape[1], up + n + 1)
        b0, b1 = max(0, wp - n), min(roi.shape[0], wp + n + 1)
        parche = roi[b0:b1, a0:a1].astype(np.float32)
        umbral = p.frac_pico * pico
        peso = parche - umbral
        peso[peso < 0] = 0.0
        area = int(np.count_nonzero(peso))
        s = float(peso.sum())
        if s <= 0:
            return Resultado(False, pico=pico, motivo="mancha vacía")
        vv, uu = np.mgrid[b0:b1, a0:a1]
        u = float((peso * uu).sum() / s) + x0
        w = float((peso * vv).sum() / s) + y0

        # PI-6: validación
        if not (p.area_min <= area <= p.area_max):
            return Resultado(False, u, w, pico, area, motivo=f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = np.hypot(u - self.ultimo[0], w - self.ultimo[1])
            if salto > p.salto_max_px:
                return Resultado(False, u, w, pico, area, motivo=f"salto {salto:.0f} px")

        # confianza simple: contraste del pico respecto del umbral
        fondo = float(np.median(roi[::8, ::8]))
        conf = float(np.clip((pico - fondo) / 255.0 * 2.0, 0.0, 1.0))
        return Resultado(True, u, w, pico, area, conf)
