"""
deteccion.py — Procesamiento de imagen, detectores de LÁSER (PI-2 a PI-6).

Sólo NumPy, para que corra igual en el PC y en la Raspberry Pi. Dos detectores con la
misma interfaz `procesar(img, usar_ventana, v=None) -> Resultado`:

  DetectorLaser      por BRILLO, sobre el plano Y (el original del esqueleto)
  DetectorLaserRojo  por COLOR, sobre el plano V, con confirmación de brillo en Y
                     (algoritmo de mi_proyecto_camara/laser_op.py)

  PI-2  recorte a la ventana de interés (seguimiento) o cuadro completo
  PI-3  candidato: pico de brillo (Y) o zona más roja por bloques (V)
  PI-4  la ventana se centra en la detección anterior (asociación trivial: 1 candidato)
  PI-5  centroide subpíxel ponderado alrededor del candidato
  PI-6  validación: área de la mancha y salto máximo respecto del cuadro anterior

Pendiente (fuera de la PoC): detectores de aeronave y de patrón desconocido,
corrección de distorsión aplicada al punto, múltiples candidatos.
"""
#* Por qué el detector rojo es el de trabajo: un rojo puro tiene Y ≈ 0,30·R ≈ 76, bajo el
#* umbral de 180 del detector por brillo (salvo que el centro del punto sature), y
#* cualquier lámpara o reflejo blanco le gana al láser en brillo. En el plano V, una
#* fuente blanca queda en ≈ 128 (neutro) y no dispara.
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

    def procesar(self, img, usar_ventana=True, v=None):
        """img: matriz 2-D uint8 (luminancia, alto x ancho). v se ignora. Devuelve Resultado."""
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


# ===========================================================================
class DetectorLaserRojo:
    """Detector de láser ROJO por crominancia (plano V del YUV420), de laser_op.py.
    """
    #* Trabaja en el plano V (media resolución: 4 veces menos píxeles) y devuelve el centroide
    #* en píxeles del plano Y. La muestra V (i, j) cubre los píxeles 2j y 2j+1, cuyo centro es
    #* 2j + 0,5 (con la convención de que el píxel entero n tiene su centro en n).
    #* Pasos: búsqueda por bloques (inmune a un píxel caliente) -> centroide ponderado
    #* alrededor del mejor bloque -> confirmación de brillo en Y (descarta ropa o carteles
    #* rojos oscuros) -> validación de área y salto (PI-6, igual que el detector por brillo).

    V_NEUTRO = 128

    def __init__(self, p_roja):
        self.p = p_roja
        self.ultimo = None      # (u, w) de la última detección válida, en px del plano Y

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True, v=None):
        """img: plano Y (alto x ancho). v: plano V (alto/2 x ancho/2). Devuelve Resultado."""
        t0 = time.perf_counter()
        if v is None:
            r = Resultado(False, motivo="sin plano V (fuente sin color)")
        else:
            r = self._procesar(img, v, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    # ------------------------------------------------------------------
    def _procesar(self, y, v, usar_ventana):
        p = self.p
        h, w = v.shape

        # PI-2 / PI-4: ventana de interés, en coordenadas del plano V
        x0, y0, x1, y1 = 0, 0, w, h
        if usar_ventana and self.ultimo is not None:
            m = max(p.ventana_seguimiento // 4, p.bloque)       # medio lado en px de V
            uc = int(round((self.ultimo[0] - 0.5) / 2))
            wc = int(round((self.ultimo[1] - 0.5) / 2))
            x0, x1 = max(0, uc - m), min(w, uc + m)
            y0, y1 = max(0, wc - m), min(h, wc + m)
        roi = v[y0:y1, x0:x1]
        if roi.size == 0:
            return Resultado(False, motivo="ventana vacía")
        pico_v = int(roi.max())
        if pico_v < p.v_umbral:                       # salida rápida: nada rojo
            return Resultado(False, pico=pico_v, motivo="sin rojo sobre umbral")

        # PI-3: bloque b x b más rojo (suma por bloques)
        b = p.bloque
        hb, wb = roi.shape[0] // b, roi.shape[1] // b
        if hb == 0 or wb == 0:
            idx = int(np.argmax(roi))
            cy, cx = divmod(idx, roi.shape[1])
        else:
            suma = roi[:hb * b, :wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.uint32)
            by, bx = divmod(int(suma.argmax()), wb)
            cx, cy = bx * b + b // 2, by * b + b // 2

        # PI-5: centroide ponderado en un radio 1,5·b alrededor del bloque
        rad = (3 * b) // 2
        a0, a1 = max(0, cx - rad), min(roi.shape[1], cx + rad)
        c0, c1 = max(0, cy - rad), min(roi.shape[0], cy + rad)
        ven = roi[c0:c1, a0:a1]
        pico = int(ven.max())
        if pico < p.v_umbral:
            return Resultado(False, pico=pico, motivo="bloque rojo sin pico")
        corte = self.V_NEUTRO + p.frac_corte * (pico - self.V_NEUTRO)
        peso = ven.astype(np.float32) - corte
        peso[peso < 0] = 0.0
        area_v = int(np.count_nonzero(peso))
        s = float(peso.sum())
        if s <= 0:
            return Resultado(False, pico=pico, motivo="mancha vacía")
        vv, uu = np.mgrid[c0:c1, a0:a1]
        mx = float((peso * uu).sum() / s) + x0
        my = float((peso * vv).sum() / s) + y0
        u_real, w_real = 2.0 * mx + 0.5, 2.0 * my + 0.5
        area = 4 * area_v                               # en px del plano Y

        # Confirmación de brillo en Y (ventana 7x7)
        if p.y_min > 0:
            ui, wi = int(u_real), int(w_real)
            brillo = int(y[max(wi - 3, 0): wi + 4, max(ui - 3, 0): ui + 4].max())
            if brillo < p.y_min:
                return Resultado(False, u_real, w_real, pico, area, motivo=f"rojo sin brillo (Y={brillo})")

        # PI-6: validación
        if not (p.area_min <= area <= p.area_max):
            return Resultado(False, u_real, w_real, pico, area, motivo=f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = np.hypot(u_real - self.ultimo[0], w_real - self.ultimo[1])
            if salto > p.salto_max_px:
                return Resultado(False, u_real, w_real, pico, area, motivo=f"salto {salto:.0f} px")

        conf = float(np.clip((pico - self.V_NEUTRO) / 127.0, 0.0, 1.0))
        return Resultado(True, u_real, w_real, pico, area, conf)
