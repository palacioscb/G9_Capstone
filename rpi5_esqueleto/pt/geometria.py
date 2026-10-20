"""
geometria.py — CS-1: conversión píxel -> ángulo (DET-4, base de ADV-1).

Modelo pinhole. Convención de imagen: u crece a la derecha, w crece hacia abajo,
origen en la esquina superior izquierda (igual que matriz[w, u] en NumPy).
Convención angular: pan > 0 hacia la derecha, tilt > 0 hacia arriba.

Por qué NO hace falta la distancia al objetivo: la cámara gira sobre sus ejes,
así que el ángulo para centrar un punto depende sólo de su dirección en el
sensor, (u - cx)/f_px, no de lo lejos que esté. La distancia sólo entra por
paralaje si el centro óptico no coincide con el cruce de los ejes pan-tilt
(error ≈ d/Z; con d = 3 cm y Z = 2 m es < 1°) — se acota en el informe.

Descomposición exacta pan-tilt (cámara con tilt ≈ 0):
    x = (u - cx)/fx,  y = (w - cy)/fy
    pan  = atan(x)
    tilt = -atan(y / sqrt(1 + x²))
Con la cámara ya inclinada, el eje de pan (vertical del mundo) deja de coincidir
con el eje y de la cámara; el caso general usa la composición de rotaciones
Rz(θa)·Ry(θe) (Guo et al. 2021, ecs. 1 y 16) con los ángulos medidos por el AS5600.
"""
import math
from dataclasses import dataclass


@dataclass
class Geometria:
    fx: float          # focal en píxeles de salida, eje horizontal
    fy: float          # focal en píxeles de salida, eje vertical
    cx: float          # centro óptico [px]
    cy: float
    ancho: int
    alto: int

    # ------------------------------------------------------------------
    @classmethod
    def desde_optica(cls, f_mm, pix_um, recorte_sensor, salida_px, centro=None):
        """recorte_sensor = (x, y, ancho, alto) en píxeles NATIVOS (ScalerCrop de libcamera).
        salida_px = (ancho, alto) de la matriz entregada."""
        _, _, cw, ch = recorte_sensor
        W, H = salida_px
        pix_mm = pix_um / 1000.0
        fx = (f_mm / pix_mm) * (W / cw)
        fy = (f_mm / pix_mm) * (H / ch)
        cx, cy = centro if centro else ((W - 1) / 2.0, (H - 1) / 2.0)
        return cls(fx, fy, cx, cy, W, H)

    @classmethod
    def desde_parametros(cls, p):
        return cls.desde_optica(p.optica.f_mm, p.optica.pix_um,
                                p.captura.recorte_sensor, p.captura.salida_px)

    # ------------------------------------------------------------------
    def pix_a_ang(self, u, w):
        """Ángulos (pan, tilt) en grados para llevar el punto (u, w) al centro."""
        x = (u - self.cx) / self.fx
        y = (w - self.cy) / self.fy
        pan = math.degrees(math.atan(x))
        tilt = -math.degrees(math.atan(y / math.sqrt(1.0 + x * x)))
        return pan, tilt

    def pix_a_ang_lineal(self, u, w):
        """Aproximación lineal Δθ = e_px · IFOV (la del diagrama CS-1). Válida cerca del centro."""
        return ((u - self.cx) * self.ifov_x_deg, -(w - self.cy) * self.ifov_y_deg)

    def ang_a_pix(self, pan, tilt):
        """Inversa de pix_a_ang (útil para simulación y calibración)."""
        x = math.tan(math.radians(pan))
        y = -math.tan(math.radians(tilt)) * math.sqrt(1.0 + x * x)
        return self.cx + x * self.fx, self.cy + y * self.fy

    # ------------------------------------------------------------------
    @property
    def ifov_x_deg(self):
        return math.degrees(math.atan(1.0 / self.fx))

    @property
    def ifov_y_deg(self):
        return math.degrees(math.atan(1.0 / self.fy))

    @property
    def hfov_deg(self):
        return 2 * math.degrees(math.atan((self.ancho / 2.0) / self.fx))

    @property
    def vfov_deg(self):
        return 2 * math.degrees(math.atan((self.alto / 2.0) / self.fy))

    def resumen(self):
        return (f"{self.ancho}x{self.alto} px | fx={self.fx:.1f} fy={self.fy:.1f} px | "
                f"FOV {self.hfov_deg:.1f}° x {self.vfov_deg:.1f}° | "
                f"IFOV centro {self.ifov_x_deg:.4f} °/px")
