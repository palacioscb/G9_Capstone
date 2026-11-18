"""
recorte.py — zona de trabajo de la imagen (cuadrilátero) elegida desde el tablero.

Quita de la vista del DETECTOR todo lo que cae fuera del cuadrilátero (la pared, los objetos de alrededor,
lo que hay más allá de la ventana). Sólo cambia la imagen que recibe det.procesar(): la imagen del
tablero, el registro, la geometría, la pose y el control siguen usando el cuadro completo, y las
coordenadas (u, w) que entrega el detector siguen siendo las del cuadro completo.

Cómo se quita: lo de afuera se rellena con la mediana de lo de adentro, para que el borde del
cuadrilátero no parezca un contorno ni un cambio brusco de brillo (los detectores por forma buscan
contornos). No se recorta ni se reescala el cuadro: una geometría distinta rompería la calibración
(0,115 °/px) y todo lo que depende de ella.

Cómo volver al comportamiento de antes (el recorte es OPCIONAL y parte apagado):
  - desmarcar «Recorte activo» en el tablero (el detector vuelve a recibir el cuadro tal cual, sin copiarlo);
  - arrancar con --sin-recorte (ni siquiera aparecen los controles);
  - si algo falla dentro del recorte, se apaga solo y el detector sigue con el cuadro completo.
"""
import math
import time

import numpy as np

#* El cuadrilátero debe cubrir al menos esta fracción del cuadro (si no, es un error de selección).
AREA_MIN_FRAC = 0.01
#* Un píxel de cada tantos se usa para sacar la mediana de lo de adentro (basta y es barato).
PASO_MEDIANA = 4


def ordenar(pts):
    """Los 4 puntos en orden de giro alrededor del centro, para que el cuadrilátero nunca quede cruzado
    (da igual en qué orden se hizo clic)."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def validar(pts, ancho, alto):
    """Devuelve los 4 puntos ordenados (en sentido horario de pantalla) o lanza ValueError."""
    if len(pts) != 4:
        raise ValueError("se necesitan 4 puntos")
    p = ordenar([(float(x), float(y)) for x, y in pts])
    cruces = []
    for i in range(4):
        (ax, ay), (bx, by), (cx, cy) = p[i], p[(i + 1) % 4], p[(i + 2) % 4]
        cruces.append((bx - ax) * (cy - by) - (by - ay) * (cx - bx))
    if not (all(c > 0 for c in cruces) or all(c < 0 for c in cruces)):
        raise ValueError("el cuadrilátero debe ser convexo y sin puntos repetidos ni alineados")
    area = 0.5 * abs(sum(p[i][0] * p[(i + 1) % 4][1] - p[(i + 1) % 4][0] * p[i][1] for i in range(4)))
    if area < AREA_MIN_FRAC * ancho * alto:
        raise ValueError(f"el recorte es demasiado chico ({area:.0f} px²; mínimo {AREA_MIN_FRAC * ancho * alto:.0f})")
    return p


def mascara(pts, forma, escala=1):
    """Booleano (alto, ancho): True dentro del cuadrilátero. `escala` = píxeles del cuadro completo por
    píxel de esta imagen (2 para el plano V, que tiene la mitad de resolución)."""
    H, W = forma
    yy, xx = np.mgrid[0:H, 0:W]
    x, y = (xx + 0.5) * escala, (yy + 0.5) * escala          # centro de cada píxel, en coordenadas del cuadro
    #* convexo: dentro = el mismo signo respecto de las 4 aristas
    cs = [((pts[(i + 1) % 4][0] - pts[i][0]) * (y - pts[i][1]) - (pts[(i + 1) % 4][1] - pts[i][1]) * (x - pts[i][0]))
          for i in range(4)]
    return np.logical_and.reduce([c >= 0 for c in cs]) | np.logical_and.reduce([c <= 0 for c in cs])


class RecorteROI:
    #* Cuadrilátero inicial (margen del 10 %): sirve si se activa sin dibujar uno.
    def __init__(self, ancho, alto):
        self.ancho, self.alto = int(ancho), int(alto)
        mx, my = int(0.1 * ancho), int(0.1 * alto)
        self.coords = {"roi.x1": mx, "roi.y1": my, "roi.x2": self.ancho - 1 - mx, "roi.y2": my,
                       "roi.x3": self.ancho - 1 - mx, "roi.y3": self.alto - 1 - my,
                       "roi.x4": mx, "roi.y4": self.alto - 1 - my}
        self.activo = False
        self.pts = None
        self.error = ""
        self.t_ms = 0.0
        self._m = {}                                           # escala -> (fuera, dentro_submuestreado)

    @staticmethod
    def claves():
        return ["roi.activo"] + [f"roi.{e}{i}" for i in range(1, 5) for e in "xy"]

    def puntos_pedidos(self):
        return [(self.coords[f"roi.x{i}"], self.coords[f"roi.y{i}"]) for i in range(1, 5)]

    def fijar(self, clave, valor):
        """Aplica un ajuste del tablero. Las coordenadas sólo se guardan; el recorte se (re)arma al
        recibir «roi.activo» (el tablero lo manda siempre al final, junto con los 8 números)."""
        if clave == "roi.activo":
            if valor:
                self.pts = validar(self.puntos_pedidos(), self.ancho, self.alto)     # ValueError: no se activa
                self._m = {}
                self.error = ""
                self.activo = True
            else:
                self.activo = False
        elif clave in self.coords:
            self.coords[clave] = int(valor)
        else:
            raise KeyError(clave)

    def _plano(self, plano, escala):
        m = self._m.get(escala)
        if m is None or m[2] != plano.shape:
            dentro = mascara(self.pts, plano.shape, escala)
            sub = dentro[::PASO_MEDIANA, ::PASO_MEDIANA]
            m = (~dentro, sub, plano.shape)
            self._m[escala] = m
        fuera, sub, _ = m
        relleno = np.uint8(np.median(plano[::PASO_MEDIANA, ::PASO_MEDIANA][sub])) if sub.any() else np.uint8(128)
        salida = plano.copy()                                  # el cuadro original NO se toca
        np.copyto(salida, relleno, where=fuera)
        return salida

    def aplicar(self, y, v=None):
        """(y, v) con lo de afuera del cuadrilátero rellenado. Si falla, se apaga y devuelve lo mismo que recibió."""
        t0 = time.perf_counter()
        try:
            y2 = self._plano(y, 1)
            v2 = self._plano(v, 2) if v is not None else None
        except Exception as ex:                                # nunca debe botar el lazo
            self.activo, self.error = False, repr(ex)
            return y, v
        self.t_ms = (time.perf_counter() - t0) * 1e3
        return y2, v2
