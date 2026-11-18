"""
formas.py — Descripción de FORMAS con descriptores de Fourier del contorno (PI-3 / PI-6).

Sirve al detector por forma de deteccion.py (DetectorForma): el objetivo se reconoce por su
silueta, no por su brillo ni por su color. Sólo NumPy (numpy.fft).

Idea (Zahn y Roskies 1972; Persoon y Fu 1977; Kuhl y Giardina 1982):
  1. el contorno cerrado se recorre a pasos iguales de longitud y cada punto se escribe como
     un número complejo  z[n] = x[n] + j·y[n],  n = 0..N−1;
  2. se calcula su transformada de Fourier  F[k] = FFT(z)/N;
  3. se normaliza para que el resultado no dependa de dónde está, de su tamaño, de cómo
     está girada en la imagen ni del punto donde empieza el contorno:
        F[0]            se descarta        -> traslación   (F[0] es el centroide del contorno)
        se divide por |F[1]|               -> escala       (F[1] es el círculo que mejor ajusta)
        se usa sólo el módulo |F[k]|       -> giro en la imagen y punto de partida
  4. el descriptor es el vector  d = ( |F[−K]|..|F[−1]|, |F[2]|..|F[K]| ) / |F[1]|.

Lectura de los armónicos:
  círculo           todo en F[1]: d = 0
  elipse            sólo F[1] y F[−1];  |F[−1]|/|F[1]| = (a − b)/(a + b)   (a, b = semiejes)
  figuras con puntas (avión)   armónicos altos: cada uno describe un detalle de la silueta

Dos formas se comparan con la distancia euclidiana entre descriptores (0 = iguales).

NORMALIZACIÓN AFÍN (paso previo). Un objeto plano que se inclina respecto de la cámara se ve
aplastado en una dirección: un disco se ve como elipse, un avión visto desde arriba se ve más
corto o más angosto. Antes de calcular el descriptor, el contorno se "desaplasta": se lleva
a que sus segundos momentos de área sean iguales en todas las direcciones (covarianza =
identidad). Dos vistas inclinadas del mismo objeto plano quedan así iguales salvo un giro,
que el módulo de Fourier ya ignora. Con esto toda elipse queda convertida en círculo.
"""
#* Lo que NO se compensa: el cambio de CARA. Un avión visto de lado y visto desde arriba son
#* siluetas distintas, así que el banco guarda una plantilla por vista (arriba, lado...).
#* El alargamiento que se quita al normalizar no se pierde: se entrega aparte (alargamiento),
#* para poder descartar figuras demasiado delgadas (un palo, un borde).
#! No hay siluetas de avión incorporadas (retiradas el 18-11-2026: eran dibujos genéricos que
#! no calzaban con el avión real). Las vistas del avión se APRENDEN del avión real con
#! herramientas/aprender_forma.py y quedan en formas.json. La única incorporada es el círculo.
import json
import math
import os

import numpy as np

N_PUNTOS = 64          # puntos del contorno remuestreado (potencia de 2: FFT rápida)
K_ARMONICOS = 8        # armónicos por lado: el descriptor tiene 2K − 1 = 15 números


def remuestrear(contorno, n=N_PUNTOS):
    """Contorno cerrado (m x 2) -> n puntos a pasos iguales de longitud de arco, como complejos."""
    c = np.asarray(contorno, dtype=np.float64).reshape(-1, 2)
    cierre = np.vstack([c, c[:1]])
    paso = np.hypot(np.diff(cierre[:, 0]), np.diff(cierre[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(paso)])
    if s[-1] <= 0:
        return None
    t = np.arange(n) * (s[-1] / n)
    return np.interp(t, s, cierre[:, 0]) + 1j * np.interp(t, s, cierre[:, 1])


def momentos(contorno):
    """Área, centroide (cx, cy) y covarianza 2x2 de la región encerrada por el polígono."""
    c = np.asarray(contorno, dtype=np.float64).reshape(-1, 2)
    x, y = c[:, 0] - c[0, 0], c[:, 1] - c[0, 1]          # origen cerca: mejor condicionado
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    cr = x * yn - xn * y                                  # fórmulas de Green para un polígono
    A = 0.5 * cr.sum()
    if abs(A) < 1e-9:
        return None
    cx, cy = ((x + xn) * cr).sum() / (6 * A), ((y + yn) * cr).sum() / (6 * A)
    ixx = ((x * x + x * xn + xn * xn) * cr).sum() / (12 * A) - cx * cx
    iyy = ((y * y + y * yn + yn * yn) * cr).sum() / (12 * A) - cy * cy
    ixy = ((x * yn + 2 * x * y + 2 * xn * yn + xn * y) * cr).sum() / (24 * A) - cx * cy
    return abs(A), (cx + c[0, 0], cy + c[0, 1]), np.array([[ixx, ixy], [ixy, iyy]])


def normalizar_afin(contorno):
    """(contorno desaplastado, alargamiento). alargamiento = eje mayor / eje menor de la
    elipse equivalente (1 = tan ancho como largo). None si el contorno es degenerado."""
    m = momentos(contorno)
    if m is None:
        return None
    _, (cx, cy), C = m
    val, vec = np.linalg.eigh(C)
    if val[0] <= 1e-9:
        return None
    c = (np.asarray(contorno, dtype=np.float64).reshape(-1, 2) - (cx, cy)) @ vec / np.sqrt(val)
    return c, float(math.sqrt(val[1] / val[0]))


def descriptor(contorno, n=N_PUNTOS, k=K_ARMONICOS, afin=True):
    """Descriptor de Fourier invariante a traslación, escala, giro en la imagen y punto de
    partida; con afin=True, también a la inclinación de un objeto plano.
    Devuelve un vector de 2k − 1 números, o None si el contorno es degenerado."""
    if afin:
        r = normalizar_afin(contorno)
        if r is None:
            return None
        contorno = r[0]
    z = remuestrear(contorno, n)
    if z is None:
        return None
    F = np.fft.fft(z) / n
    #* Sentido de recorrido: según sea horario o antihorario, el círculo principal queda en
    #* F[1] o en F[−1]. Se lleva siempre a F[1] (conjugar z equivale a invertir el sentido).
    if abs(F[-1]) > abs(F[1]):
        F = np.fft.fft(np.conj(z)) / n
    a1 = abs(F[1])
    if a1 < 1e-9:
        return None
    return np.abs(np.concatenate([F[-k:], F[2:k + 1]])) / a1


def distancia(d, plantilla):
    """Distancia euclidiana entre descriptores (0 = la misma forma)."""
    dif = np.asarray(d) - np.asarray(plantilla)
    return float(np.sqrt(dif @ dif))


def alargamiento(contorno):
    """Eje mayor / eje menor de la elipse equivalente del contorno (1 = redondo o cuadrado)."""
    r = normalizar_afin(contorno)
    return float("inf") if r is None else r[1]


# ---------------------------------------------------------------------------
#* Siluetas incorporadas (polígonos en unidades arbitrarias; sólo importa la forma)
# ---------------------------------------------------------------------------
def poligono(nombre):
    """Polígono de una silueta incorporada. Sólo hay una: 'circulo'."""
    if nombre == "circulo":
        a = np.linspace(0, 2 * np.pi, 96, endpoint=False)
        return np.c_[np.cos(a), np.sin(a)]
    raise KeyError(nombre)


INCORPORADAS = ("circulo",)


class BancoDeFormas:
    """Plantillas por nombre: las incorporadas más las aprendidas (formas.json)."""

    def __init__(self, ruta=None):
        self.ruta = ruta
        self.plantillas = {n: descriptor(poligono(n)) for n in INCORPORADAS}
        self.aprendidas = set()
        if ruta and os.path.exists(ruta):
            with open(ruta, encoding="utf-8") as f:
                for nombre, d in json.load(f).items():
                    if len(d) == 2 * K_ARMONICOS - 1:
                        self.plantillas[nombre] = np.array(d, dtype=np.float64)
                        self.aprendidas.add(nombre)

    def de_la_familia(self, familias):
        """Plantillas cuyo nombre es una de `familias` o empieza por ella y '_'
        ('avion' -> las aprendidas avion_arriba, avion_lado...). Vacío si no hay ninguna."""
        fam = (familias,) if isinstance(familias, str) else tuple(familias)
        #* Si hay vistas aprendidas de la familia, mandan ellas sobre la incorporada (círculo).
        nombres = [n for n in self.plantillas if any(n == f or n.startswith(f + "_") for f in fam)]
        aprendidas = [n for n in nombres if n in self.aprendidas]
        return {n: self.plantillas[n] for n in (aprendidas or nombres)}

    def guardar(self, nombre, d):
        """Agrega o reemplaza una plantilla aprendida y la escribe en formas.json."""
        if not (nombre.replace("_", "").isalnum() and len(nombre) <= 32):
            raise ValueError("nombre: sólo letras, números y '_' (máx. 32)")
        self.plantillas[nombre] = np.asarray(d, dtype=np.float64)
        self.aprendidas.add(nombre)
        if self.ruta:
            datos = {n: [round(float(x), 5) for x in self.plantillas[n]] for n in sorted(self.aprendidas)}
            tmp = self.ruta + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(datos, f, indent=1)
            os.replace(tmp, self.ruta)

    def mejor(self, d, familias):
        """(nombre, distancia) de la plantilla más parecida dentro de las familias pedidas."""
        mejor = (None, math.inf)
        for nombre, p in self.de_la_familia(familias).items():
            dist = distancia(d, p)
            if dist < mejor[1]:
                mejor = (nombre, dist)
        return mejor


def transformar(pol, tam, giro=0.0, centro=(0.0, 0.0), aplastar=(1.0, 1.0)):
    """Polígono de una silueta -> píxeles: `tam` es su largo mayor [px], `giro` en radianes,
    `aplastar` = (fx, fy) simula la inclinación fuera del plano. Para escenas sintéticas."""
    p = np.asarray(pol, dtype=np.float64) * np.asarray(aplastar, dtype=np.float64)
    c, s = math.cos(giro), math.sin(giro)
    p = p @ np.array([[c, s], [-s, c]])
    return p * (tam / 2.0) + np.asarray(centro, dtype=np.float64)
