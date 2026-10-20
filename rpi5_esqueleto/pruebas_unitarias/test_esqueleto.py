"""
Pruebas unitarias del esqueleto. Corren en el PC y en la RPi, sin hardware.

    python3 -m pytest pruebas_unitarias -q      (si tienes pytest)
    python3 pruebas_unitarias/test_esqueleto.py (sin pytest)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pt.parametros import Parametros                              # noqa: E402
from pt.geometria import Geometria                                # noqa: E402
from pt.deteccion import DetectorLaser                            # noqa: E402
from pt.supervisor import Supervisor, Estado                      # noqa: E402
from pt.enlace import crc16_ccitt, armar_trama, leer_trama, trama_consigna  # noqa: E402
from pt.control import omega_a_pasos                              # noqa: E402
from pt.camara import FuenteSintetica                             # noqa: E402

P = Parametros()


def test_geometria_nominal():
    g = Geometria.desde_parametros(P)
    assert abs(g.hfov_deg - 65.6) < 0.2          # 1640x1232 -> campo completo
    assert abs(g.ifov_x_deg - 0.1154) < 0.001    # a 640 px
    assert g.pix_a_ang(g.cx, g.cy) == (0.0, 0.0)


def test_geometria_modo_recortado():
    g = Geometria.desde_optica(2.85, 1.12, (1000, 752, 1280, 960), (640, 480))
    assert abs(g.hfov_deg - 28.2) < 0.2          # el modo 640x480 del sensor recorta


def test_geometria_ida_y_vuelta():
    g = Geometria.desde_parametros(P)
    for u, w in [(0, 0), (639, 479), (100, 400), (320.3, 12.7)]:
        pan, tilt = g.pix_a_ang(u, w)
        u2, w2 = g.ang_a_pix(pan, tilt)
        assert abs(u - u2) < 1e-6 and abs(w - w2) < 1e-6


def test_signos():
    g = Geometria.desde_parametros(P)
    pan, tilt = g.pix_a_ang(g.cx + 50, g.cy - 50)   # derecha y arriba
    assert pan > 0 and tilt > 0


def test_lineal_vs_exacto_en_el_borde():
    g = Geometria.desde_parametros(P)
    exacto, _ = g.pix_a_ang(639, g.cy)
    lineal, _ = g.pix_a_ang_lineal(639, g.cy)
    assert abs(exacto - 32.8) < 0.2
    assert lineal - exacto > 3.0                     # la aproximación lineal sobreestima > 3° en el borde


def _imagen_con_punto(u, w, W=640, H=480, sigma=1.6, pico=235):
    vv, uu = np.mgrid[0:H, 0:W]
    img = 30 + pico * np.exp(-((uu - u) ** 2 + (vv - w) ** 2) / (2 * sigma ** 2))
    return np.clip(img, 0, 255).astype(np.uint8)


def test_centroide_subpixel():
    d = DetectorLaser(P.deteccion)
    for u, w in [(100.3, 200.7), (320.5, 240.25), (600.9, 20.1)]:
        d.reiniciar()
        r = d.procesar(_imagen_con_punto(u, w), usar_ventana=False)
        assert r.valido, r.motivo
        assert math.hypot(r.u - u, r.w - w) < 0.1


def test_sin_objetivo():
    d = DetectorLaser(P.deteccion)
    r = d.procesar(np.full((480, 640), 40, np.uint8))
    assert not r.valido


def test_crc_vector_conocido():
    assert crc16_ccitt(b"123456789") == 0x29B1   # CRC-16/CCITT-FALSE


def test_trama_ida_y_vuelta_y_corrupta():
    t = trama_consigna(7, 12.5, -3.25)
    seq, tipo, carga, resto = leer_trama(b"\x00basura" + t + b"\xAA")
    assert seq == 7 and tipo == 1 and len(carga) == 8 and resto == b"\xAA"
    mala = bytearray(t)
    mala[6] ^= 0xFF
    seq, _, _, _ = leer_trama(bytes(mala))
    assert seq is None
    assert leer_trama(armar_trama(1, 2, b"")[:-1])[0] is None       # incompleta


def test_supervisor_ciclo():
    s = Supervisor(P.supervisor)
    t = 0.0
    s.paso(t, False, 0.0)
    assert s.estado == Estado.BUSQUEDA
    for _ in range(P.supervisor.n_adquisicion):
        t += 0.025
        s.paso(t, True, 1.0)
    assert s.estado == Estado.SEGUIMIENTO
    for _ in range(20):
        t += 0.025
        s.paso(t, False, 0.5)
    assert s.estado == Estado.PERDIDA
    s.paso(t + 0.01, True, 1.0, enlace_ok=False)
    assert s.estado == Estado.MODO_SEGURO
    s.paso(t + 0.02, True, 1.0, enlace_ok=True)
    assert s.estado == Estado.MODO_SEGURO          # rearme sólo manual (D21)


def test_omega_a_pasos():
    assert abs(omega_a_pasos(11.25, P.actuador) - 100.0) < 1e-9   # 0,1125 °/micropaso
    assert omega_a_pasos(1e6, P.actuador) == P.actuador.v_max_pasos_s


def test_flujo_sintetico_corto():
    fu = FuenteSintetica(duracion_s=1.0, oclusiones=(), falsos=())
    d = DetectorLaser(P.deteccion)
    n = sum(d.procesar(c.img).valido for c in fu)
    assert n >= 38


if __name__ == "__main__":
    fallas = 0
    for nombre, f in list(globals().items()):
        if nombre.startswith("test_") and callable(f):
            try:
                f()
                print(f"OK    {nombre}")
            except Exception as e:
                fallas += 1
                print(f"FALLA {nombre}: {e!r}")
    sys.exit(1 if fallas else 0)
