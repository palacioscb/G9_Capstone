"""
Pruebas del recorte de imagen (pt/recorte.py). Corren sin hardware ni OpenCV.

    python3 -m pytest pruebas_unitarias/test_recorte.py -q      (si tienes pytest)
    python3 pruebas_unitarias/test_recorte.py                   (sin pytest)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pt.recorte import RecorteROI, validar                        # noqa: E402


def _cuadro():
    y = np.full((480, 640), 120, np.uint8)
    y[10:20, 10:20] = 0            # objeto FUERA del recorte
    y[240:250, 320:330] = 0        # objeto DENTRO
    return y


def _activar(r, pts):
    for i, (x, y) in enumerate(pts, 1):
        r.fijar(f"roi.x{i}", x)
        r.fijar(f"roi.y{i}", y)
    r.fijar("roi.activo", True)


def test_quita_lo_de_afuera_y_conserva_lo_de_adentro():
    r = RecorteROI(640, 480)
    _activar(r, [(100, 80), (500, 80), (500, 400), (100, 400)])
    y = _cuadro()
    y2, _ = r.aplicar(y, None)
    assert (y2[10:20, 10:20] == 120).all()          # lo de afuera quedó del gris del fondo
    assert (y2[240:250, 320:330] == 0).all()        # lo de adentro no se toca
    assert y[10:20, 10:20].max() == 0               # el cuadro original NO se modifica


def test_plano_v_a_media_resolucion():
    r = RecorteROI(640, 480)
    _activar(r, [(100, 80), (500, 80), (500, 400), (100, 400)])
    v = np.full((240, 320), 140, np.uint8)
    v[5:10, 5:10] = 250                             # destello fuera
    _, v2 = r.aplicar(_cuadro(), v)
    assert v2.shape == v.shape and v2[5:10, 5:10].max() == 140


def test_el_orden_de_los_clics_no_importa():
    r = RecorteROI(640, 480)
    _activar(r, [(100, 80), (500, 400), (500, 80), (100, 400)])      # en "mariposa"
    assert r.activo and [tuple(p) for p in r.pts] == [(100, 80), (500, 80), (500, 400), (100, 400)]


def test_rechaza_cuadrilateros_invalidos():
    for malo in ([(10, 10), (20, 10), (30, 10), (40, 10)],            # alineados
                 [(0, 0), (5, 0), (5, 5), (0, 5)],                     # diminuto
                 [(100, 100), (500, 100), (300, 200), (300, 100)]):    # no convexo / repetido
        try:
            validar(malo, 640, 480)
        except ValueError:
            continue
        raise AssertionError(f"debía rechazar {malo}")
    r = RecorteROI(640, 480)
    for i in range(1, 5):
        r.fijar(f"roi.x{i}", 10), r.fijar(f"roi.y{i}", 10)
    try:
        r.fijar("roi.activo", True)
    except ValueError:
        assert not r.activo                                            # un recorte inválido no se activa
    else:
        raise AssertionError("se activó un recorte inválido")


def test_si_falla_se_apaga_y_devuelve_el_cuadro_tal_cual():
    r = RecorteROI(640, 480)
    _activar(r, [(100, 80), (500, 80), (500, 400), (100, 400)])
    r._m, r.pts = {}, None                                             # rompe el estado a propósito
    y = _cuadro()
    y2, v2 = r.aplicar(y, None)
    assert y2 is y and v2 is None and not r.activo and r.error


def test_desactivar_conserva_las_esquinas():
    r = RecorteROI(640, 480)
    _activar(r, [(100, 80), (500, 80), (500, 400), (100, 400)])
    r.fijar("roi.activo", False)
    assert not r.activo and r.coords["roi.x2"] == 500


if __name__ == "__main__":
    ok = 0
    for nombre, f in sorted(globals().items()):
        if nombre.startswith("test_"):
            f()
            ok += 1
            print("ok ", nombre)
    print(f"{ok} pruebas del recorte: todas ok")
