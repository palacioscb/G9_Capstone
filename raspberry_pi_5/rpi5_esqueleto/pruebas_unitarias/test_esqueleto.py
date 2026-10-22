"""
Pruebas unitarias del esqueleto. Corren en el PC y en la RPi, sin hardware.

    python3 -m pytest pruebas_unitarias -q      (si tienes pytest)
    python3 pruebas_unitarias/test_esqueleto.py (sin pytest)
"""
import math
import os
import struct
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


# ---------------------------------------------------------------------------
#* Integración del lazo cerrado (detector rojo, geometría con pose, enlace, modelo pid_t_2)
# ---------------------------------------------------------------------------
from pt.deteccion import DetectorLaserRojo                                    # noqa: E402
from pt.enlace import (Telemetria, trama_consigna_theta, trama_telemetria,    # noqa: E402
                       leer_telemetria, HistoriaAngulos, RelojESP32, EnlaceTrama,
                       TIPO_CONSIGNA_THETA, TIPO_COMANDO, CMD_HABILITAR, CMD_DESHABILITAR)
from pt.modelo_eje import PAN, TILT, polos_lazo_cerrado, simular_escalon, metricas_escalon  # noqa: E402


def test_geometria_mundo_centro_es_la_pose():
    #! Prueba que detectó un error de signo que la ida y vuelta NO detectaba.
    g = Geometria.desde_parametros(P)
    for pan, tilt in [(20, 10), (-35, -20), (0, 25), (50, -5)]:
        po, to = g.direccion_mundo(g.cx, g.cy, pan, tilt)
        assert abs(po - pan) < 1e-9 and abs(to - tilt) < 1e-9


def test_geometria_mundo_coincide_y_vuelve():
    g = Geometria.desde_parametros(P)
    for u, w in [(0, 0), (639, 479), (100, 400)]:
        a, b = g.pix_a_ang(u, w), g.direccion_mundo(u, w, 0, 0)
        assert abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9
    for (u, w, pan, tilt) in [(10, 20, 30, -15), (600, 450, -45, 25)]:
        po, to = g.direccion_mundo(u, w, pan, tilt)
        u2, w2 = g.mundo_a_pix(po, to, pan, tilt)
        assert abs(u - u2) < 1e-6 and abs(w - w2) < 1e-6
    po, to = g.direccion_mundo(g.cx, g.cy - 100, 0, 0)
    assert to > 0                                    # arriba en la imagen = tilt positivo


def test_detector_rojo_ignora_lampara_blanca():
    fu = FuenteSintetica(duracion_s=0.5, fps=80, oclusiones=(), falsos=(), lampara=(500, 100))
    dr, dy = DetectorLaserRojo(P.deteccion_roja), DetectorLaser(P.deteccion)
    er, ey = [], []
    for c in fu:
        r, q = dr.procesar(c.img, True, c.v), dy.procesar(c.img, True)
        assert r.valido, r.motivo
        er.append(math.hypot(r.u - c.verdad[0], r.w - c.verdad[1]))
        if q.valido:
            ey.append(math.hypot(q.u - c.verdad[0], q.w - c.verdad[1]))
    assert max(er) < 1.0                             # el rojo sigue al láser (< 1 px)
    assert ey and min(ey) > 100                      # el de brillo se va a la lámpara


def test_detector_rojo_sin_plano_v():
    r = DetectorLaserRojo(P.deteccion_roja).procesar(np.full((480, 640), 40, np.uint8), True, None)
    assert not r.valido


def test_trama_consigna_theta_y_telemetria():
    seq, tipo, carga, _ = leer_trama(trama_consigna_theta(9, 12.5, -3.25, 4.0, -1.0))
    assert seq == 9 and tipo == TIPO_CONSIGNA_THETA
    assert struct.unpack("<ffff", carga) == (12.5, -3.25, 4.0, -1.0)
    t = Telemetria(123456, 1.5, -2.5, 1.6, -2.4, 10.0, -5.0, 2, 0x40, 9)
    seq, tipo, carga, _ = leer_trama(trama_telemetria(3, t))
    assert len(carga) == 31
    t2 = leer_telemetria(carga)
    assert t2.t_us == 123456 and t2.estado == 2 and t2.fallas == 0x40 and abs(t2.pan_med - 1.5) < 1e-6
    assert t2.nombre_estado == "LAZO" and "pérdida de pasos" in t2.texto_fallas


def test_historia_y_reloj():
    h = HistoriaAngulos(extrapolacion_max_s=0.010)
    for i in range(10):
        h.agregar(i * 5_000_000, i * 1.0, -i * 0.5)          # cada 5 ms
    pan, tilt = h.interpolar(12_500_000)
    assert abs(pan - 2.5) < 1e-9 and abs(tilt + 1.25) < 1e-9
    assert h.interpolar(45_000_000 + 20_000_000) is None     # fuera del historial
    r = RelojESP32()
    a = r.a_rpi(2 ** 32 - 1000, 10_000_000_000)
    b = r.a_rpi(4000, 10_000_000_000 + 5_000_000)            # desborde de micros()
    assert abs((b - a) - 5_000_000) < 1000


class _SerialFalso:
    """Puerto serie de mentira: entrega telemetría como el ESP32 y acepta escrituras."""
    def __init__(self, datos):
        self.datos, self.escrito = bytearray(datos), bytearray()

    @property
    def in_waiting(self):
        return len(self.datos)

    def read(self, n):
        import time as _t
        if not self.datos:
            _t.sleep(0.002)
            return b""
        n = min(n, 40)
        out, self.datos = bytes(self.datos[:n]), self.datos[n:]
        return out

    def write(self, b):
        self.escrito += b

    def close(self):
        pass


def _tramas(b):
    """Todas las tramas válidas (tipo, carga) de un flujo de bytes."""
    out, buf = [], bytes(b)
    while True:
        seq, tipo, carga, buf = leer_trama(buf)
        if seq is None:
            return out
        out.append((tipo, carga))


def _telemetria_falsa(estado=1):
    datos = b"# AS5600 pan: OK\n"
    for i in range(40):                                      # 200 ms de telemetría a 200 Hz
        datos += trama_telemetria(i, Telemetria(i * 5000, 0.1 * i, -0.05 * i, 0, 0, 0, 0, estado, 0, 0))
        if i == 20:
            datos += b"\xAA\x55basura"                     # trama corrupta en medio
    return datos


def test_enlace_trama_con_serial_falso():
    import time as _t
    enl = EnlaceTrama(P.enlace, habilitar=True, puerto_serie=_SerialFalso(_telemetria_falsa()))
    _t.sleep(0.2)
    assert enl.tramas_rx == 40, enl.tramas_rx
    assert enl.telemetria.estado == 1 and abs(enl.telemetria.pan_med - 3.9) < 1e-5
    assert any("AS5600" in x for x in enl.textos)
    assert enl.listo() or not enl.ok()                       # RETENCION: listo mientras haya telemetría
    enl.enviar_consigna(1.0, 2.0)
    tr = _tramas(enl.s.escrito)
    assert tr[0] == (TIPO_COMANDO, bytes([CMD_HABILITAR]))
    assert tr[-1][0] == TIPO_CONSIGNA_THETA
    enl._corriendo = False


def test_modo_observar_nunca_manda_consignas():
    #! Hallazgo crítico de la revisión del 21-10: sin --habilitar, ni una consigna.
    import time as _t
    enl = EnlaceTrama(P.enlace, habilitar=False, puerto_serie=_SerialFalso(_telemetria_falsa()))
    for _ in range(5):
        enl.enviar_consigna(10.0, 5.0, 1.0, 1.0)
        enl.retener()
        _t.sleep(0.05)
    tr = _tramas(enl.s.escrito)
    assert tr and all(t == (TIPO_COMANDO, bytes([CMD_DESHABILITAR])) for t in tr), tr
    enl._corriendo = False


def test_reinicio_del_esp32_y_trama_cortada():
    r = RelojESP32()
    r.a_rpi(5_000_000, 10_000_000_000)
    b = r.a_rpi(1000, 10_010_000_000)                        # micros() volvió atrás: reinicio
    assert r.reinicios == 1 and abs(b - 10_010_000_000) < 1000
    t = trama_consigna_theta(1, 1.0, 2.0)
    mala = bytearray(t); mala[6] ^= 0xFF
    _, _, _, resto = leer_trama(bytes(mala) + b"\xAA")      # un 0xAA suelto al final se conserva
    assert resto == b"\xAA"


def test_instante_por_fila_persiana_rodante():
    from pt.camara import Cuadro
    c = Cuadro(np.zeros((480, 640), np.uint8), 1_000_000, 0, 0, exp_ns=2_000_000,
               lectura_ns=12_000_000, alto=480)
    assert c.instante(0) == 1_000_000 + 1_000_000
    assert c.instante(240) == 1_000_000 + 1_000_000 + 6_000_000
    assert c.instante(None) == c.instante(240)


def test_modelo_pid_t2_reproduce_polos_del_pdf():
    # pid_t_2.pdf §8: pan (Kp=5, Ki=6,25) y tilt (Kp=20, Ki=100), ζ = 0,05
    pp = sorted(polos_lazo_cerrado(PAN, 5, 6.25), key=lambda z: (z.real, z.imag))
    assert abs(pp[0].real + 4.60) < 0.01 and abs(abs(pp[0].imag) - 141.8) < 0.1
    assert abs(sorted(p.real for p in pp if abs(p.imag) < 1e-9)[0] + 2.60) < 0.01
    pt_ = polos_lazo_cerrado(TILT, 20, 100)
    assert any(abs(p.real + 58.35) < 0.05 and abs(abs(p.imag) - 1365.0) < 0.5 for p in pt_)
    assert abs(PAN.kp_max - 14.2) < 0.05 and abs(TILT.kp_max - 136.7) < 0.1


def test_modelo_escalon_converge():
    s = simular_escalon(PAN, 5, 6.25, 10.0, duracion_s=4.0, omega_max=math.inf, alfa_max=math.inf,
                        cuantizar=False, banda_integral=math.inf)
    m = metricas_escalon(s.t, s.theta, 0.0, 10.0)
    assert abs(m["error_final_deg"]) < 0.05 and 5 < m["sobrepaso_pct"] < 25


def test_frenado_evita_sobrepaso_grande():
    # Revisión del 21-10: sin tope de frenado ni banda integral, tilt se pasaba 27,8° en 60°.
    for p, kp, ki, esc in ((TILT, 10, 25, 60.0), (PAN, 3, 2.25, 120.0), (PAN, 5, 6.25, 60.0)):
        s = simular_escalon(p, kp, ki, esc, duracion_s=6.0)
        assert s.theta.max() - esc < 1.0, (kp, esc, s.theta.max() - esc)



def test_monitor_red_sirve_y_exporta():
    """MON-1 / MON-2: el servidor arranca, entrega config, flujo (sin NaN) y CSV."""
    import json
    import socket
    import urllib.request
    from pt.monitor_red import ServidorMonitor, NOMBRES
    with socket.socket() as s:                       # puerto libre
        s.bind(("127.0.0.1", 0))
        puerto = s.getsockname()[1]
    srv = ServidorMonitor(puerto=puerto, host="127.0.0.1", hz=50, imagen_hz=0, segundos=1, fps=80,
                          config={"fps": 80.0, "cx": 319.5, "cy": 239.5})
    srv.iniciar()
    try:
        nan = float("nan")
        for k in range(100):                         # más filas que la historia (80): se descartan las viejas
            fila = [k / 80, k, 1, 320.0, 240.0, 0.5, 0.5, 0.06, 0.06, 1.0, 2.0, 1.1, 2.1, nan, nan,
                    3.0, -3.0, 18.0, 1.0, "SEGUIMIENTO"]
            assert len(fila) == len(NOMBRES)
            srv.publicar_fila(tuple(fila))
        srv.publicar_estado({"modo": "SEGUIMIENTO", "lat_ms": nan})
        base = f"http://127.0.0.1:{puerto}"
        cfg = json.loads(urllib.request.urlopen(base + "/config", timeout=2).read())
        assert [c["nombre"] for c in cfg["columnas"]] == NOMBRES
        est = json.loads(urllib.request.urlopen(base + "/estado", timeout=2).read())
        assert est["lat_ms"] is None                 # NaN -> null (JSON válido)
        r = urllib.request.urlopen(base + "/flujo", timeout=2)
        linea = r.readline().decode()
        r.close()
        msg = json.loads(linea[len("data: "):])
        assert msg["e"]["modo"] == "SEGUIMIENTO" and msg["f"][-1][1] == 99 and msg["f"][-1][13] is None
        csv_txt = urllib.request.urlopen(base + "/historial.csv", timeout=2).read().decode().splitlines()
        assert csv_txt[0].split(",") == NOMBRES and len(csv_txt) == 1 + 80
        assert urllib.request.urlopen(base + "/", timeout=2).read().startswith(b"<!doctype html>")
    finally:
        srv.detener()


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
