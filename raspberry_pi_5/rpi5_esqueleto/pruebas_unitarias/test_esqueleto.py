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



def test_detector_luz_flash_grande_y_pixel_caliente():
    """DetectorLuz (de luz_2.py): mancha saturada de 22 px de diámetro + un píxel caliente."""
    from pt.deteccion import DetectorLuz
    from pt.parametros import DeteccionLuz
    H, W = 480, 640
    vv, uu = np.mgrid[0:H, 0:W]
    det = DetectorLuz(DeteccionLuz())
    for (u0, w0) in [(400.3, 120.7), (405.8, 126.2), (90.5, 400.25)]:
        img = np.full((H, W), 20, np.uint8)
        d = np.hypot(uu - u0, vv - w0)
        img[d <= 11] = 255                              # flash saturado
        borde = (d > 11) & (d < 14)
        img[borde] = 180
        img[50, 600] = 255                              # píxel caliente aislado
        r = det.procesar(img, usar_ventana=True)
        if (u0, w0) == (90.5, 400.25):                  # salto grande: debe rechazarse por salto
            assert not r.valido and "salto" in r.motivo, r
            det.reiniciar()
            r = det.procesar(img, usar_ventana=True)
        assert r.valido, r.motivo
        assert abs(r.u - u0) < 0.6 and abs(r.w - w0) < 0.6, (r.u, r.w, u0, w0)
    vacia = np.full((H, W), 30, np.uint8)
    assert not det.procesar(vacia).valido

def test_detector_avion():
    from pt.parametros import Parametros
    from pt.deteccion import DetectorAvionRojo
    P = Parametros()
    det = DetectorAvionRojo(P)
    v_dummy = np.zeros((240, 320), dtype=np.uint8)
    v_dummy[120, 160] = 200  # Valor sobre el umbral simulado
    res = det.procesar(v_dummy, usar_ventana=False, v=v_dummy)
    assert res is not None

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


def _ajustes_de_prueba(ruta=None):
    from pt.ajustes import Ajuste, Ajustes
    lista = [Ajuste("exposicion_us", "Exposición", "Cámara", "int", 2000, 100, 12500, 100, "µs"),
             Ajuste("ganancia", "Ganancia", "Cámara", "float", 1.0, 1.0, 10.0, 0.1),
             Ajuste("detector", "Detector", "Objetivo", "opcion", "rojo", opciones=("rojo", "luz", "avion")),
             Ajuste("luz.umbral", "Umbral", "Detector", "int", 220, 1, 255, 1, detector="luz"),
             Ajuste("rojo.v_umbral", "Umbral V", "Detector", "int", 150, 129, 255, 1, detector="rojo"),
             Ajuste("feedforward", "FF", "Predicción", "bool", False, en_perfil=False)]
    return Ajustes(lista, {"luz": {"detector": "luz", "exposicion_us": 3000, "clave_de_otra_corrida": 1}}, ruta)


def test_ajustes_validan_tipo_rango_y_clave():
    """Lo que no está declarado, o está fuera de rango, no llega al lazo."""
    aj = _ajustes_de_prueba()
    ok, mal = aj.pedir({"exposicion_us": 4000.0, "ganancia": 99, "detector": "radar", "feedforward": 1,
                        "habilitar": True, "luz.umbral": 200.5, "rojo.v_umbral": float("nan")})
    assert ok == {"exposicion_us": 4000} and isinstance(ok["exposicion_us"], int)
    assert set(mal) == {"ganancia", "detector", "feedforward", "habilitar", "luz.umbral", "rojo.v_umbral"}
    assert aj.pedir([1, 2])[0] == {} and aj.pedir(None)[0] == {}
    assert aj.valores["exposicion_us"] == 2000              # pedir no aplica: sólo encola
    assert aj.pendientes() == [("exposicion_us", 4000, "")] and aj.pendientes() == []
    aj.confirmar("exposicion_us", 4000)
    assert aj.valores["exposicion_us"] == 4000 and aj.estado()["v"] == 1


def test_ajustes_perfiles_aplicar_y_guardar():
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        ruta = os.path.join(d, "perfiles.json")
        aj = _ajustes_de_prueba(ruta)
        ok, mal = aj.pedir_perfil("luz", "prueba")
        assert ok == {"detector": "luz", "exposicion_us": 3000} and mal == {}   # la clave ajena se omite
        pend = aj.pendientes()
        assert [p[0] for p in pend] == ["detector", "exposicion_us", "_perfil"]  # el detector va primero
        for clave, valor, origen in pend:
            aj.confirmar(clave, valor, origen)
        assert aj.perfil_activo == "luz" and not aj.perfil_modificado
        aj.confirmar("luz.umbral", 180)
        assert aj.perfil_modificado
        assert aj.pedir_perfil("no_existe")[1]
        assert not aj.guardar_perfil("../etc")[0]            # el nombre no puede ser una ruta
        assert aj.guardar_perfil("flash")[0]
        guardado = json.load(open(ruta, encoding="utf-8"))["flash"]
        #* Se guarda lo común y lo del detector activo; no el umbral de otro detector ni lo global.
        assert guardado == {"exposicion_us": 3000, "ganancia": 1.0, "detector": "luz", "luz.umbral": 180}
        assert "flash" in _ajustes_de_prueba(ruta).perfiles  # otra corrida lo vuelve a cargar
        aj.guardar_perfil("luz")                             # regrabar conserva las claves que esta corrida no maneja
        assert aj.perfiles["luz"]["clave_de_otra_corrida"] == 1 and aj.perfiles["luz"]["luz.umbral"] == 180


def test_monitor_red_post_de_ajustes():
    """Ruta de escritura del tablero: valida, encola y nunca aplica por su cuenta."""
    import json
    import socket
    import urllib.error
    import urllib.request
    from pt.monitor_red import ServidorMonitor

    def pedir(base, cuerpo, tipo="application/json"):
        datos = cuerpo if isinstance(cuerpo, bytes) else json.dumps(cuerpo).encode()
        req = urllib.request.Request(base + "/ajustes", data=datos, headers={"Content-Type": tipo}, method="POST")
        try:
            r = urllib.request.urlopen(req, timeout=2)
            return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    servidores = []
    try:
        bases = []
        for aj in (_ajustes_de_prueba(), None):
            with socket.socket() as s:
                s.bind(("127.0.0.1", 0))
                puerto = s.getsockname()[1]
            srv = ServidorMonitor(puerto=puerto, host="127.0.0.1", imagen_hz=0, config={"fps": 80.0}, ajustes=aj)
            srv.iniciar()
            servidores.append(srv)
            bases.append(f"http://127.0.0.1:{puerto}")
        base, base_ro = bases
        aj = servidores[0].ajustes
        d = json.loads(urllib.request.urlopen(base + "/ajustes", timeout=2).read())
        assert d["habilitado"] and d["val"]["detector"] == "rojo" and d["perfiles"] == ["luz"]
        assert json.loads(urllib.request.urlopen(base + "/config", timeout=2).read())["ajustable"] is True
        assert pedir(base, {"valores": {"exposicion_us": 5000}}) == (200, {"ok": True, "aceptados": {"exposicion_us": 5000}, "rechazados": {}})
        assert aj.pendientes() == [("exposicion_us", 5000, "127.0.0.1")]
        codigo, r = pedir(base, {"valores": {"exposicion_us": 999999, "habilitar": True}})
        assert codigo == 400 and not r["ok"] and set(r["rechazados"]) == {"exposicion_us", "habilitar"}
        assert pedir(base, {"valores": {"exposicion_us": 5000}}, tipo="text/plain")[0] == 415   # formulario de otra página
        assert pedir(base, b"{no es json")[0] == 400
        assert pedir(base, {"valores": {"x": "y" * 5000}})[0] == 413
        assert pedir(base, {"perfil": "luz"})[0] == 200 and len(aj.pendientes()) == 3
        assert aj.pendientes() == [] and aj.valores["exposicion_us"] == 2000   # nada se aplicó sin el lazo
        #* Sólo lectura (--monitor-solo-lectura): no hay ajustes y el POST se rechaza.
        assert json.loads(urllib.request.urlopen(base_ro + "/ajustes", timeout=2).read()) == {"habilitado": False}
        assert pedir(base_ro, {"valores": {"exposicion_us": 5000}})[0] == 403
    finally:
        for srv in servidores:
            srv.detener()


def test_ajustes_en_vivo_cambio_de_objetivo():
    """El lazo aplica lo pedido: otro detector vuelve a BÚSQUEDA; λ y umbrales cambian sin reiniciar."""
    import argparse
    import main as programa
    from pt.ajustes import Ajustes
    from pt.control import ReferenciaVisual
    from pt.deteccion import DetectorLaserRojo, DetectorLuz, DetectorAvionRojo

    class CamaraFalsa:
        pedidos = []

        def ajustar(self, **kw):
            self.pedidos.append(kw)

    Pp = Parametros()
    a = argparse.Namespace(detector="rojo")
    dets = programa.crear_detectores(Pp)
    assert set(dets) == set(programa.DETECTORES)                             # incluye los dos por forma (requieren OpenCV)
    ref = ReferenciaVisual(Pp, Geometria.desde_parametros(Pp))
    sup = Supervisor(Pp.supervisor)
    sup.estado = Estado.SEGUIMIENTO
    cam = CamaraFalsa()
    aj = Ajustes(programa.lista_de_ajustes(a, Pp, con_camara=True))
    vivo = programa.AjustesEnVivo(aj, a, Pp, cam, dets, ref, sup)
    rls_antes = ref.est.eu.rls

    assert aj.pedir({"lam": 0.5})[1]                                         # bajo el piso de λ: rechazado
    aj.pedir({"lam": 0.9, "rojo.v_umbral": 165, "exposicion_us": 4000, "color_rojo": 2.4, "feedforward": True,
              "sigma_min_deg": 0.3})
    det = vivo.aplicar(dets["rojo"], 1.0)
    assert det is dets["rojo"] and sup.estado == Estado.SEGUIMIENTO          # nada de esto interrumpe el seguimiento
    assert Pp.prediccion.lam == 0.9 and ref.est.eu.rls is rls_antes and ref.est.eu.rls.lam == 0.9 and ref.est.ew.rls.lam == 0.9
    assert Pp.deteccion_roja.v_umbral == 165 and Pp.lazo.usar_feedforward is True
    assert ref.est.eu.sigma_min == 0.3 and ref.est.ew.sigma_min == 0.3
    assert {"exposicion_us": 4000} in cam.pedidos and {"ganancias_color": (2.4, Pp.captura.ganancias_color[1])} in cam.pedidos
    assert aj.valores["lam"] == 0.9 and aj.valores["exposicion_us"] == 4000

    aj.pedir({"detector": "avion", "avion.contraste_min": 40, "avion.polaridad": "claro", "avion_rojo.min_pix": 12})
    det = vivo.aplicar(det, 2.0)
    assert det is dets["avion"] and a.detector == "avion" and det.p is Pp.forma_avion
    assert Pp.forma_avion.contraste_min == 40 and Pp.forma_avion.polaridad == "claro" and Pp.deteccion_avion.min_pix == 12
    assert aj.pedir({"avion.polaridad": "rojo"})[1]                          # opción inexistente: rechazada
    assert sup.estado == Estado.BUSQUEDA and "cambio de objetivo" in sup.bitacora[-1][3]
    assert ref.est.eu.rls is not rls_antes and ref.est.eu.rls.lam == 0.9     # estimador nuevo, con el λ vigente
    assert ref.est.eu.sigma_min == 0.3                                       # y con el piso de gating vigente

    #! Si la cámara rechaza un control, el lazo sigue y el valor anterior queda vigente.
    def falla(**kw):
        raise RuntimeError("control rechazado")
    cam.ajustar = falla
    aj.pedir({"ganancia": 3.0})
    assert vivo.aplicar(det, 3.0) is det and aj.valores["ganancia"] == Pp.captura.ganancia


def test_registro_se_parte_en_tramos_por_detector_y_feedforward():
    """Una corrida con cambios desde el tablero se analiza por tramos (herramientas/analizar_registro.py)."""
    from herramientas.analizar_registro import partir_en_tramos
    filas = ([{"detector": "rojo", "ff": "0"}] * 5 + [{"detector": "avion", "ff": "0"}] * 7
             + [{"detector": "avion", "ff": "1"}] * 4)
    assert partir_en_tramos(filas) == [(0, 5, "rojo", 0), (5, 12, "avion", 0), (12, 16, "avion", 1)]
    #* Registros anteriores al 17-11 no traen esas columnas: quedan como un solo tramo.
    assert partir_en_tramos([{"valido": "1"}] * 9) == [(0, 9, "", None)]
    from pt.registro import COLUMNAS
    assert {"detector", "exp_us", "ganancia", "lam", "ff"} <= set(COLUMNAS)


def test_ganancias_sim_y_comparacion_por_grupos():
    """--ganancias-sim cambia sólo el PI simulado; el grupo A de la comparación es el de config.h."""
    from pt.enlace import EnlaceSimulado
    from herramientas.comparar_ganancias_ff import GRUPOS, mejora
    ganancias = lambda enl: [(c.kp, c.ki) for c in enl.ctl]
    assert ganancias(EnlaceSimulado(Parametros())) == [GRUPOS["A"]["pan"], GRUPOS["A"]["tilt"]]
    b = GRUPOS["B"]
    enl = EnlaceSimulado(Parametros(), kp=(b["pan"][0], b["tilt"][0]), ki=(b["pan"][1], b["tilt"][1]))
    assert ganancias(enl) == [(5.0, 6.25), (20.0, 100.0)]
    import main as programa
    argv, sys.argv = sys.argv, ["main.py", "--enlace", "simulado", "--ganancias-sim", "5", "6.25", "20", "100"]
    try:
        assert programa.argumentos().ganancias_sim == [5.0, 6.25, 20.0, 100.0]
    finally:
        sys.argv = argv
    casos = {("simulado", "A", 0): {"res": {"e_pan": 3.0}}, ("simulado", "A", 1): {"res": {"e_pan": 0.5}}}
    assert mejora(casos, "simulado", "A", "e_pan") == 6.0
    assert mejora(casos, "medido", "A", "e_pan") is None          # sin datos de laboratorio todavía


def test_fijar_ganancias_confirma_con_la_respuesta_del_firmware():
    """herramientas/fijar_ganancias.py: manda 'k <eje> <Kp> <Ki>' y exige la confirmación del ESP32."""
    from herramientas.fijar_ganancias import fijar

    class SerialFalso:
        def __init__(self, responde):
            self.responde, self.tx, self.rx = responde, b"", b""

        def reset_input_buffer(self):
            self.rx = b""

        def write(self, b):
            self.tx += b
            if self.responde:                # igual que aviso() en src/main.cpp, comando k
                _, eje, kp, ki = b.decode().split()
                self.rx = f"# {eje}: Kp={float(kp):.3f} 1/s  Ki={float(ki):.3f} 1/s^2\r\n".encode()

        def read(self, n):
            b, self.rx = self.rx, b""
            return b

    s = SerialFalso(True)
    assert fijar(s, "pan", 5.0, 6.25, espera=0.02) and s.tx == b"k pan 5 6.25\n"
    assert not fijar(SerialFalso(False), "tilt", 20.0, 100.0, espera=0.02)     # main.py corriendo: sin respuesta


def _seguir(pp, medicion, n=200, piso=0.1):
    """Estimador2D + Supervisor sobre medicion(k) -> (pan, tilt). Devuelve la lista de (Estimacion, estado)."""
    from pt.prediccion import Estimador2D
    est, sup, salida = Estimador2D(pp, sigma_min=piso), Supervisor(Parametros().supervisor), []
    for k in range(n):
        t = k / 80.0
        e = est.actualizar_xy(t, True, *medicion(k), t + 0.02)
        salida.append((e, sup.paso(t, True, e.confianza)))
    return est, salida


def test_reinicio_informado_no_pasa_por_perdida():
    """Tras los rechazos, el estimador nuevo parte con esas mediciones: confianza > 0 y sin PÉRDIDA."""
    salto = lambda k: (15.0 * k / 80 + (3.0 if k >= 100 else 0.0), 5.0 * k / 80)      # salto real de 3° en pan
    pp = Parametros().prediccion
    assert pp.reinicio_informado and pp.gating_conjunto
    est, s = _seguir(pp, salto)
    estados = [(e.estado_u, e.estado_w) for e, _ in s[100:104]]
    assert estados == [("rechazada", "rechazada")] * 3 + [("reinicio", "aceptada")]   # pan se reinicia; tilt sigue
    assert all(e.confianza > 0.5 for e, _ in s[10:]) and all(st != Estado.PERDIDA for _, st in s)
    assert est.eu.rls.listo() and est.ew.rls.n_valid > 100                             # tilt conserva su historia
    assert abs(s[110][0].u - (15.0 * (110 / 80 + 0.02) + 3.0)) < 0.05                  # ya sigue la trayectoria nueva
    assert abs(s[104][0].vu - 15.0) < 1.0                                              # y la velocidad (ω_ff) no se corta

    #* Con los cambios apagados se reproduce lo anterior: estimador desde cero, confianza 0 y PÉRDIDA.
    pp = Parametros().prediccion
    pp.reinicio_informado, pp.gating_conjunto, pp.gracia_gating = False, False, 0
    _, s = _seguir(pp, salto)
    assert s[103][0].estado_u == "reinicio" and s[103][0].confianza == 0.0
    assert any(st == Estado.PERDIDA for _, st in s[100:110])


def test_gating_conjunto_y_piso():
    """Decisión única para los dos ejes, sin estrechar el umbral: cada eje conserva k·max(σ, piso)."""
    pp = Parametros().prediccion
    k_g, piso = pp.k_gating, 0.1
    recta = lambda k: (15.0 * k / 80, 5.0 * k / 80)                # sin ruido: σ ≈ 0, manda el piso

    def con_desvio(dp, dt):
        return lambda k: (recta(k)[0] + (dp if k == 100 else 0.0), recta(k)[1] + (dt if k == 100 else 0.0))

    #* Aberrante sólo en tilt: se rechaza la medición completa, y pan no la incorpora.
    est, s = _seguir(pp, con_desvio(0.0, 1.1 * k_g * piso), n=101)
    assert (s[100][0].estado_u, s[100][0].estado_w) == ("rechazada", "rechazada") and s[100][0].d_gating > 1.0
    assert abs(est.eu.t_ult_valida - 99 / 80) < 1e-9 and 0.5 < s[100][0].confianza < 1.0
    #* Justo bajo el umbral de un eje, o en diagonal bajo el umbral de cada eje: se acepta (como antes).
    for dp, dt in ((0.9 * k_g * piso, 0.0), (0.9 * k_g * piso, 0.9 * k_g * piso)):
        _, s = _seguir(pp, con_desvio(dp, dt), n=101)
        assert s[100][0].estado_u == "aceptada" and 0.85 < s[100][0].d_gating < 1.0
    #* El piso se puede cambiar en caliente (tablero): con el doble, el mismo aberrante se acepta.
    _, s = _seguir(pp, con_desvio(0.0, 1.1 * k_g * piso), n=101, piso=2 * piso)
    assert s[100][0].estado_w == "aceptada"
    #* La confianza mira los dos ejes: si tilt no está listo, es 0 aunque pan lo esté.
    est, s = _seguir(pp, recta, n=50)
    est.ew.reiniciar()
    assert est.actualizar_xy(50 / 80, True, *recta(50)).confianza == 0.0


def test_oclusion_tope_de_aceleracion():
    """t_acel_max_s > 0: pasado ese tiempo sin medición se extrapola en línea recta, sin escalón."""
    from pt.prediccion import EstimadorEje
    parabola = lambda t: 2.0 + 10.0 * t + 40.0 * t * t             # aceleración de 80 °/s²
    pp = Parametros().prediccion
    assert pp.t_acel_max_s == 0.0                                   # apagado por defecto (ver parametros.py)

    def estimador(tope):
        p = Parametros().prediccion
        p.t_acel_max_s = tope
        e = EstimadorEje(p, sigma_min=0.1)
        for k in range(80):
            e.actualizar(k / 80.0, parabola(k / 80.0), True)
        return e, 79 / 80.0

    e, t0 = estimador(0.0)                                          # sin tope: la parábola completa
    assert abs(e.predecir(t0 + 0.3) - parabola(t0 + 0.3)) < 0.02
    e, t0 = estimador(0.04)
    assert abs(e.predecir(t0 + 0.03) - parabola(t0 + 0.03)) < 0.01  # dentro del tope: igual que antes
    v = e.velocidad(t0 + 0.04)
    assert abs(v - (10.0 + 80.0 * (t0 + 0.04))) < 0.5 and e.velocidad(t0 + 0.3) == v      # velocidad congelada
    p = [e.predecir(t0 + s) for s in (0.04, 0.10, 0.20, 0.30)]
    assert abs((p[1] - p[0]) - v * 0.06) < 1e-9 and abs((p[3] - p[2]) - v * 0.10) < 1e-9  # recta desde el tope
    assert abs(e.predecir(t0 + 0.04 + 1e-6) - e.predecir(t0 + 0.04 - 1e-6) - v * 2e-6) < 1e-6   # sin escalón de posición


def test_ajustes_en_vivo_del_predictor():
    """El tablero puede apagar cada cambio del 17-11 sin reiniciar main.py."""
    import argparse
    import main as programa
    from pt.ajustes import Ajustes
    from pt.control import ReferenciaVisual
    Pp = Parametros()
    a = argparse.Namespace(detector="rojo")
    ref = ReferenciaVisual(Pp, Geometria.desde_parametros(Pp))
    sup = Supervisor(Pp.supervisor)
    aj = Ajustes(programa.lista_de_ajustes(a, Pp, con_camara=False))
    vivo = programa.AjustesEnVivo(aj, a, Pp, None, {"rojo": DetectorLaser(Pp.deteccion)}, ref, sup)
    aceptados, rechazados = aj.pedir({"reinicio_informado": False, "gating_conjunto": False, "t_acel_max_s": 0.05})
    assert not rechazados
    vivo.aplicar(None, 1.0)
    pr = ref.est.pp
    assert (pr.reinicio_informado, pr.gating_conjunto, pr.t_acel_max_s) == (False, False, 0.05)
    assert aj.pedir({"t_acel_max_s": 0.5})[1]                       # fuera de rango
    from pt.registro import COLUMNAS
    assert {"gating_d", "sigma_pan", "sigma_tilt"} <= set(COLUMNAS)


def test_formas_descriptor_de_fourier_invariante():
    """pt/formas.py: el descriptor no cambia con posición, tamaño, giro ni inclinación de una figura plana."""
    from pt import formas as fm
    banco = fm.BancoDeFormas()
    avion = fm.poligono("avion_planta")
    A, (cx, cy), C = fm.momentos(np.array([(0, 0), (2, 0), (2, 1), (0, 1)]))        # rectángulo de 2 x 1
    assert abs(A - 2) < 1e-12 and abs(cx - 1) < 1e-12 and abs(cy - 0.5) < 1e-12
    assert abs(C[0, 0] - 4 / 12) < 1e-12 and abs(C[1, 1] - 1 / 12) < 1e-12 and abs(C[0, 1]) < 1e-12
    for tam, giro, centro, aplastar in ((40, 0.3, (100, 50), (1, 1)), (300, 2.1, (-20, 900), (1, 1)),
                                        (120, 4.0, (0, 0), (0.4, 1.0)), (80, 1.0, (5, 5), (1.0, 0.5))):
        d = fm.descriptor(fm.transformar(avion, tam, giro, centro, aplastar))
        assert fm.distancia(d, banco.plantillas["avion_planta"]) < 0.02
    assert fm.distancia(fm.descriptor(avion[::-1]), banco.plantillas["avion_planta"]) < 1e-3   # sentido de recorrido
    elipse = fm.transformar(fm.poligono("circulo"), 100, 0.7, (0, 0), (1.0, 0.35))
    assert fm.distancia(fm.descriptor(elipse), banco.plantillas["circulo"]) < 0.01            # toda elipse es un círculo inclinado
    assert abs(fm.alargamiento(elipse) - 1 / 0.35) < 0.02
    cuadrado = np.array([(0, 0), (1, 0), (1, 1), (0, 1)], dtype=float)
    assert fm.distancia(fm.descriptor(cuadrado), banco.plantillas["circulo"]) > 0.10
    assert fm.distancia(fm.descriptor(cuadrado), banco.plantillas["avion_planta"]) > 0.30
    assert fm.distancia(banco.plantillas["avion_planta"], banco.plantillas["circulo"]) > 0.40
    assert banco.mejor(fm.descriptor(fm.transformar(avion, 90, 1.2)), ("avion",))[0] == "avion_planta"


def test_banco_de_formas_aprende_y_prefiere_lo_aprendido():
    """Las vistas aprendidas (formas.json) reemplazan a las genéricas de su familia."""
    import tempfile
    from pt import formas as fm
    ruta = os.path.join(tempfile.mkdtemp(), "formas.json")
    banco = fm.BancoDeFormas(ruta)
    assert set(banco.de_la_familia("avion")) == {"avion_planta"}     # la genérica de perfil ya no está por defecto
    d = fm.descriptor(fm.poligono("avion_perfil")) * 1.1
    banco.guardar("avion_lado", d)
    otro = fm.BancoDeFormas(ruta)                                    # se vuelve a leer del archivo
    assert set(otro.de_la_familia("avion")) == {"avion_lado"} and "avion_lado" in otro.aprendidas
    assert np.allclose(otro.plantillas["avion_lado"], d, atol=1e-5)
    assert set(otro.de_la_familia("circulo")) == {"circulo"}        # otra familia: sin cambios
    try:
        banco.guardar("../fuera", d)
        assert False
    except ValueError:
        pass


def test_detector_por_forma_avion_y_circulo():
    """DetectorForma: encuentra la silueta entre otras figuras, sin usar color, y no inventa detecciones."""
    from herramientas import evaluar_forma as ev
    from pt.deteccion import DetectorForma
    Pp = Parametros()
    avion, ovni = DetectorForma(Pp.forma_avion), DetectorForma(Pp.forma_ovni)
    rng = np.random.default_rng(2)
    aciertos = 0
    for _ in range(12):                                              # avión entre rectángulos, manchas y una lámpara
        img, c = ev.escena(rng, objetivo="avion_planta", distraer=True, lampara=True, tam=(70, 120))
        avion.reiniciar()
        r = avion.procesar(img, usar_ventana=False)                  # sólo recibe el plano Y
        if r.valido:
            assert np.hypot(r.u - c[0], r.w - c[1]) < 3.0 and r.forma.startswith("avion") and r.dist <= Pp.forma_avion.dist_max
            aciertos += 1
    assert aciertos >= 11
    for _ in range(6):                                               # sin avión: no hay detección válida
        img, _ = ev.escena(rng, objetivo=None, distraer=True, lampara=True)
        avion.reiniciar()
        r = avion.procesar(img, usar_ventana=False)
        assert not r.valido and r.motivo
    img, c = ev.escena(rng, objetivo="circulo", tam=(50, 80), inclinar=0.5)
    r = ovni.procesar(img, usar_ventana=False)
    assert r.valido and r.forma == "circulo" and np.hypot(r.u - c[0], r.w - c[1]) < 2.0
    avion.reiniciar()
    r = avion.procesar(img, usar_ventana=False)                      # un disco no es un avión
    assert not r.valido and "ninguna figura parecida" in r.motivo
    #* En SEGUIMIENTO trabaja en la ventana alrededor de la detección anterior.
    img, c = ev.escena(rng, objetivo="avion_planta", tam=(80, 100))
    avion.reiniciar()
    r1 = avion.procesar(img, usar_ventana=True)
    r2 = avion.procesar(img, usar_ventana=True)
    assert r1.valido and r2.valido and abs(r1.u - r2.u) < 0.5 and abs(r1.w - r2.w) < 0.5


def test_fuente_sintetica_con_silueta():
    """--objeto avion: la fuente dibuja una silueta sin color y su centro de área es la verdad."""
    from pt.deteccion import DetectorForma, DetectorLaserRojo
    Pp = Parametros()
    src = FuenteSintetica(640, 480, 80, 0.2, objeto="avion", fondo=150, oclusiones=(), falsos=())
    det, rojo = DetectorForma(Pp.forma_avion), DetectorLaserRojo(Pp.deteccion_roja)
    for c in src:
        r = det.procesar(c.img, usar_ventana=True, v=c.v)
        assert r.valido and np.hypot(r.u - c.verdad[0], r.w - c.verdad[1]) < 1.5
        assert not rojo.procesar(c.img, usar_ventana=False, v=c.v).valido          # no hay nada rojo que detectar


def test_cambiar_de_detector_trae_su_perfil():
    """Elegir otro detector en el tablero aplica el perfil de ese objetivo; lo escrito en el pedido manda."""
    from pt.ajustes import perfil_de
    aj = _ajustes_de_prueba()
    assert perfil_de(aj.perfiles, "luz") == "luz" and perfil_de(aj.perfiles, "avion") is None
    assert perfil_de({"otro": {"detector": "rojo"}, "laser": {"detector": "rojo"}}, "rojo") == "laser"
    ok, mal = aj.pedir({"detector": "luz"}, "tablero")
    assert ok == {"detector": "luz", "exposicion_us": 3000} and mal == {}
    pend = aj.pendientes()
    assert [p[0] for p in pend] == ["detector", "exposicion_us", "_perfil"] and pend[-1][1] == "luz"
    for clave, valor, origen in pend:
        aj.confirmar(clave, valor, origen)
    assert aj.perfil_activo == "luz" and not aj.perfil_modificado
    assert aj.pedir({"detector": "luz"})[0] == {"detector": "luz"}           # mismo detector: no se repone el perfil
    aj.pendientes()
    aj.confirmar("detector", "rojo")
    assert aj.pedir({"detector": "luz", "exposicion_us": 5000})[0] == {"detector": "luz", "exposicion_us": 5000}
    aj.pendientes()
    assert aj.pedir({"detector": "avion"})[0] == {"detector": "avion"}       # sin perfil para ese detector: sólo cambia
    aj.pendientes()
    aj.detector_trae_perfil = False                                          # main.py --sin-perfil
    assert aj.pedir({"detector": "luz"})[0] == {"detector": "luz"}
    assert aj.pedir_perfil("luz")[0] == {"detector": "luz", "exposicion_us": 3000}   # el botón de perfil sigue igual


def test_perfil_del_avion_completo_y_dentro_de_rango():
    """El perfil «avion» cabe en los rangos del tablero y, al elegir el detector, repone cámara, umbrales y λ."""
    import argparse
    import json
    import main as programa
    from pt.ajustes import Ajustes
    from pt.control import ReferenciaVisual
    from pt.parametros import PERFILES

    class CamaraFalsa:
        def __init__(self):
            self.pedidos = []

        def ajustar(self, **kw):
            self.pedidos.append(kw)

    Pp = Parametros()
    a = argparse.Namespace(detector="rojo", exposicion=None, ganancia=None)
    aj = Ajustes(programa.lista_de_ajustes(a, Pp, con_camara=True), PERFILES)      # sin archivo: la tabla de parametros.py
    for clave, valor in PERFILES["avion"].items():
        assert clave in aj.specs and aj.validar(clave, valor)[0], clave
    with open(programa.RUTA_PERFILES, encoding="utf-8") as f:
        del_archivo = json.load(f).get("avion", {})
    for clave, valor in del_archivo.items():                                       # lo que el equipo regrabe debe seguir en rango
        assert clave not in aj.specs or aj.validar(clave, valor)[0], clave
    #* Los valores por defecto del detector son los del perfil: las herramientas (sin tablero) miden lo mismo.
    for clave, valor in PERFILES["avion"].items():
        if clave.startswith("avion.") and clave != "avion.salto_activo":
            assert getattr(Pp.forma_avion, clave.split(".", 1)[1]) == valor, clave

    dets = programa.crear_detectores(Pp)
    ref = ReferenciaVisual(Pp, Geometria.desde_parametros(Pp))
    sup = Supervisor(Pp.supervisor)
    cam = CamaraFalsa()
    vivo = programa.AjustesEnVivo(aj, a, Pp, cam, dets, ref, sup)
    aj.pedir({"lam": 0.9, "avion.alargamiento_max": 3.0, "avion.dist_max": 0.4, "ganancia": 1.5})   # lo que dejó otro objetivo
    det = vivo.aplicar(dets["rojo"], 1.0)
    assert Pp.forma_avion.alargamiento_max == 3.0 and ref.est.eu.rls.lam == 0.9
    aj.pedir({"detector": "avion"}, "tablero")
    det = vivo.aplicar(det, 2.0)
    assert det is dets["avion"] and aj.perfil_activo == "avion" and not aj.perfil_modificado
    assert {"exposicion_us": 8000} in cam.pedidos and cam.pedidos[-3:] != [] and {"ganancia": 4.0} in cam.pedidos
    assert Pp.forma_avion.alargamiento_max == 8.0 and Pp.forma_avion.dist_max == 0.18 and Pp.forma_avion.contraste_min == 12
    assert ref.est.eu.rls.lam == 0.80 and ref.est.ew.rls.lam == 0.80 and aj.valores["ganancia"] == 4.0


def test_perfil_al_partir_respeta_la_linea_de_comandos():
    """Al partir con la cámara, el perfil fija exposición y ganancia antes de abrirla; lo dado por comando manda."""
    import argparse
    import json
    import main as programa
    with open(programa.RUTA_PERFILES, encoding="utf-8") as f:
        perfil = json.load(f)["avion"]
    Pp = Parametros()
    nombre, excluir = programa.perfil_de_inicio(argparse.Namespace(detector="avion", exposicion=None, ganancia=None), Pp)
    assert nombre == "avion" and excluir == ()
    assert Pp.captura.exposicion_us == perfil["exposicion_us"] and Pp.captura.ganancia == perfil["ganancia"]
    Pp = Parametros()
    Pp.captura.exposicion_us = 4000                                           # --exposicion 4000
    nombre, excluir = programa.perfil_de_inicio(argparse.Namespace(detector="avion", exposicion=4000, ganancia=None), Pp)
    assert excluir == ("exposicion_us",) and Pp.captura.exposicion_us == 4000 and Pp.captura.ganancia == perfil["ganancia"]
    assert programa.perfil_de_inicio(argparse.Namespace(detector="brillo", exposicion=None, ganancia=None), Parametros())[0] is None


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
