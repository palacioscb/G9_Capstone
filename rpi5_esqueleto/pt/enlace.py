"""
enlace.py — CS-3: cliente del enlace RPi <-> ESP32 (USB, UART0 del ESP32).

Dos implementaciones con la misma interfaz:
  EnlaceNulo  no envía nada (PC, o pruebas de visión sin motores)
  EnlaceCLI   habla con el firmware PoC actual (pruebas/poc_drv8825), que usa
              comandos de texto: 'v <pasos/s>', 's', 'x', 'e 0/1', 'p'.

Además se deja implementado y probado el formato de TRAMA binaria del diseño
(delimitador + n.º de secuencia + CRC-16), para cuando el firmware lo adopte:
    AA 55 | seq (1 B) | tipo (1 B) | largo (1 B) | carga (largo B) | CRC-16 (2 B, LE)
    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) sobre seq..carga.
"""
import struct
import time

CABECERA = b"\xAA\x55"
TIPO_CONSIGNA_OMEGA = 0x01     # carga: 2 x float32 LE, ω pan y tilt [°/s]
TIPO_TELEMETRIA = 0x81


# ---------------------------------------------------------------------------
# Trama binaria (diseño)
# ---------------------------------------------------------------------------
def crc16_ccitt(datos: bytes, crc: int = 0xFFFF) -> int:
    for b in datos:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def armar_trama(seq: int, tipo: int, carga: bytes) -> bytes:
    cuerpo = bytes([seq & 0xFF, tipo & 0xFF, len(carga)]) + carga
    return CABECERA + cuerpo + struct.pack("<H", crc16_ccitt(cuerpo))


def leer_trama(buf: bytes):
    """Busca una trama válida en buf. Devuelve (seq, tipo, carga, resto) o (None, None, None, buf)."""
    i = buf.find(CABECERA)
    while i >= 0:
        if len(buf) < i + 5:
            return None, None, None, buf[i:]
        n = buf[i + 4]
        fin = i + 5 + n + 2
        if len(buf) < fin:
            return None, None, None, buf[i:]
        cuerpo = buf[i + 2:i + 5 + n]
        (crc,) = struct.unpack("<H", buf[fin - 2:fin])
        if crc16_ccitt(cuerpo) == crc:
            return cuerpo[0], cuerpo[1], cuerpo[3:], buf[fin:]
        i = buf.find(CABECERA, i + 1)            # CRC inválido: descartar y seguir
    return None, None, None, b""


def trama_consigna(seq, omega_pan, omega_tilt):
    return armar_trama(seq, TIPO_CONSIGNA_OMEGA, struct.pack("<ff", omega_pan, omega_tilt))


# ---------------------------------------------------------------------------
# Implementaciones
# ---------------------------------------------------------------------------
class EnlaceNulo:
    def __init__(self, *a, **k):
        self.enviados = 0

    def enviar_velocidad(self, pasos_s):
        self.enviados += 1

    def ok(self):
        return True

    def detener(self):
        pass

    def cerrar(self):
        pass


class EnlaceCLI:
    """Firmware PoC: texto por línea. Sólo mueve UN eje (el del banco)."""

    def __init__(self, pe, habilitar=False):
        import serial                           # python3-serial (apt) / pyserial (pip)
        self.pe = pe
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = pe.puerto, pe.baudios, 0
        # Evita que abrir el puerto reinicie el ESP32 (circuito de auto-reset por DTR/RTS)
        s.dtr = False
        s.rts = False
        s.open()
        self.s = s
        self.t_ult_tx = 0.0
        self.t_pendiente = None          # instante del comando aún sin respuesta
        self.rx = b""
        self.lineas = []                 # respuestas del firmware (diagnóstico)
        time.sleep(0.2)
        self.s.reset_input_buffer()
        if habilitar:
            self._tx("e 1")

    def _tx(self, texto):
        self.s.write((texto + "\n").encode())
        now = time.monotonic()
        self.t_ult_tx = now
        if self.t_pendiente is None:
            self.t_pendiente = now

    def _rx(self):
        datos = self.s.read(self.s.in_waiting or 1)
        if datos:
            self.rx += datos
            while b"\n" in self.rx:
                linea, self.rx = self.rx.split(b"\n", 1)
                self.lineas.append(linea.decode(errors="replace").strip())
                self.t_pendiente = None
            self.lineas = self.lineas[-50:]

    def enviar_velocidad(self, pasos_s):
        self._rx()
        if time.monotonic() - self.t_ult_tx >= self.pe.periodo_min_s:
            self._tx(f"v {pasos_s:.1f}")

    def ok(self):
        self._rx()
        return self.t_pendiente is None or (time.monotonic() - self.t_pendiente) < self.pe.watchdog_s

    def detener(self):
        try:
            self._tx("s")
            time.sleep(0.05)
            self._tx("x")               # STOP inmediato + driver deshabilitado
        except Exception:
            pass

    def cerrar(self):
        self.detener()
        self.s.close()
