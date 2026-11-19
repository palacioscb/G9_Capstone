"""
enlace.py — CS-3: cliente del enlace RPi <-> ESP32 (USB, UART0 del ESP32).

Implementaciones con la misma interfaz (ok, listo, angulos_en, enviar_consigna, retener, cerrar):
  EnlaceNulo      no envía nada; la cámara se considera fija en (0°, 0°)
  EnlaceTrama     firmware esp32_pan_tilt: tramas binarias con CRC a 921 600 baudios
  EnlaceSimulado  ESP32 + ejes simulados (modelo de pid_t_2 y el mismo PI del firmware),
                  para correr el lazo cerrado completo en el PC
  EnlaceCLI       firmware de la prueba de concepto (texto, un eje). Sólo por compatibilidad.

Formato de trama (igual en esp32_pan_tilt/include/protocolo.h):
    AA 55 | seq (1 B) | tipo (1 B) | largo (1 B) | carga (largo B) | CRC-16 (2 B, LE)
    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) sobre seq..carga. Little-endian.
"""
#* El dato clave que vuelve del ESP32 es θ_med de cada eje. La RPi lo necesita en el INSTANTE
#* DE EXPOSICIÓN de cada cuadro para ubicar el objetivo en el mundo (geometria.direccion_mundo).
#* La telemetría llega a 200 Hz con la marca de tiempo del ESP32 (micros); el desfase entre
#* ese reloj y CLOCK_MONOTONIC de la RPi se estima con el mínimo de (t_llegada − t_ESP32),
#* que corresponde a la trama que viajó con menos retardo.
import bisect
import collections
import re
import struct
import threading
import time
from dataclasses import dataclass

CABECERA = b"\xAA\x55"
TIPO_CONSIGNA_OMEGA = 0x01     # heredado: 2 x float32 LE, ω pan y tilt [°/s]
TIPO_CONSIGNA_THETA = 0x02     # 4 x float32 LE: θref pan, θref tilt [°], ωff pan, ωff tilt [°/s]
TIPO_COMANDO = 0x03            # 1 B: código
TIPO_TELEMETRIA = 0x81         # 31 B, ver leer_telemetria()

CMD_HABILITAR, CMD_DESHABILITAR, CMD_RETENER, CMD_CERO = 1, 2, 3, 4

#* Estados del ESP32 (mismos números que el enum Estado de main.cpp)
ESTADOS_ESP32 = {0: "DESHABILITADO", 1: "RETENCION", 2: "LAZO", 3: "ABIERTO", 4: "FALLA"}
FALLAS_ESP32 = {0x01: "nFAULT pan", 0x02: "nFAULT tilt", 0x04: "sensor pan", 0x08: "sensor tilt",
                0x10: "límite o final de carrera pan", 0x20: "límite o final de carrera tilt", 0x40: "pérdida de pasos", 0x80: "watchdog"}


# ---------------------------------------------------------------------------
# Trama binaria
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
    # un 0xAA suelto al final puede ser el comienzo de la próxima trama: se conserva
    return None, None, None, (buf[-1:] if buf.endswith(b"\xAA") else b"")


def trama_consigna(seq, omega_pan, omega_tilt):
    return armar_trama(seq, TIPO_CONSIGNA_OMEGA, struct.pack("<ff", omega_pan, omega_tilt))


def trama_consigna_theta(seq, ref_pan, ref_tilt, wff_pan=0.0, wff_tilt=0.0):
    return armar_trama(seq, TIPO_CONSIGNA_THETA, struct.pack("<ffff", ref_pan, ref_tilt, wff_pan, wff_tilt))


def trama_comando(seq, codigo):
    return armar_trama(seq, TIPO_COMANDO, bytes([codigo & 0xFF]))


@dataclass
class Telemetria:
    t_us: int                 # micros() del ESP32 (32 bits, se desborda a los 71 min)
    pan_med: float            # θ_med [°]
    tilt_med: float
    pan_ref: float            # referencia aplicada (extrapolada y limitada) [°]
    tilt_ref: float
    w_pan: float              # ω_cmd [°/s]
    w_tilt: float
    estado: int
    fallas: int
    seq_consigna: int
    t_rpi_ns: int = 0         # instante de la medición en CLOCK_MONOTONIC de la RPi (estimado)

    @property
    def nombre_estado(self):
        return ESTADOS_ESP32.get(self.estado, str(self.estado))

    @property
    def texto_fallas(self):
        return ", ".join(n for b, n in FALLAS_ESP32.items() if self.fallas & b) or "-"


FORMATO_TELEMETRIA = "<IffffffBBB"         # 31 B


def leer_telemetria(carga: bytes) -> Telemetria:
    return Telemetria(*struct.unpack(FORMATO_TELEMETRIA, carga))


def trama_telemetria(seq, tel: Telemetria) -> bytes:
    """Sólo para pruebas: arma una telemetría como lo haría el ESP32."""
    return armar_trama(seq, TIPO_TELEMETRIA, struct.pack(
        FORMATO_TELEMETRIA, tel.t_us & 0xFFFFFFFF, tel.pan_med, tel.tilt_med, tel.pan_ref,
        tel.tilt_ref, tel.w_pan, tel.w_tilt, tel.estado, tel.fallas, tel.seq_consigna))


# ---------------------------------------------------------------------------
#* Historial de ángulos medidos, para interpolar θ_med en el instante de exposición
# ---------------------------------------------------------------------------
class HistoriaAngulos:
    def __init__(self, max_muestras=2000, extrapolacion_max_s=0.010):
        self.t, self.pan, self.tilt = [], [], []
        self.max = max_muestras
        self.extrap_ns = int(extrapolacion_max_s * 1e9)
        self.lock = threading.Lock()

    def agregar(self, t_ns, pan, tilt):
        with self.lock:
            if self.t and t_ns <= self.t[-1]:
                return                                   # fuera de orden: se descarta
            self.t.append(t_ns); self.pan.append(pan); self.tilt.append(tilt)
            if len(self.t) > 2 * self.max:
                del self.t[:self.max], self.pan[:self.max], self.tilt[:self.max]

    def interpolar(self, t_ns):
        """(pan, tilt) en t_ns, o None si t_ns cae fuera del historial (± extrapolación corta)."""
        with self.lock:
            n = len(self.t)
            if n == 0:
                return None
            if n == 1:
                return (self.pan[0], self.tilt[0]) if abs(t_ns - self.t[0]) <= self.extrap_ns else None
            if t_ns < self.t[0] - self.extrap_ns or t_ns > self.t[-1] + self.extrap_ns:
                return None
            i = bisect.bisect_left(self.t, t_ns)
            i = min(max(i, 1), n - 1)
            t0, t1 = self.t[i - 1], self.t[i]
            a = (t_ns - t0) / (t1 - t0)
            return (self.pan[i - 1] + a * (self.pan[i] - self.pan[i - 1]),
                    self.tilt[i - 1] + a * (self.tilt[i] - self.tilt[i - 1]))

    def ultimo(self):
        with self.lock:
            return (self.t[-1], self.pan[-1], self.tilt[-1]) if self.t else None


class RelojESP32:
    """Lleva micros() del ESP32 (32 bits) a CLOCK_MONOTONIC de la RPi [ns]."""

    def __init__(self, ventana=400):
        self.ultimo_us = None
        self.vueltas = 0
        self.reinicios = 0
        self.desfases = collections.deque(maxlen=ventana)   # ~2 s de telemetría a 200 Hz

    def a_rpi(self, t_us, t_llegada_ns):
        if self.ultimo_us is not None and t_us < self.ultimo_us:
            if (self.ultimo_us - t_us) > 2 ** 31:
                self.vueltas += 1                            # desborde de micros()
            else:
                #! micros() volvió atrás: el ESP32 se reinició. Se rehace la estimación.
                self.vueltas = 0
                self.desfases.clear()
                self.reinicios += 1
        self.ultimo_us = t_us
        t_esp_ns = (self.vueltas * 2 ** 32 + t_us) * 1000
        self.desfases.append(t_llegada_ns - t_esp_ns)
        return t_esp_ns + min(self.desfases)


# ---------------------------------------------------------------------------
# Implementaciones
# ---------------------------------------------------------------------------
#* Tiempo de silencio antes de escribir en la consola de texto del ESP32. El firmware ignora
#* el texto en el UART del enlace mientras éste está "activo", es decir, hasta ENLACE_ACTIVO_MS
#* (1000 ms, config.h) después de la última trama válida. Se deja un margen.
SILENCIO_CONSOLA_S = 1.15
#* Líneas del firmware que informan ganancias: "#  pan  med=... Kp=3.00 Ki=2.25 ..." (comando p)
#* y "# pan: Kp=5.000 1/s  Ki=6.250 1/s^2" (comando k).
_RE_GANANCIAS = re.compile(r"\b(pan|tilt)\b.*?Kp=([0-9.]+)(?:\s*1/s)?\s+Ki=([0-9.]+)")


class _SinMando:
    """Órdenes del tablero (pt/mando.py) en los enlaces sin ESP32 real: se rechazan con un texto."""
    puede_mover = False
    en_consola = False
    consola_pendiente = False
    ganancias = {}
    _MOTIVO = "este modo de enlace no tiene ESP32"

    def atender(self):
        pass

    def latido(self):
        pass

    def habilitar(self):
        return False, self._MOTIVO

    def deshabilitar(self):
        return False, self._MOTIVO

    def rearmar(self):
        return False, self._MOTIVO

    def consola(self, linea, espera_s=0.6):
        return False, self._MOTIVO

    def fijar_ganancias(self, eje, kp, ki):
        return False, self._MOTIVO


class EnlaceNulo(_SinMando):
    """Sin actuadores: la cámara se considera fija en (0°, 0°) y no se envía nada."""

    def __init__(self, *a, **k):
        self.enviados = 0
        self.telemetria = None

    def ok(self):
        return True

    def listo(self):
        return True

    def en_falla(self):
        return False

    def angulos_en(self, t_ns):
        return 0.0, 0.0

    def enviar_consigna(self, ref_pan, ref_tilt, wff_pan=0.0, wff_tilt=0.0, t_ns=None):
        self.enviados += 1

    def enviar_velocidad(self, pasos_s):                 # interfaz antigua
        self.enviados += 1

    def retener(self):
        pass

    def detener(self):
        pass

    def cerrar(self):
        pass


class EnlaceTrama:
    """Firmware esp32_pan_tilt. Un hilo lee la telemetría y la marca con su hora de llegada."""
    #* Mando desde el tablero (19-11-2026), sin tocar el firmware:
    #*   - habilitar / deshabilitar / rearmar usan los comandos binarios que ya existían;
    #*   - ganancias, cero, estado y prueba de signos usan la CONSOLA DE TEXTO del firmware.
    #*     Como el firmware la ignora con el enlace activo, se abre una "ventana de consola":
    #*     la RPi calla SILENCIO_CONSOLA_S, escribe la línea, espera la respuesta y retoma.
    #*     Durante la ventana no hay consignas ni telemetría (ok() lo da por esperado) y el
    #*     ESP32, si venía en LAZO, pasa solo a RETENCIÓN por su watchdog (100 ms).

    def __init__(self, pe, habilitar=False, puerto_serie=None, silencio_consola_s=SILENCIO_CONSOLA_S):
        #! Sin habilitar = sólo observar: se manda DESHABILITAR (no RETENER) y NUNCA consignas.
        #! El ESP32 conserva su estado entre corridas (abrir el puerto no lo reinicia), así que
        #! un "retener" podía dejarlo energizado y siguiendo consignas (revisión del 21-10).
        self.puede_mover = bool(habilitar)
        self._visto_activo = False
        self._t_latido = 0.0
        if puerto_serie is None:
            import serial                                # python3-serial (apt) / pyserial (pip)
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = pe.puerto, pe.baudios, 0.005
            s.dtr = False                                #! no reiniciar el ESP32 al abrir el puerto
            s.rts = False
            s.open()
            time.sleep(0.2)
            s.reset_input_buffer()
            puerto_serie = s
        self.s, self.pe = puerto_serie, pe
        self.seq = 0
        self.reloj = RelojESP32()
        self.historia = HistoriaAngulos()
        self.telemetria = None
        self.t_ult_tel = None                            # time.monotonic() de la última telemetría
        self.textos = collections.deque(maxlen=50)      # líneas '#' del firmware (diagnóstico)
        #* MON-2: últimos 2 min de telemetría a 200 Hz (variables rápidas del lazo interno),
        #* exportables desde el monitor (/telemetria.csv). ~24 000 tuplas, unos 3 MB.
        self.historial_tel = collections.deque(maxlen=200 * 120)
        self.tramas_rx = 0
        self._lock_tx = threading.Lock()
        self._t_ult_tx = 0.0                             # time.monotonic() de la última trama enviada
        self._silencio = float(silencio_consola_s)
        self._cola_cons = collections.deque()            # líneas de consola pedidas: (línea, espera)
        self._cons = None                                # ventana de consola en curso
        self.t_fin_consola = 0.0
        self.ganancias = {}                              # {"pan": (Kp, Ki), "tilt": (Kp, Ki)}, leídas del firmware
        self._corriendo = True
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()
        #* La telemetría sólo sale mientras el ESP32 recibe tramas: un primer comando la activa.
        self.comando(CMD_HABILITAR if habilitar else CMD_DESHABILITAR)
        t_lim = time.monotonic() + 1.0
        while self.telemetria is None and time.monotonic() < t_lim:
            time.sleep(0.01)

    # ---- recepción -------------------------------------------------------------------------
    def _leer(self):
        buf = b""
        while self._corriendo:
            try:
                datos = self.s.read(self.s.in_waiting or 1)
            except Exception:
                break
            if not datos:
                continue
            t_llegada = time.monotonic_ns()
            buf += datos
            buf = self._consumir(buf, t_llegada)

    def _consumir(self, buf, t_llegada):
        while buf:
            if buf[0] != 0xAA:                           # texto: hasta el próximo 0xAA
                j = buf.find(b"\xAA")
                texto, buf = (buf, b"") if j < 0 else (buf[:j], buf[j:])
                for linea in texto.decode(errors="replace").splitlines():
                    if linea.strip():
                        self.textos.append(linea.strip())
                        m = _RE_GANANCIAS.search(linea)
                        if m:
                            self.ganancias = {**self.ganancias, m.group(1): (float(m.group(2)), float(m.group(3)))}
                continue
            if len(buf) < 2:
                return buf
            if buf[1] != 0x55 or (len(buf) >= 5 and buf[4] > 64):   # no es cabecera, o largo imposible
                buf = buf[1:]
                continue
            seq, tipo, carga, resto = leer_trama(buf)
            if seq is None:
                if len(resto) == len(buf):              # trama incompleta: esperar más bytes
                    return buf
                buf = resto
                continue
            buf = resto
            if tipo == TIPO_TELEMETRIA and len(carga) == 31:
                tel = leer_telemetria(carga)
                tel.t_rpi_ns = self.reloj.a_rpi(tel.t_us, t_llegada)
                self.historia.agregar(tel.t_rpi_ns, tel.pan_med, tel.tilt_med)
                self.historial_tel.append((tel.t_rpi_ns, tel.t_us, tel.pan_med, tel.tilt_med, tel.pan_ref,
                                           tel.tilt_ref, tel.w_pan, tel.w_tilt, tel.estado, tel.fallas,
                                           tel.seq_consigna))
                if tel.estado in (1, 2):
                    self._visto_activo = True
                self.telemetria = tel
                self.t_ult_tel = time.monotonic()
                self.tramas_rx += 1
        return buf

    # ---- envío ---------------------------------------------------------------------------
    def _tx(self, trama):
        with self._lock_tx:
            self.s.write(trama)
            self.seq = (self.seq + 1) & 0xFF
            self._t_ult_tx = time.monotonic()

    def comando(self, codigo):
        self._tx(trama_comando(self.seq, codigo))

    # ---- mando desde el tablero ------------------------------------------------------------
    def habilitar(self):
        """Energiza los drivers (el firmware rearma sus fallas si la causa ya no está)."""
        self.puede_mover, self._visto_activo = True, False
        self.comando(CMD_HABILITAR)
        return True, "habilitar enviado"

    def deshabilitar(self):
        """Corta los motores: los ejes quedan sueltos y la RPi deja de mandar consignas."""
        self.puede_mover, self._visto_activo = False, False
        self.comando(CMD_DESHABILITAR)
        return True, "motores cortados"

    def rearmar(self):
        """Tras una falla o un reinicio del ESP32: vuelve a habilitar si la RPi lo estaba moviendo."""
        self._visto_activo = False
        if self.puede_mover:
            self.comando(CMD_HABILITAR)
        return True, "rearme enviado" if self.puede_mover else "sin motores que rearmar (están cortados)"

    @property
    def en_consola(self):
        """Ventana de consola en curso: el enlace binario calla. Con líneas en cola pero sin
        ventana abierta (pausa entre dos líneas) el enlace funciona normal."""
        return self._cons is not None

    @property
    def consola_pendiente(self):
        """Hay una línea en curso o esperando turno (lo que muestra el tablero)."""
        return self._cons is not None or bool(self._cola_cons)

    def consola(self, linea, espera_s=0.6):
        """Pide escribir una línea en la consola de texto del firmware (ver la nota de la clase).
        No bloquea: la ventana avanza con atender(), que el lazo llama en cada cuadro."""
        linea = str(linea).strip()
        if not linea or len(linea) > 40 or any(not 0x20 <= ord(ch) <= 0x7E for ch in linea):
            return False, "línea de consola inválida"
        if len(self._cola_cons) >= 4:
            return False, "consola ocupada: esperar a que termine lo anterior"
        self._cola_cons.append((linea, float(espera_s)))
        return True, f"«{linea}» en cola: el enlace se pausa {self._silencio + float(espera_s):.1f} s"

    def fijar_ganancias(self, eje, kp, ki):
        return self.consola(f"k {eje} {kp:.4f} {ki:.4f}", 0.4)

    def atender(self):
        """Avanza la ventana de consola. Barato si no hay nada pendiente."""
        if self._cons is None:
            if not self._cola_cons:
                return
            #! Entre dos líneas seguidas el enlace se retoma un momento. El firmware deshabilita
            #! los drivers si pasa 5 s en RETENCIÓN por watchdog sin consignas
            #! (DESHABILITAR_SIN_ENLACE_MS): cada ventana debe durar bastante menos que eso.
            if time.monotonic() - self.t_fin_consola < 0.3:
                return
            linea, espera = self._cola_cons.popleft()
            self._cons = {"linea": linea, "espera": espera, "fase": "silencio", "t": time.monotonic()}
        c, ahora = self._cons, time.monotonic()
        if c["fase"] == "silencio":
            if ahora - self._t_ult_tx >= self._silencio:
                with self._lock_tx:
                    self.s.write((c["linea"] + "\n").encode("ascii"))
                self.textos.append("> " + c["linea"])
                c["fase"], c["t"] = "respuesta", ahora
        elif ahora - c["t"] >= c["espera"]:
            self._cons = None
            self.t_fin_consola = ahora

    def latido(self):
        """Mantiene vivo el enlace cuando no hay consigna que mandar (MODO_SEGURO, arranque): el
        ESP32 sólo envía telemetría mientras recibe tramas. No mueve nada: RETENER no actúa en
        FALLA ni en DESHABILITADO."""
        self.atender()
        if self.en_consola:
            return
        ahora = time.monotonic()
        if ahora - self._t_latido >= 0.2:
            self._t_latido = ahora
            self.comando(CMD_RETENER if self.puede_mover else CMD_DESHABILITAR)

    def enviar_consigna(self, ref_pan, ref_tilt, wff_pan=0.0, wff_tilt=0.0, t_ns=None):
        self.atender()
        if self.en_consola:
            return                                       #! ventana de consola: el enlace calla
        if not self.puede_mover:
            #* Modo observar: en vez de la consigna, un "deshabilitar" cada 0,2 s, que además
            #* mantiene viva la telemetría (el ESP32 la envía mientras recibe tramas).
            ahora = time.monotonic()
            if ahora - self._t_latido >= 0.2:
                self._t_latido = ahora
                self.comando(CMD_DESHABILITAR)
            return
        self._tx(trama_consigna_theta(self.seq, ref_pan, ref_tilt, wff_pan, wff_tilt))

    # ---- estado ----------------------------------------------------------------------------
    def ok(self):
        if self.t_ult_tel is None:
            return False
        #* En la ventana de consola (y un instante después) el silencio de la telemetría es esperado.
        if self.en_consola or (time.monotonic() - self.t_fin_consola) < 0.5:
            return True
        return (time.monotonic() - self.t_ult_tel) < self.pe.watchdog_s

    def listo(self):
        """Homing: con AS5600 absolutos, basta con telemetría válida y sin falla. Si la RPi
        debe mover los ejes, además el ESP32 tiene que estar energizado (RETENCION o LAZO)."""
        if not (self.ok() and not self.en_falla()):
            return False
        return (not self.puede_mover) or self.telemetria.estado in (1, 2)

    def en_falla(self):
        """FALLA del ESP32, o (si se le estaba mandando mover) volvió a DESHABILITADO: se
        reinició o alguien lo deshabilitó por consola. En ambos casos: MODO_SEGURO."""
        if self.telemetria is None:
            return False
        if self.telemetria.estado == 4:
            return True
        return self.puede_mover and self._visto_activo and self.telemetria.estado == 0

    def angulos_en(self, t_ns):
        return self.historia.interpolar(t_ns)

    def retener(self):
        if self.puede_mover:                     # en modo observar no hay nada que retener
            self.comando(CMD_RETENER)

    def detener(self):
        try:
            self.comando(CMD_RETENER)
            time.sleep(0.05)
            self.comando(CMD_DESHABILITAR)
        except Exception:
            pass

    def cerrar(self):
        self.detener()
        time.sleep(0.05)
        self._corriendo = False
        self._hilo.join(timeout=0.5)
        try:
            self.s.close()
        except Exception:
            pass


class EnlaceSimulado(_SinMando):
    """ESP32 y ejes simulados: planta de pid_t_2 + el PI del firmware a 1 kHz.
    """
    puede_mover = True                                   # el simulado siempre está "habilitado"
    _MOTIVO = "enlace simulado: no hay ESP32 que mandar"

    @property
    def ganancias(self):
        return {n: (float(c.kp), float(c.ki)) for n, c in zip(("pan", "tilt"), self.ctl)}

    def fijar_ganancias(self, eje, kp, ki):
        """Cambia el PI simulado en caliente (para probar el tablero en el PC)."""
        c = self.ctl[0 if eje == "pan" else 1]
        c.kp, c.ki = float(kp), float(ki)
        return True, f"PI simulado de {eje}: Kp = {kp:g}, Ki = {ki:g}"
    #* Permite correr main.py en lazo cerrado en el PC: la fuente sintética dibuja el objetivo
    #* según la pose simulada de la plataforma (pose_en), y la pose responde a las consignas.
    #! Ganancias y límites deben coincidir con esp32_pan_tilt/include/config.h.

    def __init__(self, P, kp=(3.0, 10.0), ki=(2.25, 25.0), Ts=1e-3, extrapolacion_max_s=0.050):
        import numpy as np
        from .modelo_eje import PAN, TILT, ControlEje, discretizar, RESOLUCION_AS5600_DEG
        self.np = np
        self.Ts, self.Ts_ns = Ts, int(round(Ts * 1e9))
        self.res = RESOLUCION_AS5600_DEG
        self.d_enlace_ns = int(P.lazo.retardo_enlace_s * 1e9)
        self.ext_ns = int(extrapolacion_max_s * 1e9)
        self.lims = (P.lazo.lim_pan_deg, P.lazo.lim_tilt_deg)
        self.planta = [discretizar(PAN, Ts), discretizar(TILT, Ts)]
        self.ctl = [ControlEje(kp[i], ki[i]) for i in range(2)]
        self.x = [np.zeros(3), np.zeros(3)]
        self.ref = [0.0, 0.0]
        self.wff = [0.0, 0.0]
        self.t_ref_ns = 0
        self.t_ns = None
        self.cola = collections.deque()
        self.historia = HistoriaAngulos(max_muestras=5000, extrapolacion_max_s=0.002)
        self.historial_tel = collections.deque(maxlen=200 * 120)   # como EnlaceTrama (200 Hz)
        self._n_paso = 0
        self._r_lim = [0.0, 0.0]

    def _paso(self):
        t = self.t_ns
        while self.cola and self.cola[0][0] <= t:
            _, ref, wff = self.cola.popleft()
            self.ref, self.wff, self.t_ref_ns = list(ref), list(wff), t
        edad = (t - self.t_ref_ns) * 1e-9
        for i in range(2):
            ext = edad < self.ext_ns * 1e-9
            r = self.ref[i] + self.wff[i] * (edad if ext else self.ext_ns * 1e-9)
            ff = self.wff[i] if ext else 0.0
            lo, hi = self.lims[i]
            r_lim = min(max(r, lo), hi)
            self._r_lim[i] = r_lim
            if r_lim != r:
                ff = 0.0
            med = round(self.x[i][0] / self.res) * self.res
            w = self.ctl[i].paso(r_lim, med, ff, self.Ts)
            Ad, Bd = self.planta[i]
            self.x[i] = Ad @ self.x[i] + Bd * w
        self.t_ns = t + self.Ts_ns
        self.historia.agregar(self.t_ns, float(self.x[0][0]), float(self.x[1][0]))
        self._n_paso += 1
        if self._n_paso % 5 == 0:                        # telemetría simulada a 200 Hz
            tel = self.telemetria
            self.historial_tel.append((tel.t_rpi_ns, tel.t_us, tel.pan_med, tel.tilt_med, tel.pan_ref,
                                       tel.tilt_ref, tel.w_pan, tel.w_tilt, tel.estado, tel.fallas, 0))

    @property
    def telemetria(self):
        """Lo que mandaría el ESP32 en este instante (para el registro y el monitor)."""
        if self.t_ns is None:
            return None
        med = [round(float(self.x[i][0]) / self.res) * self.res for i in range(2)]
        return Telemetria(int(self.t_ns // 1000) & 0xFFFFFFFF, med[0], med[1], self._r_lim[0], self._r_lim[1],
                          float(self.ctl[0].omega), float(self.ctl[1].omega), 2, 0, 0, t_rpi_ns=self.t_ns)

    def avanzar_hasta(self, t_ns):
        if self.t_ns is None:
            self.t_ns = t_ns - 50 * self.Ts_ns           # un poco de historia previa
            self.t_ref_ns = self.t_ns
            self.historia.agregar(self.t_ns, 0.0, 0.0)
        while self.t_ns < t_ns:
            self._paso()

    def pose_en(self, t_ns):
        """Pose verdadera de la plataforma en t_ns (para dibujar la escena sintética)."""
        self.avanzar_hasta(t_ns)
        return self.historia.interpolar(t_ns) or (float(self.x[0][0]), float(self.x[1][0]))

    def ok(self):
        return True

    def listo(self):
        return True

    def en_falla(self):
        return False

    def angulos_en(self, t_ns):
        return self.historia.interpolar(t_ns)

    def enviar_consigna(self, ref_pan, ref_tilt, wff_pan=0.0, wff_tilt=0.0, t_ns=None):
        t_ns = time.monotonic_ns() if t_ns is None else t_ns
        self.cola.append((t_ns + self.d_enlace_ns, (ref_pan, ref_tilt), (wff_pan, wff_tilt)))

    def retener(self):
        pass

    def detener(self):
        pass

    def cerrar(self):
        pass


class EnlaceCLI:
    """Firmware de la PoC (esp32_drv8825): texto por línea, UN eje, a 115 200 baudios.
    Se conserva sólo para pruebas con ese firmware; el lazo cerrado usa EnlaceTrama."""

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
