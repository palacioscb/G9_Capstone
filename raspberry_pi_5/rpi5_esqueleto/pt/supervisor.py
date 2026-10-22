"""
supervisor.py — Supervisor de estados (SE-2 / SE-3), subconjunto para la PoC.

Estados según D19 (siete). En esta etapa se implementan las transiciones de
visión (BÚSQUEDA <-> SEGUIMIENTO <-> PÉRDIDA) y la entrada a MODO_SEGURO por
enlace caído. HOMING se omite mientras no haya finales de carrera ni AS5600.

Nota: el diseño fija un ciclo de 20 Hz con eventos enclavados; aquí se evalúa
una vez por cuadro, lo que es equivalente para la PoC (tasa de cámara > 20 Hz).
"""
from enum import Enum


class Estado(str, Enum):
    INICIALIZACION = "INICIALIZACION"
    HOMING = "HOMING"
    BUSQUEDA = "BUSQUEDA"
    SEGUIMIENTO = "SEGUIMIENTO"
    PERDIDA = "PERDIDA"
    MODO_SEGURO = "MODO_SEGURO"
    DETENIDO = "DETENIDO"


class Supervisor:
    def __init__(self, ps, con_actuadores=False):
        self.ps = ps
        self.con_actuadores = con_actuadores
        self.estado = Estado.INICIALIZACION
        self.validos_seguidos = 0
        self.t_ult_valida = None
        self.t_entrada = 0.0
        self.bitacora = []          # SE-6: (t, desde, hacia, causa)
        self.falla = None

    # ------------------------------------------------------------------
    def _ir(self, t, nuevo, causa):
        if nuevo != self.estado:
            self.bitacora.append((t, self.estado.value, nuevo.value, causa))
            self.estado = nuevo
            self.t_entrada = t

    def reconocer_falla(self, t):
        """SE-5: rearme sólo manual (D21)."""
        if self.estado == Estado.MODO_SEGURO:
            self._ir(t, Estado.DETENIDO, "reconocimiento del operador")

    # ------------------------------------------------------------------
    def paso(self, t, det_valida, confianza, enlace_ok=True, homing_ok=True, falla_actuador=False):
        """Una evaluación. t [s]; det_valida: bool; confianza: EP-5 en [0,1].
        homing_ok: el ESP32 entrega posición absoluta válida (AS5600 con telemetría).
        falla_actuador: el ESP32 reporta FALLA (supervisor local, CS-9)."""
        ps = self.ps
        if det_valida:
            self.validos_seguidos += 1
            self.t_ult_valida = t
        else:
            self.validos_seguidos = 0
        t_sin = (t - self.t_ult_valida) if self.t_ult_valida is not None else float("inf")

        # Prioridad: falla > pérdida > adquisición
        if not enlace_ok and self.estado not in (Estado.MODO_SEGURO, Estado.DETENIDO):
            self.falla = "enlace caído"
            self._ir(t, Estado.MODO_SEGURO, self.falla)
            return self.estado
        if falla_actuador and self.estado not in (Estado.MODO_SEGURO, Estado.DETENIDO):
            self.falla = "falla reportada por el ESP32"
            self._ir(t, Estado.MODO_SEGURO, self.falla)
            return self.estado

        e = self.estado
        if e == Estado.INICIALIZACION:
            self._ir(t, Estado.HOMING if self.con_actuadores else Estado.BUSQUEDA,
                     "autodiagnóstico OK" + ("" if self.con_actuadores else " (homing omitido: sin actuadores)"))
        elif e == Estado.HOMING:
            #* Con AS5600 absolutos el homing es leer la posición: basta con telemetría válida.
            #todo Con finales de carrera, agregar la búsqueda de referencia mecánica.
            if homing_ok:
                self._ir(t, Estado.BUSQUEDA, "posición absoluta disponible (AS5600)")
        elif e == Estado.BUSQUEDA:
            if self.validos_seguidos >= ps.n_adquisicion:
                self._ir(t, Estado.SEGUIMIENTO, f"objetivo válido {ps.n_adquisicion} cuadros")
        elif e == Estado.SEGUIMIENTO:
            if t_sin > ps.t_perdida_s:
                self._ir(t, Estado.PERDIDA, f"sin detección {t_sin*1e3:.0f} ms")
            elif confianza < ps.conf_min:
                self._ir(t, Estado.PERDIDA, f"confianza {confianza:.2f}")
        elif e == Estado.PERDIDA:
            if self.validos_seguidos >= ps.n_adquisicion:
                self._ir(t, Estado.SEGUIMIENTO, "reaparición validada")
            elif t - self.t_entrada > ps.t_reintento_s:
                self._ir(t, Estado.BUSQUEDA, "sin reaparición")
        return self.estado
