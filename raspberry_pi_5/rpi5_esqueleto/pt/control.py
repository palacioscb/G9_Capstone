"""
control.py — CS-1 (error de centrado -> ángulo) y CS-2 (lazo externo) del lado RPi.
"""
#* Reparto del lazo (diagrama de control del informe):
#*   ESP32 (1 kHz):  e_θ = θ_ref − θ_med (AS5600)  ->  PI de pid_t_2  ->  ω_cmd  ->  STEP/DIR
#*   RPi (80 fps):   cámara -> (u, w) -> dirección del objetivo en el MUNDO -> predicción -> θ_ref
#*
#* La "ley de control" del lazo externo queda reducida a entregar la referencia correcta:
#*     θ_ref = dirección absoluta del objetivo, predicha al instante en que el ESP32 la aplica.
#* Es decir: si el objetivo está a Δθ del centro de la imagen, el eje debe ir a θ_med + Δθ.
#* La corrección "más rápida con más error" la hace el PI del ESP32 (ω_cmd = Kp·e + Ki∫e),
#* con los techos de ω_max y α_max y la cota de estabilidad Kp < b/M de pid_t_2.
#*
#* Por qué no sumar Δθ a la posición actual al recibir la consigna: entre la exposición y la
#* llegada de la consigna (≈ 18 ms) la plataforma ya se movió, y ese movimiento se contaría
#* dos veces. Por eso se usa θ_med en el instante de EXPOSICIÓN (telemetría interpolada) y
#* la composición exacta de rotaciones (geometria.direccion_mundo).
#*
#* ω_ff (feedforward): velocidad del objetivo estimada por el RLS. El ESP32 la suma a la
#* salida del PI y extrapola la referencia entre cuadros. Con P.lazo.usar_feedforward = False
#* queda el PI puro de pid_t_2 (para validar el modelo); con True mejora el seguimiento.
import math
from dataclasses import dataclass

from .prediccion import Estimador2D


@dataclass
class MedicionMundo:
    pan: float          # dirección del objetivo en el mundo [°]
    tilt: float
    valida: bool


def _lim(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class ReferenciaVisual:
    def __init__(self, P, geo):
        self.P, self.geo = P, geo
        #* Estimador en ÁNGULOS DEL MUNDO: el objetivo se mueve en el mundo, no en la imagen
        #* (en la imagen su movimiento incluye el de la plataforma, que depende del control).
        self.est = Estimador2D(P.prediccion, sigma_min=P.prediccion.sigma_min_deg)
        self.ref_retencion = None        # última referencia enviada (para retener)

    def reiniciar(self):
        self.est.reiniciar()

    def actualizar(self, t_exp_ns, det, pose, t_aplic_ns):
        """det: Resultado del detector. pose: (pan, tilt) medidos en t_exp, o None.
        t_aplic_ns: instante en que el ESP32 aplicará la consigna (horizonte de predicción)."""
        valida = bool(det.valido) and pose is not None and not math.isnan(det.u)
        if valida:
            pan, tilt = self.geo.direccion_mundo(det.u, det.w, pose[0], pose[1])
        else:
            pan, tilt = float("nan"), float("nan")
        e = self.est.actualizar_xy(t_exp_ns * 1e-9, valida, pan, tilt, t_aplic_ns * 1e-9)
        return MedicionMundo(pan, tilt, valida), e

    def consigna(self, siguiendo, e, pose):
        """(ref_pan, ref_tilt, ωff_pan, ωff_tilt) a enviar, o None si todavía no hay qué enviar.

        siguiendo: supervisor en SEGUIMIENTO. Fuera de él se RETIENE la última referencia
        (ω_ff = 0): en PÉRDIDA la plataforma queda apuntando donde vio el objetivo por última vez.
        """
        #todo Patrón de búsqueda en BÚSQUEDA (barrido), como pide el diagrama CS-5.
        L = self.P.lazo
        if self.ref_retencion is None and pose is not None:
            self.ref_retencion = (pose[0], pose[1])
        if siguiendo and e.confianza > 0 and not (math.isnan(e.u) or math.isnan(e.w)):
            rp = _lim(e.u, *L.lim_pan_deg)
            rt = _lim(e.w, *L.lim_tilt_deg)
            wp = wt = 0.0
            if L.usar_feedforward and not (math.isnan(e.vu) or math.isnan(e.vw)):
                wp = _lim(e.vu, -L.omega_ff_max, L.omega_ff_max)
                wt = _lim(e.vw, -L.omega_ff_max, L.omega_ff_max)
                if rp != e.u:
                    wp = 0.0                     #! en el límite no se empuja con feedforward
                if rt != e.w:
                    wt = 0.0
            self.ref_retencion = (rp, rt)
            return rp, rt, wp, wt
        if self.ref_retencion is None:
            return None
        return self.ref_retencion[0], self.ref_retencion[1], 0.0, 0.0


# ---------------------------------------------------------------------------
# Heredado (enlace de texto de la PoC, un eje en velocidad)
# ---------------------------------------------------------------------------
class ControlNulo:
    def calcular(self, dpan, dtilt, dt):
        return 0.0, 0.0


class ControlProporcionalDemo:
    def __init__(self, kp=2.0, omega_max=30.0):
        self.kp, self.omega_max = kp, omega_max       # [1/s], [°/s]

    def calcular(self, dpan, dtilt, dt):
        sat = lambda x: max(-self.omega_max, min(self.omega_max, x))
        return sat(self.kp * dpan), sat(self.kp * dtilt)


def omega_a_pasos(omega_deg_s, pa):
    """[°/s] -> [micropasos/s] para el firmware PoC, con saturación de seguridad."""
    paso = pa.paso_completo_deg / pa.micropasos
    v = omega_deg_s / paso
    return max(-pa.v_max_pasos_s, min(pa.v_max_pasos_s, v))
