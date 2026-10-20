"""
control.py — CS-2: ley de control del lazo externo (INTERFAZ, no la ley definitiva).

La sintonía del controlador es responsabilidad de otro integrante. Aquí sólo se
fija la interfaz para que el resto del flujo funcione de punta a punta:

    calcular(dpan_deg, dtilt_deg, dt_s) -> (omega_pan, omega_tilt) [°/s]

  ControlNulo            no mueve nada (por defecto: seguro)
  ControlProporcionalDemo  ω = Kp·Δθ con saturación — SÓLO para probar el enlace
"""


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
    """[°/s] -> [micropasos/s] para el firmware, con saturación de seguridad."""
    paso = pa.paso_completo_deg / pa.micropasos
    v = omega_deg_s / paso
    return max(-pa.v_max_pasos_s, min(pa.v_max_pasos_s, v))
