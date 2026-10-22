"""
modelo_eje.py — modelo de un eje según pid_t_2.pdf, y réplica del PI del firmware.

Unidades: ángulos en grados y velocidades en °/s. La planta es lineal, así que M, b y Kt
pueden quedar en sus unidades SI (N·m·s²/rad, N·m·s/rad, N·m/rad): sólo importan los
cocientes Kt/M y b/M.
"""
#* Planta (pid_t_2 §1): la variable manipulada es la velocidad de comando ω_cmd (tren STEP)
#*     θ_cmd' = ω_cmd
#*     M θ'' + b θ' = Kt (θ_cmd − θ)          (resorte magnético del paso a paso, Kt = Th·Nr)
#*     G(s) = Θ(s)/Ω_cmd(s) = Kt / ( s (M s² + b s + Kt) )
#* Controlador (pid_t_2 §2): C(s) = Kp + Ki/s sobre e = θ_ref − θ, salida ω_cmd.
#* Lazo cerrado (pid_t_2 §4): T(s) = Kt (Kp s + Ki) / (M s⁴ + b s³ + Kt s² + Kt Kp s + Kt Ki)
#* La simulación es exacta entre muestras: con ω_cmd constante en cada período Ts (retención
#* de orden cero, como en el firmware), el estado [θ, θ', θ_cmd] se avanza con la exponencial
#* de matriz. Así se resuelven sin problema los modos rápidos (tilt: ωn ≈ 1367 rad/s).
#! ControlEje replica línea por línea src/control_eje.cpp del firmware. Si se cambia uno,
#! cambiar el otro: la comparación medición–modelo depende de que sean el mismo algoritmo.
import math
from dataclasses import dataclass

import numpy as np

RESOLUCION_AS5600_DEG = 360.0 / 4096.0


# ---------------------------------------------------------------------------
#* Parámetros físicos (pid_t_2 §6 y §7)
# ---------------------------------------------------------------------------
@dataclass
class ParametrosEje:
    M: float                 # inercia reflejada [kg·m²]
    Kt: float = 20.0         # rigidez de torque Th·Nr = 0,40·50 [N·m/rad]
    zeta: float = 0.05       #? amortiguamiento SUPUESTO (pid_t_2 §7): ζ ∈ [0,02; 0,15]

    @property
    def b(self):
        """Fricción viscosa equivalente, b = 2 ζ √(Kt M) (pid_t_2 ec. 8)."""
        return 2.0 * self.zeta * math.sqrt(self.Kt * self.M)

    @property
    def wn(self):
        return math.sqrt(self.Kt / self.M)

    @property
    def kp_max(self):
        """Techo de estabilidad de Routh (pid_t_2 ec. 7): Kp < b/M."""
        return self.b / self.M

    def ki_max(self, kp):
        """Cota de Ki de Routh: Ki < Kt Kp (b − M Kp) / b²."""
        return self.Kt * kp * (self.b - self.M * kp) / self.b ** 2


PAN = ParametrosEje(M=9.91e-4)       # pid_t_2 §6 (estimación previa al masado de la estructura)
TILT = ParametrosEje(M=1.07e-5)


def con_zeta(p: ParametrosEje, zeta: float) -> ParametrosEje:
    return ParametrosEje(M=p.M, Kt=p.Kt, zeta=zeta)


def polos_lazo_cerrado(p: ParametrosEje, kp: float, ki: float):
    """Raíces de M s⁴ + b s³ + Kt s² + Kt Kp s + Kt Ki (pid_t_2 §4)."""
    return np.roots([p.M, p.b, p.Kt, p.Kt * kp, p.Kt * ki])


# ---------------------------------------------------------------------------
#* Réplica del PI del firmware (src/control_eje.cpp)
# ---------------------------------------------------------------------------
def _sat(x, lim):
    return lim if x > lim else (-lim if x < -lim else x)


class ControlEje:
    def __init__(self, kp, ki, omega_max=180.0, alfa_max=300.0, banda_integral=5.0):
        self.kp, self.ki = kp, ki
        self.omega_max, self.alfa_max = omega_max, alfa_max
        self.banda_integral = banda_integral          # BANDA_INTEGRAL_DEG de config.h
        self.reiniciar()

    def reiniciar(self, omega_actual=0.0):
        self.integ, self.omega, self.error, self.limitado = 0.0, omega_actual, 0.0, False

    def paso(self, ref, med, omega_ff, dt):
        self.error = ref - med
        integ_cand = self.integ + self.ki * self.error * dt if abs(self.error) < self.banda_integral else self.integ
        # Tope de frenado sobre el término P (igual que el firmware): |P| ≤ √(2·α·|e|)
        p = self.kp * self.error
        v_fren = math.sqrt(2.0 * self.alfa_max * abs(self.error)) if math.isfinite(self.alfa_max) else math.inf
        if abs(p) > v_fren:
            p = v_fren if self.error > 0.0 else -v_fren
        u = omega_ff + p + integ_cand
        u_sat = _sat(u, self.omega_max)
        nuevo = self.omega + _sat(u_sat - self.omega, self.alfa_max * dt)
        exceso = u - nuevo
        self.limitado = abs(exceso) > 1e-6
        if not (self.limitado and exceso * self.error > 0.0):
            self.integ = integ_cand
        self.omega = nuevo
        return self.omega


# ---------------------------------------------------------------------------
#* Discretización exacta de la planta
# ---------------------------------------------------------------------------
def _expm(A):
    """Exponencial de matriz por escalamiento y cuadrado + Taylor (matrices chicas, sin SciPy)."""
    norma = np.linalg.norm(A, ord=np.inf)
    s = max(0, int(math.ceil(math.log2(norma))) + 1) if norma > 0 else 0
    As = A / (2 ** s)
    E = np.eye(A.shape[0])
    termino = np.eye(A.shape[0])
    for k in range(1, 20):
        termino = termino @ As / k
        E = E + termino
    for _ in range(s):
        E = E @ E
    return E


def discretizar(p: ParametrosEje, Ts: float):
    """x = [θ, θ', θ_cmd], u = ω_cmd constante en Ts. Devuelve (Ad, Bd)."""
    A = np.array([[0.0, 1.0, 0.0],
                  [-p.Kt / p.M, -p.b / p.M, p.Kt / p.M],
                  [0.0, 0.0, 0.0]])
    B = np.array([[0.0], [0.0], [1.0]])
    Maug = np.zeros((4, 4))
    Maug[:3, :3] = A * Ts
    Maug[:3, 3:] = B * Ts
    E = _expm(Maug)
    return E[:3, :3], E[:3, 3]


@dataclass
class Simulacion:
    t: np.ndarray        # [s]
    ref: np.ndarray      # [°]
    med: np.ndarray      # θ medido (con cuantización si se pidió) [°]
    theta: np.ndarray    # θ verdadero del eje [°]
    omega: np.ndarray    # ω_cmd [°/s]


def simular_escalon(p: ParametrosEje, kp, ki, escalon_deg, duracion_s=3.0, Ts=1e-3,
                    omega_max=180.0, alfa_max=300.0, cuantizar=True, theta0=0.0, banda_integral=5.0):
    """Respuesta al escalón del eje con el PI del firmware a 1/Ts Hz.

    omega_max = alfa_max = banda_integral = math.inf da el modelo LINEAL de pid_t_2.
    cuantizar=True redondea la medición a la resolución del AS5600 (0,088°).
    """
    Ad, Bd = discretizar(p, Ts)
    ctl = ControlEje(kp, ki, omega_max, alfa_max, banda_integral)
    n = int(round(duracion_s / Ts))
    x = np.array([theta0, 0.0, theta0])
    ref = theta0 + escalon_deg
    t = np.arange(n) * Ts
    th = np.empty(n); me = np.empty(n); om = np.empty(n)
    for k in range(n):
        med = x[0]
        if cuantizar:
            med = round(med / RESOLUCION_AS5600_DEG) * RESOLUCION_AS5600_DEG
        w = ctl.paso(ref, med, 0.0, Ts)
        th[k], me[k], om[k] = x[0], med, w
        x = Ad @ x + Bd * w
    return Simulacion(t, np.full(n, ref), me, th, om)


# ---------------------------------------------------------------------------
#* Métricas de la respuesta al escalón (para comparar medición y modelo)
# ---------------------------------------------------------------------------
def metricas_escalon(t, y, y0, yf_ref, banda=0.02, tol_min=0.0):
    """t_subida 10–90 %, sobrepaso [%], t_establecimiento (±banda del escalón), error final.

    tol_min [°]: la banda nunca es menor que esto. Con el sensor cuantizado (0,088°), una banda
    del 2 % de un escalón chico es menor que un paso del AS5600 y la medición nunca "entra".
    """
    paso = yf_ref - y0
    if tol_min > 0 and abs(paso) > 1e-9:
        banda = max(banda, tol_min / abs(paso))
    if abs(paso) < 1e-9:
        return {"t_subida_s": float("nan"), "sobrepaso_pct": float("nan"),
                "t_establecimiento_s": float("nan"), "error_final_deg": float("nan")}
    z = (np.asarray(y) - y0) / paso                      # respuesta normalizada
    def cruce(nivel):
        idx = np.nonzero(z >= nivel)[0]
        return t[idx[0]] if idx.size else float("nan")
    t10, t90 = cruce(0.1), cruce(0.9)
    fuera = np.nonzero(np.abs(z - 1.0) > banda)[0]
    t_est = t[fuera[-1] + 1] if fuera.size and fuera[-1] + 1 < len(t) else (t[0] if not fuera.size else float("nan"))
    n_cola = max(1, len(z) // 10)
    return {
        "t_subida_s": t90 - t10,
        "sobrepaso_pct": max(0.0, (z.max() - 1.0) * 100.0),
        "t_establecimiento_s": t_est,
        "error_final_deg": float(yf_ref - np.mean(np.asarray(y)[-n_cola:])),
    }


def ajustar_zeta(t, med, theta0, escalon, base: ParametrosEje, kp, ki, Ts=1e-3,
                 omega_max=180.0, alfa_max=300.0, zetas=None):
    """Busca el ζ que mejor reproduce una respuesta medida (mínimo error cuadrático).
    """
    #* Es la forma de "medir b" que pid_t_2 §7 deja pendiente: con M estimada, el ζ ajustado
    #* da b = 2ζ√(Kt M) y, con él, el techo real Kp < b/M.
    #? Supone que M es correcta. Si el ajuste es malo para todo ζ, el problema es M (o la
    #? planta tiene algo que el modelo no captura: juego en la correa, roce seco).
    zetas = np.geomspace(0.005, 0.5, 60) if zetas is None else zetas
    mejor = (float("inf"), None)
    for z in zetas:
        s = simular_escalon(con_zeta(base, z), kp, ki, escalon, duracion_s=t[-1] + Ts, Ts=Ts,
                            omega_max=omega_max, alfa_max=alfa_max, cuantizar=False, theta0=theta0)
        y = np.interp(t, s.t, s.theta)
        ecm = float(np.mean((y - med) ** 2))
        if ecm < mejor[0]:
            mejor = (ecm, z)
    return mejor[1], math.sqrt(mejor[0])
