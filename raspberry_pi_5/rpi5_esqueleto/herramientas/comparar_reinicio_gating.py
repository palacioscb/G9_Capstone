#!/usr/bin/env python3
"""
comparar_reinicio_gating.py — ¿qué hacer tras tres mediciones rechazadas seguidas por el gating?

Compara el reinicio original de pt/prediccion.py (V0) con alternativas: cada variante es una
subclase de EstimadorEje definida aquí. Desde el 17-11-2026 prediccion.py implementa V2
(reinicio_informado = True); V0 se reproduce apagando ese parámetro. Un eje, 80 cuadros/s, horizonte de
20 ms, ruido gaussiano en la medición. Los eventos ocurren en t = 1,5 s.

  V0  original: estimador nuevo desde cero (no queda "listo" -> confianza 0 -> PÉRDIDA)
  V4  original, pero sin caída de confianza: durante el recalentamiento entrega la última medición
  V1  reinicio en caliente: conserva la velocidad, reancla la posición y abre la covarianza
  V2  reinicio informado: el estimador nuevo parte con las mediciones rechazadas (ya "listo")
  V5  V2 con paciencia de 2 rechazos en vez de 3
  V3  sin reinicio: desde el segundo rechazo sigue la medición con memoria corta (λ = 0,5)

Uso (PC):  python3 herramientas/comparar_reinicio_gating.py        # ~1 min
"""
import copy
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from pt.parametros import Parametros            # noqa: E402
from pt.prediccion import EstimadorEje          # noqa: E402
from pt.supervisor import Supervisor, Estado    # noqa: E402

T, H = 1 / 80, 0.020
T_EV = 1.5                       # instante del evento [s]


def _fuera_del_umbral(est, t, y):
    """Misma condición de gating que EstimadorEje.actualizar()."""
    if est.t_ult_valida is None or not est.rls.listo() or est.sigma is None:
        return False
    return abs(y - est.rls.predecir(t)) > est.pp.k_gating * max(est.sigma, est.sigma_min)


class V0(EstimadorEje):
    nombre = "V0 original"

    def listo(self):
        return self.rls.listo()

    def posicion(self, t):
        return self.predecir(t)


class V4(V0):
    nombre = "V4 original, sin caída de confianza"

    def listo(self):
        return self.t_ult_valida is not None

    def posicion(self, t):
        return self.predecir(t) if self.rls.listo() else float(self.rls.th[0])


class V1(V0):
    nombre = "V1 reinicio en caliente"
    gracia = 0

    def actualizar(self, t, y, valido):
        r = self.rls
        if valido and self.gracia > 0:           # tras el reinicio, n cuadros sin gating
            self.gracia -= 1
            r.actualizar(t, y, True)
            self.t_ult_valida = t
            return "aceptada"
        if valido and _fuera_del_umbral(self, t, y):
            self.rechazos += 1
            if self.rechazos <= self.pp.max_rechazos:
                return "rechazada"
            r.th = r._A(t - r.t0) @ r.th         # origen en el instante actual
            r.t0 = t
            r.th[0] = y                          # la posición es la medición
            r.th[2:] = 0.0                       # la aceleración vieja no vale
            r.P = 1e4 * np.eye(r.n)
            r.P[0, 0] = 1.0
            self.rechazos, self.t_ult_valida, self.gracia = 0, t, r.n
            return "aceptada"
        return super().actualizar(t, y, valido)


class V2(V0):
    nombre = "V2 reinicio informado"

    def reiniciar(self):
        super().reiniciar()
        self.cola = []                           # mediciones rechazadas seguidas: (t, y)

    def actualizar(self, t, y, valido):
        if valido and _fuera_del_umbral(self, t, y):
            self.rechazos += 1
            self.cola.append((t, y))
            if self.rechazos <= self.pp.max_rechazos:
                return "rechazada"
            cola, sigma = self.cola, self.sigma
            self.reiniciar()
            for tc, yc in cola:                  # las rechazadas y la actual: queda "listo"
                self.rls.actualizar(tc, yc, True)
            self.sigma, self.t_ult_valida = sigma, t
            return "aceptada"
        if valido:
            self.cola = []
        return super().actualizar(t, y, valido)


class V5(V2):
    nombre = "V5 informado, 2 rechazos"

    def __init__(self, pp, sigma_min=None):
        pp = copy.copy(pp)
        pp.max_rechazos = 2
        super().__init__(pp, sigma_min)


class V3(V0):
    nombre = "V3 sin reinicio, olvido variable"
    LAM_RAPIDO = 0.5

    def actualizar(self, t, y, valido):
        r = self.rls
        if valido and _fuera_del_umbral(self, t, y):
            self.rechazos += 1
            if self.rechazos <= 1:               # el primero puede ser un aberrante: se ignora
                return "rechazada"
            r.lam = self.LAM_RAPIDO              # desde el segundo: memoria corta y se sigue la medición
            r.actualizar(t, y, True)
            self.t_ult_valida = t
            return "aceptada"
        if valido:
            r.lam = min(self.pp.lam, r.lam + 0.1)    # vuelve de a poco al λ nominal
        return super().actualizar(t, y, valido)


# ---- escenas: (posición real en t, perturbación de la medición en el cuadro k) -----------------
V_OBJ = 15.0
K_EV = int(round(T_EV / T))


def aberrante(n):
    return (lambda t: V_OBJ * t), (lambda k: 3.0 if K_EV <= k < K_EV + n else 0.0)


def salto(d):
    return (lambda t: V_OBJ * t + (d if t >= T_EV else 0.0)), (lambda k: 0.0)


def inversion(v):
    return (lambda t: v * t if t < T_EV else v * T_EV - v * (t - T_EV)), (lambda k: 0.0)


ESCENAS = [("Aberrante de 2 cuadros (+3°)", aberrante(2)), ("Aberrante de 3 cuadros (+3°)", aberrante(3)),
           ("Salto real de 1°", salto(1.0)), ("Salto real de 3°", salto(3.0)),
           ("Inversión brusca (de +40 a −40 °/s)", inversion(40.0))]


def correr(Clase, escena, sigma, semilla):
    real, perturbacion = escena
    pp = Parametros().prediccion
    #* Las variantes se comparan sobre el estimador ORIGINAL: sin reinicio informado ni gracia
    #* (cada subclase implementa su propio reinicio).
    pp.reinicio_informado, pp.gracia_gating, pp.t_acel_max_s = False, 0, 0.0
    est = Clase(pp, sigma_min=pp.sigma_min_deg)
    sup = Supervisor(Parametros().supervisor)
    rng = np.random.default_rng(semilla)
    ult, ref, err, vel, estados = float("nan"), [], [], [], []
    for k in range(int(3.0 / T)):
        t = k * T
        est.actualizar(t, real(t) + rng.normal(0, sigma) + perturbacion(k), True)
        listo = est.listo()
        estado = sup.paso(t, True, 1.0 if listo else 0.0)
        if estado == Estado.SEGUIMIENTO and listo:
            ult = est.posicion(t + H)            # fuera de SEGUIMIENTO se retiene la última referencia
            v = est.velocidad(t + H)
            vel.append(0.0 if math.isnan(v) else v)
        else:
            vel.append(0.0)
        ref.append(ult)
        err.append(ult - real(t + H))
        estados.append(estado)
    err, vel = np.abs(np.array(err)), np.array(vel)
    post = err[K_EV:K_EV + 60]
    fuera = np.nonzero(post > max(0.2, 4 * sigma))[0]   # "recuperado": error bajo 0,2° (o 4σ si el ruido es mayor)
    v_real = np.array([(real((k + 1) * T + H) - real(k * T + H)) / T for k in range(K_EV + 4, K_EV + 16)])
    return (sum(1 for e in estados[K_EV:K_EV + 60] if e == Estado.PERDIDA),          # cuadros en PÉRDIDA
            float(post.max()),                                                       # error máximo [°]
            (fuera[-1] + 1) * T * 1e3 if len(fuera) else 0.0,                        # tiempo hasta recuperarse [ms]
            float(np.sqrt(np.mean(err[K_EV + 4:K_EV + 16] ** 2))),                   # error RMS, cuadros 4–15 [°]
            float(np.sqrt(np.mean((vel[K_EV + 4:K_EV + 16] - v_real) ** 2))))        # error de ω_ff RMS [°/s]


if __name__ == "__main__":
    for sigma in (0.05, 0.15):
        print(f"\n===== ruido de la medición σ = {sigma}° (promedio de 100 corridas) =====")
        for nombre, escena in ESCENAS:
            print(f"\n{nombre}")
            print(f"  {'variante':34s}{'PÉRDIDA':>9s}{'error máx':>11s}{'recupera':>10s}{'error 50–200 ms':>17s}{'error ω_ff':>12s}")
            for Clase in (V0, V4, V1, V2, V5, V3):
                m = np.mean([correr(Clase, escena, sigma, s) for s in range(100)], axis=0)
                print(f"  {Clase.nombre:34s}{m[0]:7.2f} c{m[1]:9.2f}°{m[2]:7.0f} ms{m[3]:15.2f}°{m[4]:8.1f} °/s")
