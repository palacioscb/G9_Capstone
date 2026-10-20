"""
prediccion.py — Estimación y predicción (EP-1 a EP-5), sobre el RLS del equipo.

  EP-1  gating: se descarta la medición si |innovación| > k·σ (ROB-4)
  EP-2  estimador cinemático por eje: PredictorRLS (pt/rls_predictor.py, sin cambios)
  EP-3  extrapolación al horizonte h (retardo total del lazo)
  EP-4  oclusiones: sin medición sólo se extrapola; tras t_max se declara no confiable
  EP-5  confianza en [0, 1]

Tiempo en segundos desde el primer cuadro (evita números grandes en el regresor).
"""
import math
from dataclasses import dataclass

from .rls_predictor import PredictorRLS


class EstimadorEje:
    def __init__(self, pp):
        self.pp = pp
        self.reiniciar()

    def reiniciar(self):
        self.rls = PredictorRLS(n=self.pp.orden, lam=self.pp.lam)
        self.sigma = None          # desviación de la innovación (EMA)
        self.rechazos = 0
        self.t_ult_valida = None

    def actualizar(self, t, y, valido):
        """Devuelve 'aceptada', 'rechazada' o 'sin_medicion'."""
        pp = self.pp
        if not valido:
            if self.t_ult_valida is not None:
                self.rls.actualizar(t, 0.0, False)       # EP-4: sólo crece la incertidumbre
            return "sin_medicion"

        if self.t_ult_valida is None:                     # primera muestra
            self.rls.actualizar(t, y, True)
            self.t_ult_valida = t
            return "aceptada"

        # EP-1: gating sobre la innovación a priori
        if self.rls.listo() and self.sigma is not None:
            innov = y - self.rls.predecir(t)
            lim = pp.k_gating * max(self.sigma, pp.sigma_min_px)
            if abs(innov) > lim:
                self.rechazos += 1
                if self.rechazos <= pp.max_rechazos:
                    return "rechazada"
                # varios rechazos seguidos = maniobra real, no un aberrante: reiniciar
                self.reiniciar()
                self.rls.actualizar(t, y, True)
                self.t_ult_valida = t
                return "aceptada"

        e = self.rls.actualizar(t, y, True)
        self.rechazos = 0
        self.t_ult_valida = t
        if e is not None and self.rls.listo():
            a = abs(e)
            self.sigma = a if self.sigma is None else 0.9 * self.sigma + 0.1 * a
        return "aceptada"

    def predecir(self, t):
        return self.rls.predecir(t) if self.t_ult_valida is not None else float("nan")


@dataclass
class Estimacion:
    u: float                 # posición predicha en t + h [px]
    w: float
    confianza: float         # EP-5
    t_sin_medicion: float    # [s]
    estado_u: str
    estado_w: str


class Estimador2D:
    def __init__(self, pp):
        self.pp = pp
        self.eu = EstimadorEje(pp)
        self.ew = EstimadorEje(pp)
        self.t0 = None

    def reiniciar(self):
        self.eu.reiniciar()
        self.ew.reiniciar()

    def actualizar(self, t_s, det):
        """t_s: instante de exposición del cuadro [s]. det: Resultado de deteccion."""
        if self.t0 is None:
            self.t0 = t_s
        t = t_s - self.t0
        su = self.eu.actualizar(t, det.u, det.valido)
        sw = self.ew.actualizar(t, det.w, det.valido)

        th = t + self.pp.horizonte_s                    # EP-3
        u_hat, w_hat = self.eu.predecir(th), self.ew.predecir(th)

        # EP-4 / EP-5
        tu = self.eu.t_ult_valida
        t_sin = (t - tu) if tu is not None else float("inf")
        if not self.eu.rls.listo() or t_sin > self.pp.t_max_s:
            conf = 0.0
        else:
            conf = math.exp(-t_sin / self.pp.tau_conf_s)
        return Estimacion(u_hat, w_hat, conf, t_sin, su, sw)
