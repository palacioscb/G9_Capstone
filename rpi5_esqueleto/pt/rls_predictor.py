"""
Mínimos cuadrados recursivos con factor de olvido, usado como predictor
de trayectoria para el bloque de Estimación y predicción (IEE2913).

Modelo local por eje de imagen:
    y(t0 + s) = th[0] + th[1]*s + th[2]*s^2

El origen t0 se traslada a la última muestra en cada paso, de modo que el
regresor siempre se evalúa con s del orden del período de cuadro (~16,7 ms).
Sin esa traslación, s crece sin límite y s^2 destruye el condicionamiento.
"""

from math import comb
import numpy as np


class PredictorRLS:
    def __init__(self, n=3, lam=0.95, delta=1e4, traza_max=1e6):
        self.n = n                    # nº de parámetros (2 = pos+vel, 3 = +acel)
        self.lam = lam                # factor de olvido
        self.traza_max = traza_max    # tope anti-windup de covarianza
        self.P = delta * np.eye(n)    # covarianza; delta grande = "no sé nada"
        self.th = np.zeros(n)         # parámetros estimados
        self.t0 = None                # origen temporal del polinomio
        self.n_valid = 0              # muestras válidas acumuladas

    def _A(self, s):
        """Traslación del origen: th' = A @ th, P' = A P A^T."""
        A = np.zeros((self.n, self.n))
        for i in range(self.n):
            for j in range(i, self.n):
                A[i, j] = comb(j, i) * s ** (j - i)
        return A

    def _phi(self, s):
        return np.array([s ** i for i in range(self.n)])

    def actualizar(self, t, y, valido):
        """Un cuadro. Devuelve el error de predicción a priori, o None."""
        if self.t0 is None:
            self.t0 = t
            self.th[0] = y if valido else 0.0
            if valido:
                self.n_valid = 1
            return None

        s = t - self.t0

        if not valido:
            # Sin medición: sólo crece la incertidumbre (oclusión).
            self.P = self.P / self.lam
            self._sanear()
            return None

        phi = self._phi(s)
        Pphi = self.P @ phi
        den = self.lam + phi @ Pphi
        k = Pphi / den
        e = y - phi @ self.th
        self.th = self.th + k * e
        self.P = (self.P - np.outer(k, Pphi)) / self.lam
        self._sanear()

        # Trasladar el origen a la muestra recién incorporada.
        A = self._A(s)
        self.th = A @ self.th
        self.P = A @ self.P @ A.T
        self.t0 = t
        self.n_valid += 1
        return e

    def _sanear(self):
        self.P = 0.5 * (self.P + self.P.T)          # forzar simetría
        tr = np.trace(self.P)
        if tr > self.traza_max:                      # anti-windup
            self.P *= self.traza_max / tr

    def listo(self):
        return self.n_valid >= self.n

    def predecir(self, t):
        """Predicción en el instante absoluto t."""
        return float(self._phi(t - self.t0) @ self.th)
