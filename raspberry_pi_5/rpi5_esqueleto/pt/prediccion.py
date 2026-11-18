"""
prediccion.py — Estimación y predicción (EP-1 a EP-5), sobre el RLS del equipo.

  EP-1  gating: se descarta la medición si |innovación| > k·σ (ROB-4)
  EP-2  estimador cinemático por eje: PredictorRLS (pt/rls_predictor.py, sin cambios)
  EP-3  extrapolación al horizonte h (retardo total del lazo)
  EP-4  oclusiones: sin medición sólo se extrapola; tras t_max se declara no confiable
  EP-5  confianza en [0, 1]

Tiempo en segundos desde el primer cuadro (evita números grandes en el regresor).
"""
#* Revisión del 17-11-2026 (docs/prediccion_rls.pdf, ajustes 4, 5 y 6). Cada cambio se puede
#* apagar en parametros.Prediccion, y con los tres apagados el código se comporta como antes:
#*   reinicio_informado  tras max_rechazos rechazos seguidos, el estimador nuevo parte con las
#*                       mediciones rechazadas: queda listo en el mismo cuadro, la confianza
#*                       no cae a 0 y el supervisor no pasa por PÉRDIDA;
#*   gating_conjunto     la medición (pan, tilt) se acepta o se rechaza COMPLETA, y la
#*                       confianza mira los dos ejes;
#*   t_acel_max_s        más allá de ese tiempo desde la última medición aceptada, la
#*                       predicción sigue a velocidad constante (oclusión sin aceleración).
#! El PISO del gating (sigma_min) conserva su significado: umbral por eje = k·max(σ, piso).
#! El gating conjunto NO lo estrecha: rechaza si algún eje supera SU umbral, igual que antes;
#! lo que cambia es que la decisión vale para los dos ejes.
#? El piso depende del detector: con un detector por forma el ruido del centroide será otro.
#? Para ajustarlo con datos, el registro guarda gating_d (innovación / umbral·k) y σ por eje;
#? herramientas/analizar_registro.py los resume en el punto 5.
import math
from dataclasses import dataclass

from .rls_predictor import PredictorRLS

NAN = float("nan")


class EstimadorEje:
    def __init__(self, pp, sigma_min=None):
        self.pp = pp
        #* Piso de σ del gating en las unidades del eje: px (por defecto) o grados.
        self.sigma_min = pp.sigma_min_px if sigma_min is None else sigma_min
        self.sigma = None          # desviación de la innovación (EMA); sobrevive a los reinicios informados
        self.reiniciar()

    def reiniciar(self):
        self.rls = PredictorRLS(n=self.pp.orden, lam=self.pp.lam)
        self.sigma = None
        self.rechazos = 0
        self.cola = []             # mediciones rechazadas seguidas: (t, y)
        self.t_ult_valida = None
        self.z_ult = NAN           # innovación normalizada del último cuadro (diagnóstico)

    # ---- piezas (las usa Estimador2D para decidir en conjunto) -------------------------------
    def z(self, t, y):
        """Innovación a priori dividida por max(σ, piso). None si el gating todavía no aplica."""
        #* El gating se arma cuando el ajuste tiene n + gracia_gating muestras. Con sólo n, el
        #* polinomio pasa casi exacto por las muestras y su predicción a un cuadro es ruidosa
        #* (≈ 4,5 veces el ruido de la medición con orden 3): al nivel del piso, eso rechazaba
        #* mediciones buenas justo después de un reinicio.
        r = self.rls
        if self.t_ult_valida is None or self.sigma is None:
            return None
        if r.n_valid < r.n + getattr(self.pp, "gracia_gating", 0):
            return None
        return (y - self.predecir(t)) / max(self.sigma, self.sigma_min, 1e-9)

    def aceptar(self, t, y):
        r = self.rls
        previa = (y - self.predecir(t)) if (self.t_ult_valida is not None and r.listo()) else None
        e = r.actualizar(t, y, True)
        if previa is None:
            previa = e                                   # muestra que completa el arranque
        self.rechazos, self.cola = 0, []
        self.t_ult_valida = t
        if previa is not None and r.listo():
            a = abs(previa)
            self.sigma = a if self.sigma is None else 0.9 * self.sigma + 0.1 * a

    def sin_medicion(self, t):
        if self.t_ult_valida is not None:
            self.rls.actualizar(t, 0.0, False)           # EP-4: sólo crece la incertidumbre

    def reiniciar_con(self, muestras):
        """Reinicio informado: estimador nuevo alimentado con las mediciones rechazadas."""
        #* Varios rechazos seguidos = maniobra real o salto de la detección, no un aberrante.
        #* Las mediciones rechazadas ya describen la trayectoria nueva: con max_rechazos + 1
        #* muestras (4) el RLS de orden 3 queda listo de inmediato.
        #? σ se conserva: es el nivel de ruido de la medición, que no cambia con la maniobra.
        sigma = self.sigma
        self.reiniciar()
        for tc, yc in muestras:
            self.rls.actualizar(tc, yc, True)
        self.sigma, self.t_ult_valida = sigma, muestras[-1][0]

    def ponerse_al_dia(self, muestras):
        """Incorpora mediciones que quedaron retenidas por un rechazo debido al OTRO eje."""
        for tc, yc in muestras:
            self.aceptar(tc, yc)

    # ---- un eje solo (gating_conjunto = False, y las herramientas de un eje) ------------------
    def actualizar(self, t, y, valido):
        """Devuelve 'aceptada', 'rechazada', 'reinicio' o 'sin_medicion'."""
        pp = self.pp
        self.z_ult = NAN
        if not valido:
            self.sin_medicion(t)
            return "sin_medicion"
        z = self.z(t, y)                                  # EP-1: gating sobre la innovación a priori
        if z is not None:
            self.z_ult = z
            if abs(z) > pp.k_gating:
                self.rechazos += 1
                self.cola.append((t, y))
                if self.rechazos <= pp.max_rechazos:
                    return "rechazada"
                if getattr(pp, "reinicio_informado", True):
                    self.reiniciar_con(list(self.cola))
                else:                                     # como antes del 17-11: desde cero
                    self.reiniciar()
                    self.aceptar(t, y)
                return "reinicio"
        self.aceptar(t, y)
        return "aceptada"

    # ---- predicción --------------------------------------------------------------------------
    def _s_tope(self, s):
        """Tiempo desde la última medición aceptada hasta el que se usa la aceleración."""
        tope = getattr(self.pp, "t_acel_max_s", 0.0)
        return min(s, tope) if tope and tope > 0 else s

    def predecir(self, t):
        #* y(t0 + s) = th0 + th1·s + th2·s². Para s > t_acel_max_s se continúa en línea recta
        #* con la velocidad que el polinomio tiene en t_acel_max_s: la posición y la velocidad
        #* quedan continuas (sin escalón en θ_ref ni en ω_ff al empezar una oclusión).
        if self.t_ult_valida is None:
            return NAN
        r = self.rls
        s = t - r.t0
        sa = self._s_tope(s)
        y = r.predecir(r.t0 + sa)
        if s > sa and r.listo():
            y += self._derivada(sa) * (s - sa)
        return y

    def _derivada(self, s):
        th = self.rls.th
        return float(sum(i * th[i] * s ** (i - 1) for i in range(1, len(th))))

    def velocidad(self, t):
        """Derivada de la predicción en el instante t (unidades del eje por segundo)."""
        #* Se calcula con los parámetros del RLS sin modificar rls_predictor.py:
        #*   y'(t0 + s) = th1 + 2·th2·s, congelada en s = t_acel_max_s (ver predecir).
        if self.t_ult_valida is None or not self.rls.listo() or self.rls.t0 is None:
            return NAN
        return self._derivada(self._s_tope(t - self.rls.t0))


@dataclass
class Estimacion:
    u: float                 # posición predicha en t + h [px o °, según el estimador]
    w: float
    confianza: float         # EP-5
    t_sin_medicion: float    # [s]
    estado_u: str
    estado_w: str
    vu: float = NAN          # velocidad predicha en t + h [px/s o °/s]
    vw: float = NAN
    #* Diagnóstico del gating: d = máx(|z_u|, |z_w|) / k. d > 1 es rechazo; un p99 de d cerca
    #* de 1 con el objetivo quieto dice que el piso (o k) está apretado para ese detector.
    d_gating: float = NAN
    sigma_u: float = NAN     # σ de la innovación (EMA) de cada eje, sin el piso
    sigma_w: float = NAN


class Estimador2D:
    """Dos estimadores por eje. Sirve en píxeles (u, w) o en ángulos del mundo (pan, tilt):
    sólo cambian las unidades del piso de σ del gating (sigma_min)."""

    def __init__(self, pp, sigma_min=None):
        self.pp = pp
        self.eu = EstimadorEje(pp, sigma_min)
        self.ew = EstimadorEje(pp, sigma_min)
        self.t0 = None
        self.rechazos = 0
        self.cola = []           # mediciones rechazadas seguidas: (t, a, b, fuera_u, fuera_w)

    def reiniciar(self):
        self.eu.reiniciar()
        self.ew.reiniciar()
        self.rechazos, self.cola = 0, []

    def actualizar(self, t_s, det, t_pred_s=None):
        """t_s: instante de exposición del cuadro [s]. det: Resultado de deteccion.
        t_pred_s: instante absoluto [s] al que se predice; por defecto t_s + horizonte_s."""
        return self.actualizar_xy(t_s, det.valido, det.u, det.w, t_pred_s)

    def _conjunto(self, t, a, b, valido):
        """Gating conjunto: una decisión para los dos ejes. Devuelve (estado_u, estado_w)."""
        pp, eu, ew = self.pp, self.eu, self.ew
        eu.z_ult = ew.z_ult = NAN
        if not valido:
            eu.sin_medicion(t)
            ew.sin_medicion(t)
            return "sin_medicion", "sin_medicion"
        zu, zw = eu.z(t, a), ew.z(t, b)
        eu.z_ult = NAN if zu is None else zu
        ew.z_ult = NAN if zw is None else zw
        fuera_u = zu is not None and abs(zu) > pp.k_gating
        fuera_w = zw is not None and abs(zw) > pp.k_gating
        if not (fuera_u or fuera_w):
            self.rechazos, self.cola = 0, []
            eu.aceptar(t, a)
            ew.aceptar(t, b)
            return "aceptada", "aceptada"

        #* Algún eje fuera de su umbral: se descarta la medición completa (una detección
        #* falsa suele estar mal en los dos ejes, aunque sólo uno lo delate).
        self.rechazos += 1
        self.cola.append((t, a, b, fuera_u, fuera_w))
        eu.rechazos = ew.rechazos = self.rechazos
        if self.rechazos <= pp.max_rechazos:
            return "rechazada", "rechazada"

        #* max_rechazos + 1 seguidas: es la trayectoria nueva. Se reinicia SÓLO el eje que
        #* estuvo fuera de su umbral; el otro no tiene por qué perder su historia y se pone
        #* al día con las mediciones retenidas.
        cola, estados = self.cola, []
        for eje, i in ((eu, 1), (ew, 2)):
            muestras = [(c[0], c[i]) for c in cola]
            if any(c[i + 2] for c in cola):
                if getattr(pp, "reinicio_informado", True):
                    eje.reiniciar_con(muestras)
                else:                                     # como antes del 17-11: desde cero
                    eje.reiniciar()
                    eje.aceptar(*muestras[-1])
                estados.append("reinicio")
            else:
                eje.ponerse_al_dia(muestras)
                estados.append("aceptada")
        self.rechazos, self.cola = 0, []
        eu.rechazos = ew.rechazos = 0
        return estados[0], estados[1]

    def actualizar_xy(self, t_s, valido, a, b, t_pred_s=None):
        """Igual que actualizar(), con la medición (a, b) explícita (p. ej. pan y tilt en °)."""
        if self.t0 is None:
            self.t0 = t_s
        t = t_s - self.t0
        pp, eu, ew = self.pp, self.eu, self.ew
        conjunto = getattr(pp, "gating_conjunto", True)
        if conjunto:
            su, sw = self._conjunto(t, a, b, valido)
        else:
            su = eu.actualizar(t, a, valido)
            sw = ew.actualizar(t, b, valido)

        th = (t + pp.horizonte_s) if t_pred_s is None else (t_pred_s - self.t0)   # EP-3
        u_hat, w_hat = eu.predecir(th), ew.predecir(th)

        # EP-4 / EP-5
        if conjunto:
            #* Los dos ejes: confianza 0 si alguno no está listo, y el tiempo sin medición es
            #* el del eje que lleva más tiempo sin aceptar una.
            ult = [e.t_ult_valida for e in (eu, ew)]
            t_sin = float("inf") if None in ult else t - min(ult)
            listo = eu.rls.listo() and ew.rls.listo()
        else:                                             # como antes del 17-11: sólo el eje u
            t_sin = (t - eu.t_ult_valida) if eu.t_ult_valida is not None else float("inf")
            listo = eu.rls.listo()
        conf = 0.0 if (not listo or t_sin > pp.t_max_s) else math.exp(-t_sin / pp.tau_conf_s)

        zs = [abs(z) for z in (eu.z_ult, ew.z_ult) if not math.isnan(z)]
        return Estimacion(u_hat, w_hat, conf, t_sin, su, sw, eu.velocidad(th), ew.velocidad(th),
                          d_gating=(max(zs) / pp.k_gating) if zs else NAN,
                          sigma_u=NAN if eu.sigma is None else eu.sigma,
                          sigma_w=NAN if ew.sigma is None else ew.sigma)
