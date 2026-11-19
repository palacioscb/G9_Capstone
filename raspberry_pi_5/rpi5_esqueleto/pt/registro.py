"""
registro.py — registro por cuadro (anticipo de MR-2 / CTL-7).

Una fila por cuadro con marca de tiempo común (CLOCK_MONOTONIC) para medir
latencias por etapa (ADV-2) y analizar la respuesta temporal después.
"""
import csv
import os
from datetime import datetime

#* Ángulos en grados. "_med" = posición de la plataforma (AS5600) en el instante de exposición;
#* "_obj" = dirección del objetivo en el mundo; "_ref" = consigna enviada al ESP32.
COLUMNAS = [
    "k", "t_exp_ns", "t_entrega_ns", "t_fin_ns", "lat_captura_ms", "lat_total_ms",
    "t_det_ms", "estado", "valido", "motivo", "u", "w", "pico", "area",
    "dpan_deg", "dtilt_deg", "pan_med", "tilt_med", "pan_obj", "tilt_obj",
    "pan_pred", "tilt_pred", "vpan_pred", "vtilt_pred", "confianza", "t_sin_med_s", "est_u", "est_w",
    "pan_ref", "tilt_ref", "wff_pan", "wff_tilt", "h_ms",
    "esp32_estado", "esp32_fallas", "esp32_pan_med", "esp32_tilt_med",
    "esp32_pan_ref", "esp32_tilt_ref", "esp32_w_pan", "esp32_w_tilt",
    "u_real", "w_real", "err_centro_px",
    #* Con el tablero se puede cambiar de objetivo y de cámara en la misma corrida: cada fila
    #* dice con qué detector y con qué exposición/ganancia REALES (metadatos) se obtuvo.
    "detector", "exp_us", "ganancia", "lam",
    "ff",                         # 1 = feedforward de velocidad activo en este cuadro
    #* Diagnóstico del gating (para ajustar el piso con datos): gating_d = innovación más
    #* alejada de los dos ejes, en veces su umbral (> 1 = rechazo); σ de la innovación por eje.
    "gating_d", "sigma_pan", "sigma_tilt",
    #* Detector por forma: plantilla más parecida y su distancia (se acepta si <= dist_max).
    "forma", "forma_dist",
    #* Mando desde el tablero (19-11-2026): ganancias del PI vigentes en el ESP32 (vacío si no se
    #* conocen), 1 = referencia de prueba manual (no es seguimiento) y etiqueta del experimento
    #* que se está grabando. Con esto una sola corrida se puede separar por ganancias y por ω_ff.
    "kp_pan", "ki_pan", "kp_tilt", "ki_tilt", "manual", "experimento",
]


class Registro:
    def __init__(self, carpeta="registros", prefijo="sesion"):
        os.makedirs(carpeta, exist_ok=True)
        self.ruta = os.path.join(carpeta, f"{prefijo}_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.f = open(self.ruta, "w", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=COLUMNAS, extrasaction="ignore")
        self.w.writeheader()
        self.n = 0

    def fila(self, **kv):
        self.w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in kv.items()})
        self.n += 1
        if self.n % 80 == 0:
            #* ~1 vez por segundo a disco: el monitor puede descargar el CSV en curso (/registro.csv)
            #* y un corte de energía pierde a lo más 1 s de registro.
            self.f.flush()

    def cerrar(self):
        self.f.close()


class Grabacion:
    """Experimentos guardados aparte, uno por archivo, mientras la corrida sigue (tablero).

    El registro de la corrida completa (Registro) no cambia. Al iniciar una grabación con una
    etiqueta, las mismas filas se escriben además en registros/exp_<etiqueta>_<fecha>.csv hasta
    que se detiene. Así cada caso de una comparación (ganancias A sin ω_ff, A con ω_ff...) queda
    en su propio CSV, listo para herramientas/comparar_ganancias_ff.py --lab."""

    def __init__(self, carpeta="registros"):
        self.carpeta = carpeta
        self.reg = None
        self.etiqueta = ""
        self.hechas = []            # [(nombre de archivo, ruta, filas, etiqueta)]

    @property
    def activa(self):
        return self.reg is not None

    def iniciar(self, etiqueta):
        if self.activa:
            self.detener()
        self.reg = Registro(self.carpeta, prefijo=f"exp_{etiqueta}")
        self.etiqueta = etiqueta
        return self.reg.ruta

    def fila(self, **kv):
        self.reg.fila(**kv)

    def detener(self):
        """Cierra el archivo. Devuelve (nombre, ruta, filas) o None si no había grabación."""
        if not self.activa:
            return None
        reg, self.reg = self.reg, None
        reg.cerrar()
        hecho = (os.path.basename(reg.ruta), reg.ruta, reg.n, self.etiqueta)
        self.hechas.append(hecho)
        self.etiqueta = ""
        return hecho[:3]
