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
