"""
registro.py — registro por cuadro (anticipo de MR-2 / CTL-7).

Una fila por cuadro con marca de tiempo común (CLOCK_MONOTONIC) para medir
latencias por etapa (ADV-2) y analizar la respuesta temporal después.
"""
import csv
import os
from datetime import datetime

COLUMNAS = [
    "k", "t_exp_ns", "t_entrega_ns", "t_fin_ns", "lat_captura_ms", "lat_total_ms",
    "t_det_ms", "estado", "valido", "motivo", "u", "w", "pico", "area",
    "u_pred", "w_pred", "confianza", "t_sin_med_s", "est_u", "est_w",
    "dpan_deg", "dtilt_deg", "omega_pan", "omega_tilt", "pasos_s", "u_real", "w_real",
]


class Registro:
    def __init__(self, carpeta="registros", prefijo="sesion"):
        os.makedirs(carpeta, exist_ok=True)
        self.ruta = os.path.join(carpeta, f"{prefijo}_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.f = open(self.ruta, "w", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=COLUMNAS, extrasaction="ignore")
        self.w.writeheader()

    def fila(self, **kv):
        self.w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in kv.items()})

    def cerrar(self):
        self.f.close()
