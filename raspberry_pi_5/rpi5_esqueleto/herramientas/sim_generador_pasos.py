#!/usr/bin/env python3
"""
sim_generador_pasos.py — ¿conviene cambiar el acumulador de fase del generador de pasos?

Simula la RETENCIÓN de una referencia fija con el lazo completo del ESP32: AS5600 cuantizado
(12 bits) con ruido, el PI de control_eje (réplica de pt/modelo_eje.py) a 1 kHz y el generador
de pasos tic a tic (40 kHz), en sus dos variantes:

  sin_signo  el del firmware (esp32_pan_tilt/src/generador_pasos.cpp): la fase acumula |v| y
             cada desborde es un paso en el sentido vigente
  con_signo  alternativa "exacta": la fase acumula v con signo; desborde = +1 paso, préstamo = −1

Resultado (análisis del 17-11-2026, docs/analisis_generador_pasos.md): la variante con signo
emite decenas de veces más pasos en reposo, porque queda oscilando sobre el borde de un paso.
La del firmware tiene una histéresis implícita de un paso completo. No conviene cambiarla.

Uso (PC):  python3 herramientas/sim_generador_pasos.py        # ~1 min
"""
#? Planta ideal: el eje sigue exactamente los pasos emitidos (sin inercia ni resonancia).
#? Sirve para comparar las dos variantes entre sí, no para predecir la vibración real.
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from pt.modelo_eje import ControlEje, RESOLUCION_AS5600_DEG as LSB    # noqa: E402

F_TICK, DT = 40000, 1e-3
TICS_POR_CTL = F_TICK // 1000


def simular(modo, kp, ki, micropasos, ruido_deg, ref=10.033, dur=20.0, semilla=0):
    rng = np.random.default_rng(semilla)
    grados_por_paso = 1.8 / micropasos
    ctl = ControlEje(kp, ki)
    fase = 0 if modo == "sin_signo" else 1 << 31
    pos = pasos = inversiones = ultimo = 0
    theta = []
    for k in range(int(dur / DT)):
        ang = pos * grados_por_paso
        med = round((ang + rng.normal(0, ruido_deg)) / LSB) * LSB           # AS5600: ruido + 12 bits
        v = ctl.paso(ref, med, 0.0, DT) / grados_por_paso                   # ω_cmd en micropasos/s
        inc = int(min(abs(v), F_TICK / 4) * (2 ** 32 / F_TICK))
        sentido = 1 if v >= 0 else -1
        for _ in range(TICS_POR_CTL):
            d = 0
            if modo == "sin_signo":
                f2 = (fase + inc) & 0xFFFFFFFF
                if f2 < fase:
                    d = sentido
            else:
                f2 = fase + sentido * inc
                if f2 >= 2 ** 32:
                    f2, d = f2 - 2 ** 32, 1
                elif f2 < 0:
                    f2, d = f2 + 2 ** 32, -1
            fase = f2
            if d:
                pos += d
                pasos += 1
                inversiones += 1 if (ultimo and d != ultimo) else 0
                ultimo = d
        if k * DT > 5.0:
            theta.append(ang)
    th = np.array(theta)
    return pasos / dur, inversiones / dur, float(th.std()), float(th.max() - th.min())


if __name__ == "__main__":
    print("Retención de una referencia fija, 20 s, micropasos 1/16 (promedio de 3 semillas)")
    print(f"{'eje (Kp, Ki)':<18}{'ruido [°]':>10}{'variante':>11}{'pasos/s':>9}{'inversiones/s':>15}{'σ θ [°]':>9}{'p-p θ [°]':>11}")
    for nombre, kp, ki in (("pan (3; 2,25)", 3.0, 2.25), ("tilt (10; 25)", 10.0, 25.0)):
        for ruido in (0.03, 0.06):
            for modo in ("sin_signo", "con_signo"):
                r = np.mean([simular(modo, kp, ki, 16, ruido, semilla=s) for s in range(3)], axis=0)
                print(f"{nombre:<18}{ruido:>10.2f}{modo:>11}{r[0]:>9.1f}{r[1]:>15.1f}{r[2]:>9.3f}{r[3]:>11.3f}")
