#!/usr/bin/env python3
"""
respuesta_escalon.py — Prueba del modelo de pid_t_2.pdf contra la respuesta real de un eje.

Uso (en la RPi, con el ESP32 por USB; el firmware esp32_pan_tilt cargado):
    python3 herramientas/respuesta_escalon.py --puerto /dev/ttyUSB0 --eje pan --escalon 3 --habilitar
    python3 herramientas/respuesta_escalon.py --puerto /dev/ttyUSB0 --eje tilt --escalon 2 --kp 20 --ki 100 --alfa 0
    python3 herramientas/respuesta_escalon.py --archivo registros/escalon_pan_....csv   # re-analizar
    python3 herramientas/respuesta_escalon.py --solo-modelo --eje pan --kp 5 --ki 6.25 --escalon 10
"""
#* Qué hace:
#*   1. (con --puerto) habilita el ESP32 si se pide, fija Kp/Ki/α y manda un escalón por
#*      consola ('g <eje> <deg>'); el firmware captura 6 s (una muestra cada 2 ms) y se descarga con 'd';
#*   2. simula el MISMO escalón con el modelo del eje (planta de pid_t_2 + réplica del PI
#*      del firmware), en dos versiones: lineal (sin límites) y con los límites del firmware;
#*   3. compara tiempo de subida, sobrepaso, establecimiento y error final;
#*   4. busca el ζ (y por lo tanto b) que mejor reproduce la medición y recalcula el techo
#*      de estabilidad Kp < b/M. Así queda medido lo que pid_t_2 §7 dejó supuesto.
#
#! Antes del primer escalón: prueba de signos ('t pan' / 't tilt' en la consola). Con un
#! signo mal puesto el PI realimenta positivo; el firmware lo corta por pérdida de pasos,
#! pero igual conviene partir con escalones chicos (2–5°).
#? --alfa 0 quita el límite de aceleración: es lo que permite comparar con el modelo LINEAL.
#? Usarlo sólo con escalones chicos (salto inicial de ω = Kp·escalón).
import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pt.modelo_eje import (PAN, TILT, ParametrosEje, ajustar_zeta, con_zeta,   # noqa: E402
                           metricas_escalon, polos_lazo_cerrado, simular_escalon,
                           RESOLUCION_AS5600_DEG)


# ---------------------------------------------------------------------------
#* Comunicación por consola con el firmware
# ---------------------------------------------------------------------------
def abrir(puerto, baudios):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = puerto, baudios, 0.2
    s.dtr = False                       #! sin esto, abrir el puerto reinicia el ESP32
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def comando(s, texto, espera=0.15, mostrar=True):
    s.write((texto + "\n").encode())
    time.sleep(espera)
    resp = s.read(s.in_waiting or 1).decode(errors="replace")
    if mostrar:
        for linea in resp.splitlines():
            if linea.startswith("#"):
                print("   ESP32", linea)
    return resp


def descargar(s, timeout_s=15.0):
    """Manda 'd' y lee hasta '#FIN'. Devuelve (cabecera: dict, filas: ndarray N×5)."""
    s.reset_input_buffer()
    s.write(b"d\n")
    t_fin = time.monotonic() + timeout_s
    lineas, cab = [], None
    buf = b""
    while time.monotonic() < t_fin:
        buf += s.read(s.in_waiting or 1)
        while b"\n" in buf:
            linea, buf = buf.split(b"\n", 1)
            linea = linea.decode(errors="replace").strip()
            if linea.startswith("#CAPTURA"):
                cab = dict(re.findall(r"(\w+)=([^\s]+)", linea))
            elif linea.startswith("#FIN"):
                return cab, np.array(lineas, dtype=float)
            elif cab is not None and linea and linea[0] in "0123456789-":
                lineas.append([float(x) for x in linea.split(",")])
    raise RuntimeError("no llegó '#FIN': ¿hay captura? (el firmware responde 'no hay captura' si no)")


def guardar(ruta, cab, datos):
    with open(ruta, "w", newline="") as f:
        f.write("#CAPTURA " + " ".join(f"{k}={v}" for k, v in cab.items()) + "\n")
        w = csv.writer(f)
        w.writerow(["t_s", "ref_deg", "med_deg", "omega_dps", "pasos_deg"])
        w.writerows(datos.tolist())


def leer_archivo(ruta):
    with open(ruta) as f:
        primera = f.readline()
        cab = dict(re.findall(r"(\w+)=([^\s]+)", primera))
        datos = np.loadtxt(f, delimiter=",", skiprows=1)
    return cab, datos


# ---------------------------------------------------------------------------
def analizar(cab, datos, base: ParametrosEje, carpeta, nombre):
    kp, ki = float(cab["kp"]), float(cab["ki"])
    esc = float(cab["escalon"])
    wmax = float(cab.get("omega_max", 180))
    alfa = float(cab.get("alfa_max", 300))
    alfa = np.inf if alfa <= 0 else alfa
    t, ref, med = datos[:, 0], datos[:, 1], datos[:, 2]
    theta0 = float(ref[0] - esc)
    dur = float(t[-1])

    lin = simular_escalon(base, kp, ki, esc, duracion_s=dur, omega_max=np.inf, alfa_max=np.inf,
                          cuantizar=False, theta0=theta0, banda_integral=np.inf)
    fw = simular_escalon(base, kp, ki, esc, duracion_s=dur, omega_max=wmax, alfa_max=alfa,
                         cuantizar=True, theta0=theta0)
    z_aj, rms_aj = ajustar_zeta(t, med, theta0, esc, base, kp, ki, omega_max=wmax, alfa_max=alfa)
    aj = simular_escalon(con_zeta(base, z_aj), kp, ki, esc, duracion_s=dur, omega_max=wmax,
                         alfa_max=alfa, cuantizar=True, theta0=theta0)

    filas = [("medido", t, med), (f"modelo lineal ζ={base.zeta:.3f}", lin.t, lin.theta),
             (f"modelo + límites ζ={base.zeta:.3f}", fw.t, fw.med), (f"modelo ajustado ζ={z_aj:.3f}", aj.t, aj.med)]
    if abs(esc) >= 5.0:
        print("AVISO: escalón >= banda integral (5°): el firmware no integra al inicio y la respuesta "
              "ya no es la lineal de pid_t_2. Para validar el modelo usar escalones de 2–4°.")
    print(f"\nEje {cab.get('eje')} | Kp={kp} 1/s, Ki={ki} 1/s², escalón {esc:+.3f}°, "
          f"ω_max={wmax}°/s, α_max={'sin límite' if np.isinf(alfa) else f'{alfa}°/s²'}")
    print(f"{'curva':<30} {'t_subida[s]':>11} {'sobrepaso[%]':>12} {'t_est 2%[s]':>11} {'e_final[°]':>10}")
    for nom, tt, yy in filas:
        m = metricas_escalon(tt, yy, theta0, theta0 + esc, tol_min=RESOLUCION_AS5600_DEG)
        print(f"{nom:<30} {m['t_subida_s']:>11.3f} {m['sobrepaso_pct']:>12.1f} "
              f"{m['t_establecimiento_s']:>11.3f} {m['error_final_deg']:>10.3f}")

    p_aj = con_zeta(base, z_aj)
    #! ζ sólo es identificable si la resonancia (fn = ωn/2π) deja huella en la respuesta muestreada
    #! cada 2 ms (500 Hz). Pan: fn ≈ 23 Hz, sí. Tilt: fn ≈ 218 Hz, no: el ζ ajustado de tilt no es confiable.
    fn = base.wn / (2 * np.pi)
    if fn > 500.0 / 5:
        print(f"AVISO: resonancia a {fn:.0f} Hz, demasiado rápida para la captura a 500 Hz: "
              f"el ζ ajustado de este eje NO es confiable (sí lo es t_subida, sobrepaso y error).")
    print(f"\nζ ajustado = {z_aj:.3f}  (error RMS del ajuste {rms_aj:.3f}°)")
    print(f"  ⇒ b = {p_aj.b:.3e} N·m·s/rad   ⇒ techo de estabilidad Kp < b/M = {p_aj.kp_max:.1f} 1/s")
    print(f"  polos de lazo cerrado con ζ ajustado: {np.round(polos_lazo_cerrado(p_aj, kp, ki), 2)}")
    if rms_aj > 0.1 * abs(esc):
        print("  AVISO: el ajuste es pobre: revisar M, juego mecánico o roce seco (el modelo no los incluye).")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(t, ref, "k:", lw=1, label="referencia")
        for (nom, tt, yy), est in zip(filas, ["C0-", "C1--", "C2-.", "C3-"]):
            ax[0].plot(tt, yy, est, lw=1.4 if nom == "medido" else 1.1, label=nom)
        ax[0].set_ylabel("θ [°]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
        ax[0].set_title(f"Escalón {cab.get('eje')}: Kp={kp}, Ki={ki}")
        ax[1].plot(t, datos[:, 3], "C0-", lw=1, label="ω_cmd medido")
        ax[1].plot(fw.t, fw.omega, "C2-.", lw=1, label="ω_cmd modelo + límites")
        ax[1].set_ylabel("ω_cmd [°/s]"); ax[1].set_xlabel("t [s]"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
        fig.tight_layout()
        ruta = os.path.join(carpeta, nombre + ".png")
        fig.savefig(ruta, dpi=120)
        print(f"Figura: {ruta}")
    except ImportError:
        print("(sin matplotlib: no se genera la figura; sudo apt install python3-matplotlib)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", help="p. ej. /dev/ttyUSB0")
    ap.add_argument("--baudios", type=int, default=921600)
    ap.add_argument("--eje", choices=["pan", "tilt"], default="pan")
    ap.add_argument("--escalon", type=float, default=3.0, help="grados (relativo a la posición actual)")
    ap.add_argument("--kp", type=float, help="si se da, se manda 'k' antes del escalón")
    ap.add_argument("--ki", type=float)
    ap.add_argument("--alfa", type=float, help="°/s²; 0 = sin límite de aceleración")
    ap.add_argument("--habilitar", action="store_true", help="manda 'e 1' antes del escalón")
    ap.add_argument("--zeta", type=float, default=0.05, help="ζ del modelo de referencia (pid_t_2 §7)")
    ap.add_argument("--archivo", help="analizar una captura ya guardada")
    ap.add_argument("--solo-modelo", action="store_true", help="sin hardware: sólo el modelo")
    ap.add_argument("--carpeta", default="registros")
    a = ap.parse_args()

    base = con_zeta(PAN if a.eje == "pan" else TILT, a.zeta)
    os.makedirs(a.carpeta, exist_ok=True)
    nombre = f"escalon_{a.eje}_{datetime.now():%Y%m%d_%H%M%S}"

    if a.solo_modelo:
        kp = a.kp if a.kp is not None else (3.0 if a.eje == "pan" else 10.0)
        ki = a.ki if a.ki is not None else kp * kp / 4
        alfa = 300.0 if a.alfa is None else a.alfa
        s = simular_escalon(base, kp, ki, a.escalon, duracion_s=6.0, omega_max=180.0,
                            alfa_max=np.inf if alfa <= 0 else alfa)
        cab = {"eje": a.eje, "kp": kp, "ki": ki, "escalon": a.escalon, "omega_max": 180.0, "alfa_max": alfa}
        datos = np.column_stack([s.t, s.ref, s.med, s.omega, s.theta])[::2]   # como el firmware: cada 2 ms
        analizar(cab, datos, base, a.carpeta, nombre + "_modelo")
        return

    if a.archivo:
        cab, datos = leer_archivo(a.archivo)
        analizar(cab, datos, con_zeta(PAN if cab.get("eje") == "pan" else TILT, a.zeta),
                 os.path.dirname(a.archivo) or ".", os.path.splitext(os.path.basename(a.archivo))[0])
        return

    if not a.puerto:
        ap.error("falta --puerto (o --archivo / --solo-modelo)")
    s = abrir(a.puerto, a.baudios)
    try:
        if a.habilitar:
            comando(s, "e 1", 0.3)
        if a.kp is not None and a.ki is not None:
            comando(s, f"k {a.eje} {a.kp} {a.ki}")
        if a.alfa is not None:
            comando(s, f"a {a.eje} {a.alfa}")
        comando(s, "s")                              # partir desde reposo
        time.sleep(0.5)
        comando(s, f"g {a.eje} {a.escalon}")
        print("   capturando 6 s...")
        time.sleep(6.4)
        comando(s, "", 0.1, mostrar=True)
        cab, datos = descargar(s)
    finally:
        s.close()
    ruta = os.path.join(a.carpeta, nombre + ".csv")
    guardar(ruta, cab, datos)
    print(f"Captura: {ruta} ({len(datos)} muestras)")
    analizar(cab, datos, base, a.carpeta, nombre)


if __name__ == "__main__":
    main()
