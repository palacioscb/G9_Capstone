#!/usr/bin/env python3
"""
fijar_ganancias.py — deja en el ESP32 las ganancias del PI de un grupo de la comparación
(los mismos grupos de herramientas/comparar_ganancias_ff.py), sin recompilar el firmware.

Uso (en la RPi, con main.py DETENIDO):
    python3 herramientas/fijar_ganancias.py A             # pan 3 / 2,25 ; tilt 10 / 25  (config.h)
    python3 herramientas/fijar_ganancias.py B             # pan 5 / 6,25 ; tilt 20 / 100 (pid_t_2 §8)
    python3 herramientas/fijar_ganancias.py --pan 4 4     # paso intermedio, sólo un eje
    python3 herramientas/fijar_ganancias.py --tilt 20 100
"""
#* Envía el comando de consola "k <eje> <Kp> <Ki>" por el mismo UART del enlace y muestra lo
#* que responde el firmware. No habilita los drivers ni mueve los motores.
#! main.py debe estar detenido: con tramas binarias en el último segundo, el firmware ignora
#! el texto que llega por el UART del enlace (ENLACE_ACTIVO_MS = 1000 en config.h).
#! Las ganancias fijadas con k NO quedan guardadas: si el ESP32 se reinicia, vuelven las de
#! config.h. Repetir este comando después de cada reinicio o corte de energía.
#! Grupo B en pan (Kp = 5): si ζ real ≈ 0,02 el techo de estabilidad es 5,7 1/s. Subir por
#! pasos (--pan 4 4 primero) y volver al grupo A si la plataforma zumba (~22 Hz).
import argparse
import os
import re
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, ".."))
sys.path.insert(0, AQUI)
from comparar_ganancias_ff import GRUPOS                           # noqa: E402
from pt.parametros import Parametros                              # noqa: E402


def fijar(s, eje, kp, ki, espera=0.4):
    """Manda "k <eje> <Kp> <Ki>" por el puerto s. True si el firmware confirma esos valores."""
    s.reset_input_buffer()
    s.write(f"k {eje} {kp:g} {ki:g}\n".encode())
    fin, texto = time.monotonic() + espera, b""
    while time.monotonic() < fin:
        texto += s.read(256)
    for linea in texto.decode("ascii", "replace").splitlines():
        m = re.search(rf"{eje}: Kp=([\d.]+) .*Ki=([\d.]+)", linea)
        if m and abs(float(m.group(1)) - kp) < 1e-3 and abs(float(m.group(2)) - ki) < 1e-3:
            print(f"  {eje:<5s} OK   {linea.strip()}")
            return True
    print(f"  {eje:<5s} SIN CONFIRMAR (se esperaba Kp={kp:g}, Ki={ki:g}). ¿main.py sigue corriendo? ¿ESP32 encendido?")
    return False


def main():
    pe = Parametros().enlace
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grupo", nargs="?", choices=sorted(GRUPOS), help="grupo de ganancias (ambos ejes)")
    ap.add_argument("--pan", nargs=2, type=float, metavar=("KP", "KI"), help="sólo pan, con estos valores")
    ap.add_argument("--tilt", nargs=2, type=float, metavar=("KP", "KI"), help="sólo tilt, con estos valores")
    ap.add_argument("--puerto", default=pe.puerto)
    ap.add_argument("--baudios", type=int, default=pe.baudios)
    a = ap.parse_args()
    pedidos = {}
    if a.grupo:
        pedidos = {"pan": GRUPOS[a.grupo]["pan"], "tilt": GRUPOS[a.grupo]["tilt"]}
    for eje in ("pan", "tilt"):
        if getattr(a, eje):
            pedidos[eje] = tuple(getattr(a, eje))
    if not pedidos:
        ap.error("indicar un grupo (A, B) o --pan / --tilt")
    import serial
    with serial.Serial(a.puerto, a.baudios, timeout=0.05) as s:
        time.sleep(1.2)                       #* margen sobre ENLACE_ACTIVO_MS: el firmware vuelve a aceptar texto
        print(f"ESP32 en {a.puerto}:")
        ok = all([fijar(s, eje, kp, ki) for eje, (kp, ki) in pedidos.items()])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
