#!/usr/bin/env python3
"""
prueba_enlace.py — Mide el enlace RPi <-> ESP32 con el firmware PoC actual.

Envía N veces el comando 'p' (estado) y mide el tiempo de ida y vuelta hasta
recibir la línea de respuesta. NO habilita el driver ni mueve el motor.

Uso (en la RPi, con el ESP32 conectado por USB):
    ls /dev/serial/by-id/                         # identificar el puerto
    python3 herramientas/prueba_enlace.py --puerto /dev/ttyUSB0 -n 200                   # esp32_pan_tilt (921 600)
    python3 herramientas/prueba_enlace.py --puerto /dev/ttyUSB0 --baudios 115200 -n 200  # firmware PoC

Salida: RTT mediana / p95 / máx [ms], respuestas perdidas, y CSV por muestra.
"""
#* Con esp32_pan_tilt, 'p' responde varias líneas: se mide hasta la primera.
import argparse
import csv
import statistics as st
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--baudios", type=int, default=921600)
    ap.add_argument("-n", type=int, default=200)
    ap.add_argument("--comando", default="p", help="comando que responde una línea")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--csv", default="rtt_enlace.csv")
    a = ap.parse_args()

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = a.puerto, a.baudios, a.timeout
    s.dtr = False            # no reiniciar el ESP32 al abrir el puerto
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    rtts, perdidas, filas = [], 0, []
    for i in range(a.n):
        s.reset_input_buffer()
        t0 = time.perf_counter_ns()
        s.write((a.comando + "\n").encode())
        linea = s.readline()
        t1 = time.perf_counter_ns()
        if not linea.endswith(b"\n"):
            perdidas += 1
            filas.append((i, "", 0))
            continue
        rtt = (t1 - t0) / 1e6
        rtts.append(rtt)
        filas.append((i, f"{rtt:.3f}", len(linea)))
        time.sleep(0.01)
    s.close()

    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["i", "rtt_ms", "bytes_respuesta"])
        w.writerows(filas)

    if rtts:
        rtts.sort()
        p95 = rtts[int(0.95 * (len(rtts) - 1))]
        print(f"{a.baudios} baudios | n={len(rtts)} | RTT mediana {st.median(rtts):.2f} ms | "
              f"p95 {p95:.2f} ms | máx {max(rtts):.2f} ms | perdidas {perdidas}")
        bytes_prom = st.mean(b for (_, r, b) in filas if r)
        t_tx = (len(a.comando) + 1 + bytes_prom) * 10 / a.baudios * 1e3
        print(f"Tiempo teórico de transmisión (10 bits/byte): {t_tx:.2f} ms "
              f"-> el resto es latencia USB + procesamiento del ESP32")
    else:
        print("Sin respuestas: revisa puerto, baudios y que el firmware esté corriendo.")
    print(f"Detalle: {a.csv}")


if __name__ == "__main__":
    main()
