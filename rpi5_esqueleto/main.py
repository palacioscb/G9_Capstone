#!/usr/bin/env python3
"""
main.py — Esqueleto del software de la Raspberry Pi (IEE2913, Grupo 9).

Flujo por cuadro:
  cámara (PI-1) -> detección láser (PI-2..6) -> estimación/predicción (EP-1..5)
  -> supervisor (SE-2/3) -> píxel->ángulo (CS-1) -> ley de control (CS-2, interfaz)
  -> enlace al ESP32 (CS-3) -> registro CSV

Ejemplos:
  python3 main.py                                   # fuente sintética, sin motores (PC o RPi)
  python3 main.py --fuente camara --duracion 20     # cámara real, sin motores
  python3 main.py --fuente camara --enlace cli --puerto /dev/ttyUSB0 --control demo --habilitar
"""
import argparse
import math
import statistics as st
import time

import numpy as np

from pt.parametros import Parametros
from pt.geometria import Geometria
from pt.deteccion import DetectorLaser
from pt.prediccion import Estimador2D
from pt.supervisor import Supervisor, Estado
from pt.control import ControlNulo, ControlProporcionalDemo, omega_a_pasos
from pt.enlace import EnlaceNulo, EnlaceCLI
from pt.registro import Registro


def argumentos():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fuente", choices=["sintetica", "camara"], default="sintetica")
    ap.add_argument("--duracion", type=float, default=10.0, help="segundos")
    ap.add_argument("--fps", type=float, help="sobrescribe parametros.Captura.fps")
    ap.add_argument("--exposicion", type=int, help="µs; fija la exposición (láser)")
    ap.add_argument("--enlace", choices=["nulo", "cli"], default="nulo")
    ap.add_argument("--puerto", help="p. ej. /dev/ttyUSB0 o /dev/serial/by-id/...")
    ap.add_argument("--baudios", type=int)
    ap.add_argument("--control", choices=["nulo", "demo"], default="nulo")
    ap.add_argument("--habilitar", action="store_true", help="envía 'e 1' al firmware (energiza el motor)")
    ap.add_argument("--tiempo-real", action="store_true", help="fuente sintética al ritmo real de la cámara")
    ap.add_argument("--registro", default="registros", help="carpeta de salida del CSV")
    return ap.parse_args()


def main():
    a = argumentos()
    P = Parametros()
    if a.fps:
        P.captura.fps = a.fps
    if a.exposicion:
        P.captura.exposicion_us = a.exposicion
    if a.puerto:
        P.enlace.puerto = a.puerto
    if a.baudios:
        P.enlace.baudios = a.baudios

    # --- Fuente de imágenes (PI-1) --------------------------------------
    if a.fuente == "camara":
        from pt.camara import FuenteCamara
        fuente = FuenteCamara(P.captura, a.duracion)
        if fuente.info.get("ScalerCrop"):
            P.captura.recorte_sensor = tuple(fuente.info["ScalerCrop"])
        P.captura.salida_px = fuente.info["salida"]
        print("Cámara:", fuente.info)
    else:
        from pt.camara import FuenteSintetica
        W, H = P.captura.salida_px
        fuente = FuenteSintetica(W, H, P.captura.fps, a.duracion, tiempo_real=a.tiempo_real)

    geo = Geometria.desde_parametros(P)
    print("Geometría:", geo.resumen())

    det = DetectorLaser(P.deteccion)
    est = Estimador2D(P.prediccion)
    sup = Supervisor(P.supervisor, con_actuadores=(a.enlace != "nulo"))
    ctl = ControlProporcionalDemo() if a.control == "demo" else ControlNulo()
    enl = EnlaceCLI(P.enlace, habilitar=a.habilitar) if a.enlace == "cli" else EnlaceNulo()
    reg = Registro(a.registro, prefijo=a.fuente)

    t_prev = None
    lat_total, t_det, err_det, err_pred, n_val = [], [], [], [], 0
    estado_ant = sup.estado
    k = -1
    try:
        for c in fuente:
            k = c.k
            t = c.t_exp_ns * 1e-9
            seguimiento = sup.estado == Estado.SEGUIMIENTO

            r = det.procesar(c.img, usar_ventana=seguimiento)                     # PI
            e = est.actualizar(t, r)                                              # EP
            estado = sup.paso(t, r.valido, e.confianza, enl.ok())                 # SE

            if estado != estado_ant and estado in (Estado.BUSQUEDA, Estado.PERDIDA):
                det.reiniciar()                  # la ventana vuelve al cuadro completo
                if estado == Estado.BUSQUEDA:
                    est.reiniciar()
            estado_ant = estado

            # CS-1: error de centrado -> ángulo (sobre la posición PREDICHA)
            if estado == Estado.SEGUIMIENTO and not math.isnan(e.u):
                dpan, dtilt = geo.pix_a_ang(e.u, e.w)
            else:
                dpan, dtilt = 0.0, 0.0

            # CS-2 (interfaz) y CS-3
            dt = (t - t_prev) if t_prev is not None else 0.0
            t_prev = t
            wp, wt = ctl.calcular(dpan, dtilt, dt) if estado == Estado.SEGUIMIENTO else (0.0, 0.0)
            pasos = omega_a_pasos(wp, P.actuador)
            enl.enviar_velocidad(pasos)

            t_fin = time.monotonic_ns()
            lat = (t_fin - c.t_exp_ns) / 1e6 if a.fuente == "camara" or a.tiempo_real else \
                  (c.t_entrega_ns - c.t_exp_ns) / 1e6 + r.t_proc_ms
            lat_total.append(lat)
            t_det.append(r.t_proc_ms)

            # Evaluación contra la verdad (sólo sintética)
            ur, wr = (c.verdad if c.verdad else (float("nan"), float("nan")))
            if r.valido:
                n_val += 1
                if c.verdad:
                    err_det.append(math.hypot(r.u - ur, r.w - wr))
            if a.fuente == "sintetica" and estado == Estado.SEGUIMIENTO and r.valido \
                    and not math.isnan(e.u):
                # posición real en t + h vs. (a) predicción y (b) "sin predictor" (última medición)
                ts = t + P.prediccion.horizonte_s - fuente.t0_ns * 1e-9
                uh, wh = fuente.trayectoria(ts)
                err_pred.append((math.hypot(e.u - uh, e.w - wh), math.hypot(r.u - uh, r.w - wh)))

            reg.fila(k=k, t_exp_ns=c.t_exp_ns, t_entrega_ns=c.t_entrega_ns, t_fin_ns=t_fin,
                     lat_captura_ms=(c.t_entrega_ns - c.t_exp_ns) / 1e6, lat_total_ms=lat,
                     t_det_ms=r.t_proc_ms, estado=estado.value, valido=int(r.valido), motivo=r.motivo,
                     u=r.u, w=r.w, pico=r.pico, area=r.area, u_pred=e.u, w_pred=e.w,
                     confianza=e.confianza, t_sin_med_s=e.t_sin_medicion, est_u=e.estado_u,
                     est_w=e.estado_w, dpan_deg=dpan, dtilt_deg=dtilt, omega_pan=wp,
                     omega_tilt=wt, pasos_s=pasos, u_real=ur, w_real=wr)
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        enl.cerrar()
        fuente.cerrar()
        reg.cerrar()

    # --- Resumen -----------------------------------------------------------
    n = k + 1
    print(f"\nCuadros: {n} | detecciones válidas: {n_val} ({100*n_val/max(n,1):.1f} %)")
    if t_det:
        print(f"Detección: mediana {st.median(t_det):.2f} ms, p95 {np.percentile(t_det, 95):.2f} ms")
        print(f"Latencia exposición->consigna: mediana {st.median(lat_total):.1f} ms, "
              f"p95 {np.percentile(lat_total, 95):.1f} ms")
    if err_det:
        print(f"Error de detección vs verdad: RMS {np.sqrt(np.mean(np.square(err_det))):.3f} px, "
              f"máx {max(err_det):.2f} px")
    if err_pred:
        ep = np.array(err_pred)
        rms = np.sqrt(np.mean(ep ** 2, axis=0))
        h = P.prediccion.horizonte_s * 1e3
        print(f"Posición a t+{h:.0f} ms — error RMS con predictor: {rms[0]:.2f} px | "
              f"sin predictor (última medición): {rms[1]:.2f} px")
    print("Transiciones del supervisor:")
    for (tt, d, h, causa) in sup.bitacora:
        print(f"  t={tt - (est.t0 or 0):7.3f} s  {d:>15} -> {h:<12} ({causa})")
    print(f"Registro: {reg.ruta}")


if __name__ == "__main__":
    main()
