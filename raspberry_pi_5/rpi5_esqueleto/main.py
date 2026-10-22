#!/usr/bin/env python3
"""
main.py — Programa único de la Raspberry Pi (IEE2913, Grupo 9): lazo externo de seguimiento.

Flujo por cuadro:
  cámara (PI-1) -> detección láser rojo (PI-2..6) -> pose de la plataforma en la exposición
  (telemetría AS5600) -> dirección del objetivo en el mundo (CS-1) -> predicción al instante
  de aplicación (EP-1..5) -> supervisor (SE-2/3) -> θ_ref (+ ω_ff) al ESP32 (CS-2/CS-3)
  -> el ESP32 cierra el lazo de posición con el PI de pid_t_2 (CS-10) -> registro CSV

Ejemplos:
  python3 main.py                                          # sintética, sin motores (PC o RPi)
  python3 main.py --enlace simulado --lampara              # LAZO CERRADO SIMULADO en el PC
  python3 main.py --fuente camara --duracion 20            # cámara real, sin motores
  python3 main.py --fuente camara --enlace trama                         # observar: el ESP32 queda DESHABILITADO
  python3 main.py --fuente camara --enlace trama --habilitar             # LAZO CERRADO REAL (láser rojo)
  python3 main.py --fuente camara --enlace trama --habilitar --detector luz   # ídem con el flash del celular
  python3 main.py --fuente camara --duracion 0 ...         # sin fin (Ctrl+C o SIGTERM para detener)
  python3 main.py ... --monitor                            # tablero web: http://g9-cap.local:8000
"""
#! --habilitar energiza los motores: sólo con el banco armado, la prueba de signos hecha
#! ('t pan' / 't tilt' en la consola del ESP32) y alguien a cargo del corte.
import argparse
import collections
import math
import signal
import statistics as st
import time

import numpy as np

from pt.parametros import Parametros
from pt.geometria import Geometria
from pt.deteccion import DetectorLaser, DetectorLaserRojo, DetectorLuz
from pt.supervisor import Supervisor, Estado
from pt.control import ReferenciaVisual
from pt.enlace import EnlaceNulo, EnlaceTrama, EnlaceSimulado
from pt.registro import Registro


def argumentos():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fuente", choices=["sintetica", "camara"], default="sintetica")
    ap.add_argument("--duracion", type=float, default=10.0, help="segundos (0 = sin fin, sólo con --fuente camara)")
    ap.add_argument("--fps", type=float, help="sobrescribe parametros.Captura.fps")
    ap.add_argument("--exposicion", type=int, help="µs; fija la exposición")
    ap.add_argument("--ganancia", type=float, help="ganancia analógica (luz_2.py usa 2,0)")
    ap.add_argument("--detector", choices=["rojo", "luz", "brillo"], default="rojo",
                    help="rojo = láser rojo (plano V); luz = flash del celular (de luz_2.py); brillo = el original")
    ap.add_argument("--enlace", choices=["nulo", "simulado", "trama"], default="nulo",
                    help="nulo: sin actuadores; simulado: ESP32 y ejes simulados; trama: ESP32 real")
    ap.add_argument("--puerto", help="por defecto /dev/ttyAMA0 (UART del header, pines 8 y 10)")
    ap.add_argument("--baudios", type=int)
    ap.add_argument("--habilitar", action="store_true", help="habilita los drivers del ESP32 (mueve los motores)")
    ap.add_argument("--feedforward", action="store_true", help="envía ω_ff (velocidad estimada del objetivo)")
    ap.add_argument("--tiempo-real", action="store_true", help="fuente sintética al ritmo real de la cámara")
    ap.add_argument("--lampara", action="store_true", help="fuente sintética: agrega una lámpara blanca fija")
    ap.add_argument("--ventana", action="store_true", help="muestra la imagen (requiere OpenCV y pantalla)")
    ap.add_argument("--registro", default="registros", help="carpeta de salida del CSV")
    #* MON-1 / MON-2: tablero web en la red local, en un hilo aparte (sin monitor HDMI).
    red = ap.add_argument_group("monitoreo por red (MON-1 / MON-2)")
    red.add_argument("--monitor", action="store_true",
                     help="tablero web en http://<rpi>.local:<puerto> (sólo lectura)")
    red.add_argument("--monitor-puerto", type=int, default=8000, help="puerto HTTP (por defecto 8000)")
    red.add_argument("--monitor-hz", type=float, default=20.0,
                     help="refrescos por segundo del tablero; cada refresco trae TODOS los cuadros nuevos")
    red.add_argument("--monitor-imagen-hz", type=float, default=5.0,
                     help="cuadros por segundo de la imagen en el tablero (0 = sin imagen)")
    red.add_argument("--monitor-host", default="0.0.0.0",
                     help="0.0.0.0 = toda la red local; 127.0.0.1 = sólo por túnel SSH")
    red.add_argument("--monitor-segundos", type=float, default=120.0,
                     help="historia en memoria que se puede exportar (/historial.csv)")
    return ap.parse_args()


def _sigterm(_signo, _frame):
    #* systemd detiene el servicio con SIGTERM: se convierte en KeyboardInterrupt para pasar
    #* por el mismo cierre ordenado (retener y deshabilitar motores, cerrar cámara y registro).
    raise KeyboardInterrupt


class Ventana:
    """Vista opcional (cada 4 cuadros, fuera de la ruta crítica), al estilo de laser_op.py."""

    def __init__(self, geo):
        import cv2
        self.cv2, self.geo, self.n = cv2, geo, 0
        cv2.namedWindow("Pan-tilt G9", cv2.WINDOW_NORMAL)

    def mostrar(self, img, r, estado, pose, consigna, fps):
        self.n += 1
        if self.n % 4:
            return True
        cv2 = self.cv2
        im = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cx, cy = int(round(self.geo.cx)), int(round(self.geo.cy))
        cv2.drawMarker(im, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 25, 2)
        if r.valido:
            cv2.circle(im, (int(r.u), int(r.w)), 12, (0, 0, 255), 2)
            cv2.line(im, (cx, cy), (int(r.u), int(r.w)), (0, 255, 255), 1)
        if consigna and pose:
            up, wp = self.geo.mundo_a_pix(consigna[0], consigna[1], pose[0], pose[1])
            if 0 <= up < im.shape[1] and 0 <= wp < im.shape[0]:
                cv2.drawMarker(im, (int(up), int(wp)), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 14, 2)
        f = cv2.FONT_HERSHEY_SIMPLEX
        txt = f"{estado.value}  {fps:.0f} fps"
        if pose:
            txt += f"  pan {pose[0]:+.1f}  tilt {pose[1]:+.1f}"
        cv2.putText(im, txt, (10, 25), f, 0.6, (0, 255, 0), 2)
        cv2.putText(im, r.motivo or "detectado", (10, 50), f, 0.5, (0, 255, 255), 1)
        cv2.imshow("Pan-tilt G9", im)
        return (cv2.waitKey(1) & 0xFF) != ord("q")

    def cerrar(self):
        self.cv2.destroyAllWindows()


class MonitorRed:
    """MON-1 / MON-2: publica las variables de seguimiento en un tablero web (pt/monitor_red.py).

    Corre en hilos aparte. El lazo sólo le entrega una tupla por cuadro y, cada pocos cuadros,
    el estado lento y una copia de la imagen. Ver el tablero nunca frena el control.
    """

    def __init__(self, a, P, geo, enl, reg):
        from pt.monitor_red import ServidorMonitor
        self.a, self.geo, self.enl = a, geo, enl
        fps = P.captura.fps
        config = {
            "fps": fps, "ancho": int(P.captura.salida_px[0]), "alto": int(P.captura.salida_px[1]),
            "cx": geo.cx, "cy": geo.cy, "ifov_deg": geo.ifov_x_deg,
            "lim_pan": list(P.lazo.lim_pan_deg), "lim_tilt": list(P.lazo.lim_tilt_deg),
            "fuente": a.fuente, "enlace": a.enlace, "detector": a.detector,
            "habilitar": bool(a.habilitar), "feedforward": bool(P.lazo.usar_feedforward),
            "registro": reg.ruta,
            "omega_max": 180.0,          #? OMEGA_MAX_DEG_S de esp_32/esp32_pan_tilt/include/config.h
        }
        self.srv = ServidorMonitor(puerto=a.monitor_puerto, host=a.monitor_host, hz=a.monitor_hz,
                                   imagen_hz=a.monitor_imagen_hz, segundos=a.monitor_segundos, fps=fps,
                                   config=config, archivos={"registro.csv": lambda: reg.ruta},
                                   telemetria=getattr(enl, "historial_tel", None))
        self.cada_estado = max(1, int(round(fps / max(a.monitor_hz, 1.0))))
        self.cada_imagen = max(1, int(round(fps / a.monitor_imagen_hz))) if a.monitor_imagen_hz > 0 else 0
        self.validos = collections.deque(maxlen=int(2 * fps))      # % de detección en los últimos 2 s
        self.t0_ns = None
        self.ref_px = None
        self.urls = self.srv.iniciar()

    def publicar(self, c, r, estado, pose, med, e, consigna, tel, lat, dpan, dtilt, fps, sup):
        if self.t0_ns is None:
            self.t0_ns = c.t_exp_ns
        nan = float("nan")
        self.validos.append(1 if r.valido else 0)
        ref_pan, ref_tilt = (consigna[0], consigna[1]) if consigna else (nan, nan)
        if consigna and pose:
            self.ref_px = self.geo.mundo_a_pix(consigna[0], consigna[1], pose[0], pose[1])
        self.srv.publicar_fila((
            (c.t_exp_ns - self.t0_ns) * 1e-9, c.k, int(r.valido), r.u, r.w,
            r.u - self.geo.cx if r.valido else nan, r.w - self.geo.cy if r.valido else nan,
            dpan, dtilt,
            pose[0] if pose else nan, pose[1] if pose else nan, med.pan, med.tilt,
            ref_pan, ref_tilt,
            tel.w_pan if tel else nan, tel.w_tilt if tel else nan,
            lat, e.confianza, estado.value))

        if c.k % self.cada_estado == 0:
            self.srv.publicar_estado({
                "t": (c.t_exp_ns - self.t0_ns) * 1e-9, "k": c.k, "fps": fps, "lat_ms": lat,
                "modo": estado.value, "enlace_ok": bool(self.enl.ok()),
                "detectado": bool(r.valido), "motivo": r.motivo or "detectado", "pico": r.pico, "area": r.area,
                "det_pct": 100.0 * sum(self.validos) / max(len(self.validos), 1),
                "conf": e.confianza, "t_sin_med": e.t_sin_medicion,
                "ref_px": list(self.ref_px) if self.ref_px else None,
                "esp32": tel.nombre_estado if tel else None,
                "fallas": int(tel.fallas) if tel else 0,
                "fallas_txt": tel.texto_fallas if tel else "-",
                "esp32_ref": [tel.pan_ref, tel.tilt_ref] if tel else None,
                "textos": list(getattr(self.enl, "textos", []))[-6:],
                "bitacora": [[tt - self.t0_ns * 1e-9, getattr(d, "value", str(d)), getattr(h, "value", str(h)), str(causa)]
                             for (tt, d, h, causa) in list(sup.bitacora)[-8:]],
            })
        if self.cada_imagen and c.k % self.cada_imagen == 0 and self.srv.quiere_imagen():
            self.srv.publicar_imagen(np.ascontiguousarray(c.img).copy())

    def finalizar(self, motivo):
        """El lazo terminó: el tablero queda abierto con el último estado, para exportar."""
        e = dict(self.srv.estado)
        e.update(modo="FINALIZADO", motivo_fin=motivo, enlace_ok=False)
        if e.get("esp32"):
            e["esp32"] = f"{e['esp32']} (último)"           #* enlace ya cerrado: último estado visto
        self.srv.publicar_estado(e)

    def cerrar(self):
        self.srv.detener()


def main():
    a = argumentos()
    signal.signal(signal.SIGTERM, _sigterm)
    P = Parametros()
    if a.fps:
        P.captura.fps = a.fps
    if a.exposicion:
        P.captura.exposicion_us = a.exposicion
    if a.ganancia:
        P.captura.ganancia = a.ganancia
    if a.puerto:
        P.enlace.puerto = a.puerto
    if a.baudios:
        P.enlace.baudios = a.baudios
    if a.feedforward:
        P.lazo.usar_feedforward = True
    if a.enlace == "simulado" and a.fuente != "sintetica":
        raise SystemExit("--enlace simulado sólo tiene sentido con --fuente sintetica")
    if a.monitor and a.fuente == "sintetica" and not a.tiempo_real:
        a.tiempo_real = True             #* con el tablero abierto, la simulación va al ritmo real
        print("Monitor: fuente sintética en tiempo real (--tiempo-real).")

    # --- Geometría y enlace (antes que la fuente: el simulado le da la pose a la escena) -----
    if a.fuente == "camara":
        from pt.camara import FuenteCamara
        fuente = FuenteCamara(P.captura, a.duracion if a.duracion > 0 else None)
        if fuente.info.get("ScalerCrop"):
            P.captura.recorte_sensor = tuple(fuente.info["ScalerCrop"])
        P.captura.salida_px = fuente.info["salida"]
        print("Cámara:", fuente.info)
    geo = Geometria.desde_parametros(P)
    print("Geometría:", geo.resumen())

    if a.enlace == "trama":
        enl = EnlaceTrama(P.enlace, habilitar=a.habilitar)
        print(f"ESP32: {'telemetría OK, estado ' + enl.telemetria.nombre_estado if enl.telemetria else 'SIN TELEMETRÍA'}")
    elif a.enlace == "simulado":
        enl = EnlaceSimulado(P)
    else:
        enl = EnlaceNulo()

    if a.fuente == "sintetica":
        from pt.camara import FuenteSintetica
        W, H = P.captura.salida_px
        kw = {"lampara": (500, 100)} if a.lampara else {}
        if a.enlace == "simulado":
            kw.update(geometria=geo, pose_en=enl.pose_en)
        fuente = FuenteSintetica(W, H, P.captura.fps, a.duracion if a.duracion > 0 else 10.0,
                                 tiempo_real=a.tiempo_real, **kw)

    #* La pose medida se usa con cámara real + ESP32, o en la simulación. Con imagen sintética y
    #* ESP32 real la escena no responde a los motores: se toma la cámara fija en (0°, 0°) y el
    #* ESP32 sigue la dirección del objetivo sintético (prueba del lazo interno con referencia real).
    usar_pose = a.enlace == "simulado" or (a.enlace == "trama" and a.fuente == "camara")
    tiempo_virtual = a.fuente == "sintetica" and not a.tiempo_real
    retardo_ns = int(P.lazo.retardo_enlace_s * 1e9)

    if a.detector == "rojo":
        det = DetectorLaserRojo(P.deteccion_roja)
    elif a.detector == "luz":
        det = DetectorLuz(P.deteccion_luz)
    else:
        det = DetectorLaser(P.deteccion)
    ref = ReferenciaVisual(P, geo)
    sup = Supervisor(P.supervisor, con_actuadores=(a.enlace != "nulo"))
    reg = Registro(a.registro, prefijo=f"{a.fuente}_{a.enlace}")
    vista = Ventana(geo) if a.ventana else None
    mon = MonitorRed(a, P, geo, enl, reg) if a.monitor else None
    if mon:
        print("Monitor (abrir en el navegador de un equipo en la misma red):")
        for u in mon.urls:
            print("   ", u)

    lat_total, t_det, err_det, err_pred, err_centro, n_val = [], [], [], [], [], 0
    estado_ant = sup.estado
    modo_seguro_avisado = False
    k = -1
    t_prev_fps, fps = time.perf_counter(), 0.0
    motivo_fin = "duración cumplida"
    try:
        for c in fuente:
            k = c.k
            t = c.t_exp_ns * 1e-9
            seguimiento = sup.estado == Estado.SEGUIMIENTO

            # --- PI: detección -----------------------------------------------------------------
            r = det.procesar(c.img, usar_ventana=seguimiento, v=c.v)

            # --- CS-1 + EP: objetivo en el mundo, predicho al instante de aplicación ------------
            #* Instante de la fila donde está el objetivo (persiana rodante): ahí se lee la pose.
            t_obj = c.instante(r.w if r.valido else None)
            pose = enl.angulos_en(t_obj) if usar_pose else (0.0, 0.0)
            t_envio = (c.t_entrega_ns + int(r.t_proc_ms * 1e6)) if tiempo_virtual else time.monotonic_ns()
            t_aplic = t_envio + retardo_ns
            med, e = ref.actualizar(t_obj, r, pose, t_aplic)

            # --- SE ----------------------------------------------------------------------------
            estado = sup.paso(t, med.valida, e.confianza, enl.ok(), enl.listo(), enl.en_falla())
            if estado != estado_ant and estado in (Estado.BUSQUEDA, Estado.PERDIDA):
                det.reiniciar()                  # la ventana vuelve al cuadro completo
                if estado == Estado.BUSQUEDA:
                    ref.reiniciar()
            estado_ant = estado

            # --- CS-2 / CS-3: consigna al ESP32 --------------------------------------------------
            consigna = None
            if estado == Estado.MODO_SEGURO:
                if not modo_seguro_avisado:
                    enl.retener()                #! la RPi deja de mandar consignas; el ESP32 retiene
                    modo_seguro_avisado = True
            elif estado not in (Estado.INICIALIZACION, Estado.HOMING):
                consigna = ref.consigna(estado == Estado.SEGUIMIENTO, e, pose)
                if consigna is not None:
                    enl.enviar_consigna(*consigna, t_ns=t_envio)

            # --- Métricas ----------------------------------------------------------------------
            t_fin = time.monotonic_ns()
            lat = (t_fin - c.t_exp_ns) / 1e6 if not tiempo_virtual else \
                  (c.t_entrega_ns - c.t_exp_ns) / 1e6 + r.t_proc_ms
            lat_total.append(lat)
            t_det.append(r.t_proc_ms)

            ur, wr = (c.verdad if c.verdad else (float("nan"), float("nan")))
            dpan, dtilt = geo.pix_a_ang(r.u, r.w) if r.valido else (float("nan"), float("nan"))
            e_centro = float("nan")
            if r.valido:
                n_val += 1
                if c.verdad:
                    err_det.append(math.hypot(r.u - ur, r.w - wr))
            if a.enlace == "simulado" and c.verdad and estado == Estado.SEGUIMIENTO:
                e_centro = math.hypot(ur - geo.cx, wr - geo.cy)        # CTL-5: objetivo centrado
                err_centro.append(e_centro)
            if a.fuente == "sintetica" and a.enlace == "nulo" and estado == Estado.SEGUIMIENTO \
                    and r.valido and not math.isnan(e.u):
                # posición real en el instante de aplicación vs (a) predicción y (b) última medición
                uh, wh = fuente.trayectoria((t_aplic - fuente.t0_ns) * 1e-9)
                up, wp = geo.mundo_a_pix(e.u, e.w, 0.0, 0.0)
                err_pred.append((math.hypot(up - uh, wp - wh), math.hypot(r.u - uh, r.w - wh)))

            tel = getattr(enl, "telemetria", None)
            reg.fila(k=k, t_exp_ns=c.t_exp_ns, t_entrega_ns=c.t_entrega_ns, t_fin_ns=t_fin,
                     lat_captura_ms=(c.t_entrega_ns - c.t_exp_ns) / 1e6, lat_total_ms=lat,
                     t_det_ms=r.t_proc_ms, estado=estado.value, valido=int(r.valido), motivo=r.motivo,
                     u=r.u, w=r.w, pico=r.pico, area=r.area, dpan_deg=dpan, dtilt_deg=dtilt,
                     pan_med=pose[0] if pose else float("nan"), tilt_med=pose[1] if pose else float("nan"),
                     pan_obj=med.pan, tilt_obj=med.tilt, pan_pred=e.u, tilt_pred=e.w,
                     vpan_pred=e.vu, vtilt_pred=e.vw, confianza=e.confianza, t_sin_med_s=e.t_sin_medicion,
                     est_u=e.estado_u, est_w=e.estado_w,
                     pan_ref=consigna[0] if consigna else float("nan"),
                     tilt_ref=consigna[1] if consigna else float("nan"),
                     wff_pan=consigna[2] if consigna else float("nan"),
                     wff_tilt=consigna[3] if consigna else float("nan"),
                     h_ms=(t_aplic - c.t_exp_ns) / 1e6,
                     esp32_estado=tel.nombre_estado if tel else "", esp32_fallas=tel.fallas if tel else "",
                     esp32_pan_med=tel.pan_med if tel else float("nan"),
                     esp32_tilt_med=tel.tilt_med if tel else float("nan"),
                     esp32_pan_ref=tel.pan_ref if tel else float("nan"),
                     esp32_tilt_ref=tel.tilt_ref if tel else float("nan"),
                     esp32_w_pan=tel.w_pan if tel else float("nan"),
                     esp32_w_tilt=tel.w_tilt if tel else float("nan"),
                     u_real=ur, w_real=wr, err_centro_px=e_centro)

            ahora = time.perf_counter()
            fps = 0.9 * fps + 0.1 / max(ahora - t_prev_fps, 1e-6)
            t_prev_fps = ahora
            if mon:
                mon.publicar(c, r, estado, pose, med, e, consigna, tel, lat, dpan, dtilt, fps, sup)
            if vista:
                if not vista.mostrar(c.img, r, estado, pose, consigna, fps):
                    motivo_fin = "ventana cerrada"
                    break
    except KeyboardInterrupt:
        print("\nInterrumpido.")
        motivo_fin = "interrumpido"
    finally:
        enl.cerrar()
        fuente.cerrar()
        reg.cerrar()
        if vista:
            vista.cerrar()
        if mon:
            mon.finalizar(motivo_fin)

    # --- Resumen -----------------------------------------------------------------------------
    n = k + 1
    print(f"\nCuadros: {n} | detecciones válidas: {n_val} ({100*n_val/max(n,1):.1f} %) | detector: {a.detector}")
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
        print(f"Posición al aplicar la consigna — error RMS con predictor: {rms[0]:.2f} px | "
              f"sin predictor (última medición): {rms[1]:.2f} px")
    if err_centro:
        ec = np.array(err_centro)
        ifov = geo.ifov_x_deg
        print(f"Error de centrado en SEGUIMIENTO (lazo cerrado simulado): RMS {np.sqrt(np.mean(ec**2)):.2f} px "
              f"({np.sqrt(np.mean(ec**2))*ifov:.3f}°), p95 {np.percentile(ec, 95):.2f} px, máx {ec.max():.2f} px")
    print("Transiciones del supervisor:")
    for (tt, d, h, causa) in sup.bitacora:
        print(f"  t={tt - (ref.est.t0 or 0):7.3f} s  {d:>15} -> {h:<12} ({causa})")
    tel = getattr(enl, "telemetria", None)
    if tel:
        print(f"ESP32 al final: {tel.nombre_estado}, fallas: {tel.texto_fallas}")
    for linea in list(getattr(enl, "textos", []))[-10:]:
        print(f"  ESP32 {linea}")
    print(f"Registro: {reg.ruta}")

    #* Con --monitor el tablero queda abierto al terminar (motores ya deshabilitados), para
    #* revisar y exportar los datos desde el navegador. Se cierra con Ctrl+C.
    if mon:
        if motivo_fin != "interrumpido":
            print(f"\nMonitor sigue abierto en {mon.urls[0]} para exportar los datos. Ctrl+C para cerrarlo.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
        mon.cerrar()


if __name__ == "__main__":
    main()
