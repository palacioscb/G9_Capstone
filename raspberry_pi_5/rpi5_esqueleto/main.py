#!/usr/bin/env python3
"""
main.py — Programa único de la Raspberry Pi (IEE2913, Grupo 9): lazo externo de seguimiento.

Flujo por cuadro:
  cámara (PI-1) -> detección láser rojo (PI-2..6) -> pose de la plataforma en la exposición
  (telemetría AS5600) -> dirección del objetivo en el mundo (CS-1) -> predicción al instante
  de aplicación (EP-1..5) -> supervisor (SE-2/3) -> θ_ref (+ ω_ff) al ESP32 (CS-2/CS-3)
  -> el ESP32 cierra el lazo de posición con el PI de pid_t_2 (CS-10) -> registro CSV

Uso normal (19-11-2026): se ejecuta UNA vez y todo lo demás se hace desde el tablero.
  python3 main.py                  # en la RPi: cámara + ESP32 + tablero (http://g9-cap.local:8000), sin fin.
                                   # Los motores parten CORTADOS; se habilitan, se cortan y se rearman desde
                                   # el tablero, igual que el objetivo, las ganancias, el cero y las grabaciones.
  python3 main.py                  # en el PC (sin picamera2): fuente sintética, sin ESP32, 10 s
Otras formas:
  python3 main.py --enlace simulado --lampara --monitor    # LAZO CERRADO SIMULADO en el PC, con tablero
  python3 main.py --detector avion --habilitar             # partir con ese objetivo y los motores habilitados
  python3 main.py --enlace nulo                            # sólo cámara y detección, sin ESP32
  python3 main.py --duracion 20 --sin-monitor              # corrida corta sin tablero
"""
#! Los motores se pueden habilitar desde el tablero (decisión del equipo, 19-11-2026): hacerlo
#! con el banco armado, la prueba de signos hecha y alguien a cargo de "Cortar motores".
import argparse
import collections
import math
import os
import signal
import statistics as st
import sys
import time

import numpy as np

from pt.parametros import Parametros, PERFILES
from pt.geometria import Geometria
from pt.deteccion import DetectorLaser, DetectorLaserRojo, DetectorLuz, DetectorAvionRojo, DetectorForma
from pt.supervisor import Supervisor, Estado
from pt.control import ReferenciaVisual
from pt.enlace import EnlaceNulo, EnlaceTrama, EnlaceSimulado
from pt.registro import Registro, Grabacion

#* Objetivos que se pueden elegir (opción --detector y tablero).
DETECTORES = ("rojo", "luz", "avion", "ovni", "avion_rojo", "brillo")
#* Con la protección de salto desactivada (ajuste "<detector>.salto_activo" del tablero), salto_max_px vale esto:
#* un valor finito y enorme, no inf, para que la configuración siga siendo JSON válido.
SALTO_DESACTIVADO = 1e9


def crear_detectores(P, ruta_formas=None):
    """Todos los detectores creados de una vez: cambiar de objetivo es elegir otro."""
    d = {"rojo": DetectorLaserRojo(P.deteccion_roja), "luz": DetectorLuz(P.deteccion_luz),
         "avion_rojo": DetectorAvionRojo(P), "brillo": DetectorLaser(P.deteccion)}
    try:
        from pt.formas import BancoDeFormas
        banco = BancoDeFormas(ruta_formas)
        d["avion"] = DetectorForma(P.forma_avion, banco)        #* por FORMA, sobre el plano Y
        d["ovni"] = DetectorForma(P.forma_ovni, banco)
    except ImportError as ex:
        #! Los detectores por forma necesitan OpenCV (sudo apt install python3-opencv).
        print(f"AVISO: sin detectores por forma (avion, ovni): {ex}")
    return d


def argumentos():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fuente", choices=["sintetica", "camara"],
                    help="por defecto: camara si esta máquina tiene picamera2 (la RPi); si no, sintetica")
    ap.add_argument("--duracion", type=float,
                    help="segundos; 0 = sin fin. Por defecto: sin fin con la cámara, 10 s con la fuente sintética")
    ap.add_argument("--fps", type=float, help="sobrescribe parametros.Captura.fps")
    ap.add_argument("--exposicion", type=int, help="µs; fija la exposición")
    ap.add_argument("--ganancia", type=float, help="ganancia analógica (luz_2.py usa 2,0)")
    ap.add_argument("--detector", choices=list(DETECTORES), default="rojo",
                    help="rojo = láser rojo (plano V); luz = flash del celular; avion = avión por FORMA (plano Y); "
                         "ovni = objeto circular por forma; avion_rojo = avión por color; brillo = original")
    ap.add_argument("--objeto", choices=["punto", "circulo"], default="punto",
                    help="fuente sintética: qué se dibuja. punto = láser; circulo = silueta oscura sobre fondo claro "
                         "(para --detector ovni). El avión no se simula: se prueba con cuadros reales")
    ap.add_argument("--enlace", choices=["nulo", "simulado", "trama"],
                    help="nulo: sin actuadores; simulado: ESP32 y ejes simulados; trama: ESP32 real. "
                         "Por defecto: trama con la cámara (si no hay puerto serie, sigue sin ESP32) y nulo con la sintética")
    ap.add_argument("--puerto", help="por defecto /dev/ttyAMA0 (UART del header, pines 8 y 10)")
    ap.add_argument("--baudios", type=int)
    ap.add_argument("--habilitar", action="store_true",
                    help="habilita los motores AL PARTIR. Sin esto parten cortados y se habilitan desde el tablero")
    ap.add_argument("--feedforward", action="store_true", help="envía ω_ff (velocidad estimada del objetivo)")
    ap.add_argument("--ganancias-sim", nargs=4, type=float, metavar=("KP_PAN", "KI_PAN", "KP_TILT", "KI_TILT"),
                    help="sólo con --enlace simulado: ganancias del PI simulado (por defecto, las de config.h)")
    ap.add_argument("--tiempo-real", action="store_true", help="fuente sintética al ritmo real de la cámara")
    ap.add_argument("--lampara", action="store_true", help="fuente sintética: agrega una lámpara blanca fija")
    ap.add_argument("--ventana", action="store_true", help="muestra la imagen (requiere OpenCV y pantalla)")
    ap.add_argument("--sin-recorte", action="store_true",
                    help="sin la herramienta de recorte del tablero (pt/recorte.py): ni controles ni código en el lazo, como antes del 18-11")
    ap.add_argument("--sin-perfil", action="store_true",
                    help="no aplicar el perfil del objetivo (perfiles.json) ni al partir con --fuente camara "
                         "ni al cambiar de detector en el tablero; los botones de perfil siguen disponibles")
    ap.add_argument("--registro", default="registros", help="carpeta de salida del CSV")
    #* MON-1 / MON-2: tablero web en la red local, en un hilo aparte (sin monitor HDMI).
    red = ap.add_argument_group("monitoreo por red (MON-1 / MON-2)")
    red.add_argument("--monitor", action="store_true",
                     help="tablero web en http://<rpi>.local:<puerto>, con ajustes y mando en vivo. "
                          "Con la cámara va por defecto")
    red.add_argument("--sin-monitor", action="store_true", help="sin tablero web")
    red.add_argument("--sin-mando", action="store_true",
                     help="el tablero no puede mandar al ESP32 (habilitar motores, ganancias, cero, referencia de "
                          "prueba) ni grabar experimentos; conserva los ajustes en vivo")
    red.add_argument("--monitor-solo-lectura", action="store_true",
                     help="el tablero no puede cambiar parámetros (como antes del 17-11)")
    red.add_argument("--monitor-puerto", type=int, default=8000, help="puerto HTTP (por defecto 8000)")
    red.add_argument("--monitor-hz", type=float, default=20.0,
                     help="refrescos por segundo del tablero; cada refresco trae TODOS los cuadros nuevos")
    red.add_argument("--monitor-imagen-hz", type=float, default=5.0,
                     help="cuadros por segundo de la imagen en el tablero (0 = sin imagen)")
    red.add_argument("--monitor-host", default="0.0.0.0",
                     help="0.0.0.0 = toda la red local; 127.0.0.1 = sólo por túnel SSH")
    red.add_argument("--monitor-segundos", type=float, default=120.0,
                     help="historia en memoria que se puede exportar (/historial.csv)")
    return resolver_por_defecto(ap.parse_args())


def resolver_por_defecto(a):
    """Lo que no se escribe en la línea de comandos se decide según dónde corre (19-11-2026):
    en la RPi, 'python3 main.py' a secas es cámara + ESP32 + tablero, sin fin; en el PC, la
    fuente sintética sin actuadores, como siempre. Lo escrito a mano manda."""
    import importlib.util
    a.enlace_pedido = a.enlace                   # None = enlace por defecto (no es error que falte el puerto)
    if a.fuente is None:
        hay_camara = importlib.util.find_spec("picamera2") is not None
        a.fuente = "camara" if (hay_camara and a.enlace != "simulado") else "sintetica"
    if a.enlace is None:
        a.enlace = "trama" if a.fuente == "camara" else "nulo"
    if a.duracion is None:
        a.duracion = 0.0 if a.fuente == "camara" else 10.0
    a.monitor = (a.monitor or a.fuente == "camara") and not a.sin_monitor
    return a


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


#* Archivo donde el tablero guarda los perfiles por objetivo ("Guardar perfil").
RUTA_PERFILES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "perfiles.json")
#* Siluetas aprendidas con herramientas/aprender_forma.py (vistas del avión real).
RUTA_FORMAS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "formas.json")


def lista_de_ajustes(a, P, con_camara):
    """Parámetros que el tablero puede cambiar con el lazo corriendo (pt/ajustes.py).
    Cada uno lleva su rango: el servidor rechaza lo que esté fuera, antes de que llegue al lazo."""
    from pt.ajustes import Ajuste
    c, pr = P.captura, P.prediccion
    L = []
    if con_camara:
        #? Tope de exposición = período de cuadro (12 500 µs a 80 fps): los fps están fijos, así
        #? que libcamera recorta cualquier valor mayor.
        L += [Ajuste("exposicion_us", "Exposición", "Cámara", "int", int(c.exposicion_us), 100, int(1e6 / c.fps), 100, "µs",
                     ayuda="Más exposición: imagen más clara, pero más desenfoque de movimiento."),
              Ajuste("ganancia", "Ganancia analógica", "Cámara", "float", float(c.ganancia), 1.0, 10.0, 0.1, "×",
                     ayuda="Aclara sin alargar la exposición; sube el ruido.")]
        if c.ganancias_color:
            L += [Ajuste("color_rojo", "Ganancia de color rojo", "Cámara", "float", float(c.ganancias_color[0]),
                         0.5, 4.0, 0.05, "×", ayuda="Balance de blancos fijo. Sube el nivel del plano V (láser y avión)."),
                  Ajuste("color_azul", "Ganancia de color azul", "Cámara", "float", float(c.ganancias_color[1]),
                         0.5, 4.0, 0.05, "×")]
    L += [
        Ajuste("detector", "Detector", "Objetivo", "opcion", a.detector, opciones=DETECTORES,
               ayuda="rojo = láser; luz = flash; avion = avión por forma; ovni = objeto circular por forma; "
                     "avion_rojo = avión por color; brillo = pico de brillo (original). "
                     "Al cambiarlo se aplica el perfil de ese objetivo (cámara, umbrales y predictor)."),
        Ajuste("rojo.v_umbral", "Umbral de rojo (plano V)", "Detector", "int", P.deteccion_roja.v_umbral, 129, 255, 1,
               detector="rojo", ayuda="128 = neutro, 255 = rojo puro."),
        Ajuste("rojo.y_min", "Brillo mínimo (plano Y)", "Detector", "int", P.deteccion_roja.y_min, 0, 255, 1,
               detector="rojo", ayuda="0 = sin confirmación de brillo."),
        Ajuste("luz.umbral", "Umbral de brillo", "Detector", "int", P.deteccion_luz.umbral, 1, 255, 1, detector="luz"),
        Ajuste("luz.min_pix", "Píxeles mínimos", "Detector", "int", P.deteccion_luz.min_pix, 1, 500, 1, "px", detector="luz"),
        *[aj for nombre, pf in (("avion", P.forma_avion), ("ovni", P.forma_ovni)) for aj in (
            Ajuste(f"{nombre}.polaridad", "Objetivo respecto del fondo", "Detector", "opcion", pf.polaridad,
                   opciones=("oscuro", "claro", "ambos"), detector=nombre,
                   ayuda="oscuro = más oscuro que lo que lo rodea; claro = más claro. 'ambos' cuesta el doble y da más falsos."),
            Ajuste(f"{nombre}.contraste_min", "Contraste mínimo", "Detector", "int", pf.contraste_min, 5, 200, 1,
                   detector=nombre, ayuda="Diferencia de gris con el entorno. Bajarlo si el objetivo se ve tenue; subirlo si aparecen sombras."),
            Ajuste(f"{nombre}.dist_max", "Parecido exigido (distancia máx.)", "Detector", "float", float(pf.dist_max), 0.02, 0.60, 0.01,
                   detector=nombre, ayuda="0 = idéntico a la plantilla. Más alto acepta más vistas, y más figuras ajenas. El tablero muestra la distancia medida."),
            Ajuste(f"{nombre}.tam_max_px", "Ancho máximo del objetivo", "Detector", "int", pf.tam_max_px, 21, 201, 2, "px",
                   detector=nombre, ayuda="El objetivo debe ser más angosto que esto en su parte más gruesa."),
            Ajuste(f"{nombre}.area_min", "Área mínima", "Detector", "int", pf.area_min, 30, 5000, 10, "px",
                   detector=nombre),
            Ajuste(f"{nombre}.alargamiento_max", "Alargamiento máximo (largo/ancho)", "Detector", "float", float(pf.alargamiento_max),
                   2.0, 20.0, 0.5, detector=nombre,
                   ayuda="Se descartan las figuras más alargadas que esto (motivo «figuras muy alargadas»). De lejos las alas se pierden y el "
                         "avión queda como un palo largo: subirlo. Muy alto deja pasar palos y bordes."),
            Ajuste(f"{nombre}.frac_borde", "Dónde se toma el borde", "Detector", "float", float(pf.frac_borde), 0.2, 0.9, 0.05,
                   detector=nombre,
                   ayuda="El borde se toma donde el contraste cae a esta fracción de su máximo. Más bajo: las partes delgadas o desenfocadas "
                         "(alas de lejos) se conservan y la figura sale más gruesa. Más alto: más fina."),
            Ajuste(f"{nombre}.abrir_frac", "Quitar lo delgado (palo)", "Detector", "float", float(pf.abrir_frac), 0.0, 0.30, 0.01,
                   detector=nombre,
                   ayuda="Borra lo más delgado que esta fracción de √área (el palo que sostiene al avión). 0 = no borra nada. "
                         "Muy alto se come las alas."),
        )],
        *[Ajuste(f"{nombre}.salto_activo", "Rechazar saltos entre cuadros", "Detector", "bool", True,
                 detector=nombre,
                 ayuda="Activa: se descarta una detección que cae a más de salto_max_px (80 px) de la anterior. "
                       "Desactivarla para probar y ajustar sin esa protección. El estimador (gating) sigue activo.")
          for nombre in DETECTORES],
        Ajuste("avion_rojo.v_umbral", "Umbral de rojo (plano V)", "Detector", "int", P.deteccion_avion.v_umbral, 129, 255, 1,
               detector="avion_rojo", ayuda="128 = neutro, 255 = rojo puro."),
        Ajuste("avion_rojo.min_pix", "Píxeles mínimos (plano V)", "Detector", "int", P.deteccion_avion.min_pix, 1, 500, 1, "px",
               detector="avion_rojo"),
        Ajuste("brillo.umbral_abs", "Umbral de brillo", "Detector", "int", P.deteccion.umbral_abs, 1, 255, 1,
               detector="brillo"),
        #? Piso de 0,60: con orden 3 a 80 fps, más abajo la predicción amplifica el ruido y la traza
        #? de P se acerca al tope de rls_predictor.py (ver docs/prediccion_rls.pdf, sección 4).
        Ajuste("lam", "Factor de olvido λ", "Predicción", "float", float(pr.lam), 0.60, 0.99, 0.01,
               ayuda="Más alto: filtra más el ruido y reacciona más lento. Memoria ≈ 1/(1−λ) cuadros."),
        Ajuste("sigma_min_deg", "Piso del gating (σ mín.)", "Predicción", "float", float(pr.sigma_min_deg), 0.02, 1.0, 0.01, "°",
               ayuda="Una medición se rechaza si se aleja más de 6 veces este valor de lo previsto. Subirlo si el centroide del objetivo salta (mancha grande o forma que cambia): 6 veces el piso debe superar ese salto."),
        Ajuste("reinicio_informado", "Reinicio sin pasar por PÉRDIDA", "Predicción", "bool", bool(pr.reinicio_informado),
               en_perfil=False, ayuda="Tras 3 rechazos seguidos, el estimador nuevo parte con esas mediciones. Apagado: como antes del 17-11."),
        Ajuste("gating_conjunto", "Gating conjunto (pan y tilt)", "Predicción", "bool", bool(pr.gating_conjunto),
               en_perfil=False, ayuda="La medición se acepta o se rechaza completa, y la confianza mira los dos ejes. Apagado: cada eje decide solo."),
        Ajuste("t_acel_max_s", "Oclusión: tope de la aceleración", "Predicción", "float", float(pr.t_acel_max_s), 0.0, 0.30, 0.01, "s",
               en_perfil=False, ayuda="0 = apagado. Mayor que 0: pasado ese tiempo sin medición, se extrapola a velocidad constante. Sólo conviene con objetivos sin aceleración."),
        Ajuste("orden", "Orden del RLS", "Predicción", "opcion", int(pr.orden), opciones=(2, 3), en_perfil=False,
               ayuda="2 = posición y velocidad; 3 = además aceleración. Cambiarlo reinicia el estimador."),
        Ajuste("feedforward", "Feedforward de velocidad (ω_ff)", "Predicción", "bool", bool(P.lazo.usar_feedforward),
               en_perfil=False, ayuda="Envía al ESP32 la velocidad estimada del objetivo, además de θ_ref."),
    ]
    if not getattr(a, "sin_recorte", False):
        #* Recorte de la zona de trabajo (pt/recorte.py). Parte APAGADO; sólo cambia lo que ve el detector.
        #* No va en los perfiles: depende de dónde está montada la cámara, no del objetivo.
        from pt.recorte import RecorteROI
        An, Al = int(P.captura.salida_px[0]), int(P.captura.salida_px[1])
        ini = RecorteROI(An, Al).coords
        L += [Ajuste("roi.activo", "Recorte activo", "Recorte de imagen", "bool", False, en_perfil=False,
                     ayuda="El detector sólo ve lo que está dentro del cuadrilátero; lo de afuera se rellena con el "
                           "gris del fondo. Desmarcar = como antes. El cuadrilátero se dibuja sobre la imagen (botón «Dibujar recorte»)."),
              *[Ajuste(f"roi.{e}{i}", f"Esquina {i} ({e})", "Recorte de imagen", "int", ini[f"roi.{e}{i}"],
                       0, (An if e == "x" else Al) - 1, 1, "px", en_perfil=False)
                for i in range(1, 5) for e in "xy"]]
    return L


def perfil_de_inicio(a, P):
    """Perfil del objetivo con que parte main.py (--detector), para no ajustar a mano en cada corrida.

    Deja en P.captura la cámara del perfil ANTES de abrirla (parte ya con esa exposición). Devuelve
    (nombre del perfil o None, claves que no se tocan). El resto del perfil (umbrales, predictor)
    lo aplica main() cuando ya existen los detectores, por el mismo camino que usa el tablero."""
    from pt.ajustes import Ajustes, perfil_de
    aj = Ajustes(lista_de_ajustes(a, P, con_camara=True), PERFILES, RUTA_PERFILES)   # sólo para validar rangos
    nombre = perfil_de(aj.perfiles, a.detector)
    if nombre is None:
        return None, ()
    #* Lo escrito en la línea de comandos manda sobre el perfil.
    excluir = tuple(c for c, dado in (("exposicion_us", a.exposicion), ("ganancia", a.ganancia)) if dado)
    perfil, c = aj.perfiles[nombre], P.captura
    for clave in ("exposicion_us", "ganancia", "color_rojo", "color_azul"):
        if clave not in perfil or clave in excluir or clave not in aj.specs:
            continue
        ok, v = aj.validar(clave, perfil[clave])
        if not ok:
            print(f"Perfil «{nombre}»: {clave} = {perfil[clave]!r} no aplicado ({v}).")
        elif clave == "exposicion_us":
            c.exposicion_us = v
        elif clave == "ganancia":
            c.ganancia = v
        elif clave == "color_rojo":
            c.ganancias_color = (v, c.ganancias_color[1])
        else:
            c.ganancias_color = (c.ganancias_color[0], v)
    return nombre, excluir


class AjustesEnVivo:
    """Aplica, dentro del lazo, los cambios pedidos desde el tablero (ya validados en pt/ajustes.py)."""

    def __init__(self, aj, a, P, fuente, detectores, ref, sup):
        self.aj, self.a, self.P, self.fuente = aj, a, P, fuente
        self.detectores, self.ref, self.sup = detectores, ref, sup
        self.params_det = {"rojo": P.deteccion_roja, "luz": P.deteccion_luz, "avion": P.forma_avion,
                           "ovni": P.forma_ovni, "avion_rojo": P.deteccion_avion, "brillo": P.deteccion}

    def aplicar(self, det, t):
        """Una vez por cuadro, antes de detectar. Devuelve el detector vigente (puede haber cambiado)."""
        for clave, valor, origen in self.aj.pendientes():
            try:
                det = self._uno(det, clave, valor, t)
            except Exception as ex:
                #! Un ajuste que falla (p. ej. la cámara rechaza un control) no debe botar el lazo
                #! con los motores andando: se avisa y el valor anterior sigue vigente.
                print(f"Ajuste {clave} = {valor!r} no aplicado: {ex!r}")
                continue
            self.aj.confirmar(clave, valor, origen)
        return det

    def _uno(self, det, clave, valor, t):
        P = self.P
        if clave == "_perfil":
            pass                                     # marca de "perfil aplicado": sólo se confirma
        elif clave == "detector":
            if valor != self.a.detector:
                #* Otro objetivo: lo que se venía siguiendo ya no vale. Detector y estimador parten
                #* de cero y el supervisor vuelve a BÚSQUEDA; mientras tanto la plataforma RETIENE
                #* la última referencia, hasta adquirir el objetivo nuevo (n_adquisicion cuadros).
                self.a.detector = valor
                det = self.detectores[valor]
                det.reiniciar()
                self.ref.reiniciar()
                self.sup.reiniciar_busqueda(t, f"cambio de objetivo: detector {valor}")
        elif clave in ("exposicion_us", "ganancia"):
            self.fuente.ajustar(**{clave: valor})
        elif clave in ("color_rojo", "color_azul"):
            r, b = P.captura.ganancias_color
            self.fuente.ajustar(ganancias_color=(valor, b) if clave == "color_rojo" else (r, valor))
        elif clave == "lam":
            P.prediccion.lam = valor
            for eje in (self.ref.est.eu, self.ref.est.ew):
                eje.rls.lam = valor                  #* sin reiniciar: el RLS sigue con su historia
        elif clave == "sigma_min_deg":
            P.prediccion.sigma_min_deg = valor
            for eje in (self.ref.est.eu, self.ref.est.ew):
                eje.sigma_min = valor
        elif clave == "orden":
            if valor != P.prediccion.orden:
                P.prediccion.orden = valor
                self.ref.reiniciar()                 # otro número de parámetros: el estimador parte de cero
        elif clave == "feedforward":
            P.lazo.usar_feedforward = bool(valor)
        elif clave in ("reinicio_informado", "gating_conjunto"):
            setattr(P.prediccion, clave, bool(valor))    #* prediccion.py lo lee en cada cuadro
        elif clave == "t_acel_max_s":
            P.prediccion.t_acel_max_s = float(valor)
        elif clave.startswith("roi."):
            #* Recorte de imagen (pt/recorte.py). Un cuadrilátero inválido lanza ValueError: no se activa,
            #* aplicar() lo informa y el tablero vuelve a mostrar el valor vigente.
            self.recorte.fijar(clave, valor)
        else:                                        # "<detector>.<atributo>": umbral de un detector
            nombre_det, atributo = clave.split(".", 1)
            if atributo == "salto_activo":
                #* No es un campo de los parámetros: se conmuta salto_max_px entre su valor original y
                #* SALTO_DESACTIVADO. Los detectores lo leen en cada cuadro, así que no hay que reiniciarlos.
                pd = self.params_det[nombre_det]
                if not hasattr(pd, "salto_max_px_orig"):
                    pd.salto_max_px_orig = pd.salto_max_px
                pd.salto_max_px = pd.salto_max_px_orig if valor else SALTO_DESACTIVADO
            else:
                setattr(self.params_det[nombre_det], atributo, valor)
        return det


class MandoEnVivo:
    """Ejecuta, dentro del lazo, las órdenes del tablero ya validadas (pt/mando.py)."""
    #* Cada orden devuelve un texto con lo que pasó: va al historial del tablero y a la consola.
    #* Lo que el firmware sólo acepta por su consola de texto (ganancias, cero, estado, prueba de
    #* signos) se pide con enl.consola(): el enlace se pausa cerca de 1,5 s y sigue solo.

    def __init__(self, m, enl, sup, ref, grab):
        self.m, self.enl, self.sup, self.ref, self.grab = m, enl, sup, ref, grab
        self.manual = None          # (pan, tilt) [°]: referencia de prueba. None = seguimiento
        self.archivos = None        # descargas del tablero (lo fija MonitorRed)

    def aplicar(self, det, t):
        for accion, args, origen in self.m.pendientes():
            try:
                texto = self._uno(accion, args, det, t)
            except Exception as ex:
                #! Una orden que falla no debe botar el lazo con los motores andando.
                texto = f"{accion}: no aplicado ({ex!r})"
            print(f"Mando ({origen or 'local'}): {texto}")
            self.m.anotar(texto, origen)

    def _desde_cero(self, det, t, causa):
        """La pose o el objetivo cambiaron de golpe: estimador y detector parten de nuevo, y la
        referencia retenida se toma otra vez de donde está la plataforma."""
        self.ref.reiniciar()
        self.ref.ref_retencion = None
        det.reiniciar()
        self.sup.reiniciar_busqueda(t, causa)

    def _uno(self, accion, args, det, t):
        enl, sup = self.enl, self.sup
        tel = getattr(enl, "telemetria", None)
        if accion == "habilitar":
            ok, d = enl.habilitar()
            if not ok:
                return f"habilitar: {d}"
            #* Parte reteniendo donde está: sin esto iría a la última referencia del seguimiento,
            #* que puede ser vieja si la plataforma se movió a mano con los motores cortados.
            self._desde_cero(det, t, "motores habilitados")
            sup.rearmar(t, "habilitar desde el tablero")
            return "motores HABILITADOS"
        if accion == "deshabilitar":
            ok, d = enl.deshabilitar()
            self.manual = None
            return "motores CORTADOS" if ok else f"cortar: {d}"
        if accion == "retener":
            if not enl.puede_mover or tel is None:
                return "retener: los motores están cortados"
            self.manual = (round(tel.pan_med, 3), round(tel.tilt_med, 3))
            return f"retenido en pan {self.manual[0]:+.2f}°, tilt {self.manual[1]:+.2f}° (referencia manual)"
        if accion == "rearmar":
            ok, d = enl.rearmar()
            hubo = sup.rearmar(t)
            self._desde_cero(det, t, "rearme")
            return ("rearmado: " if hubo else "no estaba en MODO_SEGURO; ") + d
        if accion == "estado":
            ok, d = enl.consola("p", 0.6)
            return f"estado: {d}"
        if accion == "cero":
            #! El firmware sólo fija el cero con los drivers deshabilitados: se cortan antes.
            if enl.puede_mover:
                enl.deshabilitar()
            self.manual = None
            ok, d = enl.consola("z", 0.6)
            if ok:
                self._desde_cero(det, t, "cero nuevo")
            return f"cero (motores cortados): {d}"
        if accion == "prueba_signos":
            if not enl.puede_mover:
                return "prueba de signos: primero habilitar los motores"
            ok, d = enl.consola(f"t {args['eje']}", 3.0)
            return f"prueba de signos de {args['eje']}: {d}"
        if accion == "ganancias":
            ok, d = enl.fijar_ganancias(args["eje"], args["kp"], args["ki"])
            return f"ganancias de {args['eje']} (Kp {args['kp']:g}, Ki {args['ki']:g}): {d}"
        if accion == "aceleracion":
            ok, d = enl.consola(f"a {args['eje']} {args['alfa']:.1f}", 0.4)
            return f"aceleración máxima de {args['eje']} ({args['alfa']:g} °/s²): {d}"
        if accion == "referencia":
            if not enl.puede_mover:
                return "referencia de prueba: los motores están cortados"
            self.manual = (args["pan"], args["tilt"])
            return f"referencia manual: pan {args['pan']:+.2f}°, tilt {args['tilt']:+.2f}°"
        if accion == "seguimiento":
            if self.manual is None:
                return "ya estaba en seguimiento"
            self.manual = None
            self.ref.reiniciar()
            det.reiniciar()
            sup.reiniciar_busqueda(t, "fin de la referencia manual")
            return "vuelve al seguimiento (retiene hasta adquirir el objetivo)"
        if accion == "grabar":
            ruta = self.grab.iniciar(args["etiqueta"])
            if self.archivos is not None:
                self.archivos[os.path.basename(ruta)] = lambda r=ruta: r
            return f"grabando «{args['etiqueta']}» en {ruta}"
        if accion == "detener_grabacion":
            hecho = self.grab.detener()
            return f"grabación guardada: {hecho[1]} ({hecho[2]} filas)" if hecho else "no había grabación en curso"
        return f"{accion}: sin implementar"

    def estado(self):
        """Lo vigente, para el tablero."""
        enl, g = self.enl, self.grab
        self.m.publicar(
            motores=bool(getattr(enl, "puede_mover", False)),
            con_esp32=getattr(enl, "telemetria", None) is not None,
            manual=list(self.manual) if self.manual else None,
            gan={k: list(v) for k, v in (getattr(enl, "ganancias", None) or {}).items()},
            consola=bool(getattr(enl, "consola_pendiente", getattr(enl, "en_consola", False))),
            grab={"activa": g.activa, "etiqueta": g.etiqueta, "filas": g.reg.n if g.activa else 0},
            grabs=[[n, filas, etq] for (n, _, filas, etq) in g.hechas[-8:]])
        return self.m.estado()


class MonitorRed:
    """MON-1 / MON-2: publica las variables de seguimiento en un tablero web (pt/monitor_red.py).

    Corre en hilos aparte. El lazo sólo le entrega una tupla por cuadro y, cada pocos cuadros,
    el estado lento y una copia de la imagen. Ver el tablero nunca frena el control.
    """

    def __init__(self, a, P, geo, enl, reg, ajustes=None, mando=None):
        from pt.monitor_red import ServidorMonitor
        self.a, self.geo, self.enl, self.aj, self.mando = a, geo, enl, ajustes, mando
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
                                   telemetria=getattr(enl, "historial_tel", None), ajustes=ajustes,
                                   mando=mando.m if mando else None)
        if mando:
            mando.archivos = self.srv.archivos       # las grabaciones quedan descargables desde el tablero
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
                "detectado": bool(r.valido), "pico": r.pico, "area": r.area,
                "motivo": r.motivo or (f"detectado: {r.forma} (d = {r.dist:.2f})" if r.forma else "detectado"),
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
                #* Ajustes en vivo: valores VIGENTES (ya aplicados por el lazo) y lo que la cámara
                #* informa en los metadatos de este cuadro (puede diferir de lo pedido si lo recortó).
                "aj": self.aj.estado() if self.aj else None,
                #* Mando: motores, referencia manual, ganancias del PI, ventana de consola y grabaciones.
                "mando": self.mando.estado() if self.mando else None,
                "cam": {"exp_us": c.meta.get("ExposureTime"), "gan": c.meta.get("AnalogueGain")},
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
    #* El lazo comparte el intérprete con otros hilos de Python (lector de telemetría, servidor
    #* del tablero). Por defecto Python cambia de hilo cada 5 ms; con 0,5 ms, una descarga de CSV
    #* desde el tablero deja de retrasar al lazo. Medido en simulación con descargas continuas:
    #* p99 de la latencia total 18,8 -> 13,8 ms (sin tablero: 13,0 ms).
    sys.setswitchinterval(0.0005)
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
    if a.ganancias_sim and a.enlace != "simulado":
        #! Las ganancias reales viven en el ESP32 (config.h o comando k de su consola): la RPi no las cambia.
        raise SystemExit("--ganancias-sim sólo vale con --enlace simulado (en el ESP32 real: comando k <eje> <Kp> <Ki>)")
    if a.monitor and a.fuente == "sintetica" and not a.tiempo_real:
        a.tiempo_real = True             #* con el tablero abierto, la simulación va al ritmo real
        print("Monitor: fuente sintética en tiempo real (--tiempo-real).")

    #* Perfil del objetivo al partir. Sólo con la cámara real: con la fuente sintética no se aplica,
    #* para que las simulaciones no dependan de lo que alguien haya guardado en perfiles.json.
    perfil_inicio, excluir_inicio = (None, ())
    if a.fuente == "camara" and not a.sin_perfil:
        perfil_inicio, excluir_inicio = perfil_de_inicio(a, P)

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
        try:
            enl = EnlaceTrama(P.enlace, habilitar=a.habilitar)
        except Exception as ex:
            if a.enlace_pedido == "trama":
                raise SystemExit(f"No se pudo abrir el enlace con el ESP32 ({P.enlace.puerto}): {ex}")
            #* Enlace por defecto y sin puerto serie (p. ej. la RPi sin el UART activado): se sigue sin ESP32.
            print(f"AVISO: sin enlace con el ESP32 ({P.enlace.puerto}: {ex}). Se sigue sólo con cámara y detección.")
            a.enlace, enl = "nulo", EnlaceNulo()
        else:
            print(f"ESP32: {'telemetría OK, estado ' + enl.telemetria.nombre_estado if enl.telemetria else 'SIN TELEMETRÍA'}"
                  f" | motores {'HABILITADOS' if a.habilitar else 'cortados (se habilitan desde el tablero)'}")
            #* Ganancias vigentes del PI: se piden por la consola del firmware (comando p), para
            #* mostrarlas en el tablero y dejarlas en el CSV. Cuesta una pausa del enlace al partir.
            enl.consola("p", 0.5)
    elif a.enlace == "simulado":
        if a.ganancias_sim:
            #* Sólo cambia el PI SIMULADO, para comparar grupos de ganancias antes de ir al laboratorio.
            kp_pan, ki_pan, kp_tilt, ki_tilt = a.ganancias_sim
            enl = EnlaceSimulado(P, kp=(kp_pan, kp_tilt), ki=(ki_pan, ki_tilt))
            print(f"Lazo simulado: pan Kp = {kp_pan:g}, Ki = {ki_pan:g} | tilt Kp = {kp_tilt:g}, Ki = {ki_tilt:g}")
        else:
            enl = EnlaceSimulado(P)
    else:
        enl = EnlaceNulo()

    if a.fuente == "sintetica":
        from pt.camara import FuenteSintetica
        W, H = P.captura.salida_px
        kw = {"lampara": (500, 100)} if a.lampara else {}
        if a.objeto != "punto":
            kw.update(objeto=a.objeto, fondo=150)        # silueta oscura sobre fondo claro
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

    #* Todos los detectores quedan creados: cambiar de objetivo desde el tablero es elegir otro.
    detectores = crear_detectores(P, RUTA_FORMAS)
    if a.detector not in detectores:
        raise SystemExit(f"El detector '{a.detector}' necesita OpenCV: sudo apt install python3-opencv")
    det = detectores[a.detector]
    ref = ReferenciaVisual(P, geo)
    sup = Supervisor(P.supervisor, con_actuadores=(a.enlace != "nulo"))
    reg = Registro(a.registro, prefijo=f"{a.fuente}_{a.enlace}")
    vista = Ventana(geo) if a.ventana else None
    vivo = None
    recorte = None                                  #* recorte de imagen del tablero (None = sin la herramienta)
    if a.monitor and not a.monitor_solo_lectura:
        from pt.ajustes import Ajustes
        aj = Ajustes(lista_de_ajustes(a, P, con_camara=(a.fuente == "camara")), PERFILES, RUTA_PERFILES)
        aj.detector_trae_perfil = not a.sin_perfil
        vivo = AjustesEnVivo(aj, a, P, fuente, detectores, ref, sup)
        if not a.sin_recorte:
            from pt.recorte import RecorteROI
            recorte = vivo.recorte = RecorteROI(*P.captura.salida_px)
    if perfil_inicio:
        #* Resto del perfil (umbrales del detector, predictor): se valida y se aplica igual que un
        #* clic en el tablero, pero aquí mismo, antes del primer cuadro. Funciona con o sin --monitor.
        from pt.ajustes import Ajustes
        aj0 = vivo.aj if vivo else Ajustes(lista_de_ajustes(a, P, con_camara=True), PERFILES, RUTA_PERFILES)
        aplicador = vivo or AjustesEnVivo(aj0, a, P, fuente, detectores, ref, sup)
        puestos, malos = aj0.pedir_perfil(perfil_inicio, "inicio", excluir=excluir_inicio)
        det = aplicador.aplicar(det, 0.0)
        print(f"Perfil «{perfil_inicio}» aplicado al partir (detector {a.detector}): "
              + ", ".join(f"{c} = {v}" for c, v in puestos.items() if c != "detector")
              + (f" | no aplicado: {malos}" if malos else "")
              + (f" | de la línea de comandos: {', '.join(excluir_inicio)}" if excluir_inicio else "")
              + ". Para partir sin perfil: --sin-perfil")
    #* Mando del ESP32 y grabación de experimentos desde el tablero (pt/mando.py).
    grab = Grabacion(a.registro)
    mando = None
    if a.monitor and not a.monitor_solo_lectura and not a.sin_mando:
        from pt.mando import Mando
        mando = MandoEnVivo(Mando(P.lazo.lim_pan_deg, P.lazo.lim_tilt_deg), enl, sup, ref, grab)
    mon = MonitorRed(a, P, geo, enl, reg, ajustes=vivo.aj if vivo else None, mando=mando) if a.monitor else None
    if mon:
        print("Monitor (abrir en el navegador de un equipo en la misma red):")
        for u in mon.urls:
            print("   ", u)
        if vivo:
            #! Decisión del equipo (17-11-2026): el tablero puede cambiar parámetros en caliente.
            print("    AJUSTES EN VIVO activos: cualquier equipo de esta red puede cambiar cámara, objetivo, "
                  "umbrales y predictor.\n    Para dejar el tablero sin escritura: --monitor-solo-lectura")
        if mando:
            #! Decisión del equipo (19-11-2026): el tablero puede habilitar y mover los motores.
            print("    MANDO activo: desde el tablero se habilitan, cortan y rearman los motores, se fijan cero y "
                  "ganancias, se da una referencia de prueba\n    y se graban experimentos. Para quitarlo: --sin-mando")

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
            if vivo:
                det = vivo.aplicar(det, t)       #* ajustes del tablero, ya validados (pt/ajustes.py)
            if mando:
                mando.aplicar(det, t)            #* órdenes del tablero, ya validadas (pt/mando.py)
            seguimiento = sup.estado == Estado.SEGUIMIENTO

            # --- PI: detección -----------------------------------------------------------------
            if recorte is not None and recorte.activo:
                #* Recorte del tablero: SÓLO la imagen que ve el detector. c.img queda intacta para el
                #* tablero, el registro y la geometría; (u, w) siguen siendo del cuadro completo.
                img_det, v_det = recorte.aplicar(c.img, c.v)
                if not recorte.activo:               # falló dentro del recorte: quedó apagado
                    print(f"AVISO: recorte de imagen apagado por un error: {recorte.error}")
                    vivo.aj.confirmar("roi.activo", False, "automático")
                r = det.procesar(img_det, usar_ventana=seguimiento, v=v_det)
                r.t_proc_ms += recorte.t_ms
            else:
                r = det.procesar(c.img, usar_ventana=seguimiento, v=c.v)

            # --- CS-1 + EP: objetivo en el mundo, predicho al instante de aplicación ------------
            #* Instante de la fila donde está el objetivo (persiana rodante): ahí se lee la pose.
            t_obj = c.instante(r.w if r.valido else None)
            pose = enl.angulos_en(t_obj) if usar_pose else (0.0, 0.0)
            t_envio = (c.t_entrega_ns + int(r.t_proc_ms * 1e6)) if tiempo_virtual else time.monotonic_ns()
            t_aplic = t_envio + retardo_ns
            med, e = ref.actualizar(t_obj, r, pose, t_aplic)

            # --- SE ----------------------------------------------------------------------------
            #* Con los motores cortados, MODO_SEGURO se deja solo cuando la causa desaparece; con
            #* los motores habilitados hace falta "Rearmar" en el tablero (pt/supervisor.py).
            estado = sup.paso(t, med.valida, e.confianza, enl.ok(), enl.listo(), enl.en_falla(),
                              rearme_auto=not getattr(enl, "puede_mover", False))
            if estado != estado_ant and estado in (Estado.BUSQUEDA, Estado.PERDIDA):
                det.reiniciar()                  # la ventana vuelve al cuadro completo
                if estado == Estado.BUSQUEDA:
                    ref.reiniciar()
            estado_ant = estado

            # --- CS-2 / CS-3: consigna al ESP32 --------------------------------------------------
            consigna = None
            manual = mando.manual if mando else None
            if estado == Estado.MODO_SEGURO:
                if not modo_seguro_avisado:
                    enl.retener()                #! la RPi deja de mandar consignas; el ESP32 retiene
                    modo_seguro_avisado = True
            else:
                modo_seguro_avisado = False      # si vuelve a caer, se retiene de nuevo
                if estado not in (Estado.INICIALIZACION, Estado.HOMING):
                    if manual is not None:
                        #* Referencia de prueba del tablero: reemplaza a la del seguimiento (sin ω_ff).
                        consigna = (manual[0], manual[1], 0.0, 0.0)
                        ref.ref_retencion = (manual[0], manual[1])   # al volver, se retiene donde quedó
                    else:
                        consigna = ref.consigna(estado == Estado.SEGUIMIENTO, e, pose)
                    if consigna is not None:
                        enl.enviar_consigna(*consigna, t_ns=t_envio)
            if consigna is None:
                #* Sin consigna que mandar (arranque, MODO_SEGURO): un latido mantiene la telemetría,
                #* que es lo que permite ver que el ESP32 volvió y salir de MODO_SEGURO.
                enl.latido()

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
            gan = getattr(enl, "ganancias", None) or {}
            fila_csv = dict(k=k, t_exp_ns=c.t_exp_ns, t_entrega_ns=c.t_entrega_ns, t_fin_ns=t_fin,
                     lat_captura_ms=(c.t_entrega_ns - c.t_exp_ns) / 1e6, lat_total_ms=lat,
                     t_det_ms=r.t_proc_ms, estado=estado.value, valido=int(r.valido), motivo=r.motivo,
                     u=r.u, w=r.w, pico=r.pico, area=r.area, forma=r.forma, forma_dist=r.dist,
                     dpan_deg=dpan, dtilt_deg=dtilt,
                     pan_med=pose[0] if pose else float("nan"), tilt_med=pose[1] if pose else float("nan"),
                     pan_obj=med.pan, tilt_obj=med.tilt, pan_pred=e.u, tilt_pred=e.w,
                     vpan_pred=e.vu, vtilt_pred=e.vw, confianza=e.confianza, t_sin_med_s=e.t_sin_medicion,
                     est_u=e.estado_u, est_w=e.estado_w,
                     gating_d=e.d_gating, sigma_pan=e.sigma_u, sigma_tilt=e.sigma_w,
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
                     u_real=ur, w_real=wr, err_centro_px=e_centro,
                     detector=a.detector, exp_us=c.meta.get("ExposureTime", ""),
                     ganancia=c.meta.get("AnalogueGain", ""), lam=P.prediccion.lam,
                     ff=int(P.lazo.usar_feedforward),
                     kp_pan=gan.get("pan", ("", ""))[0], ki_pan=gan.get("pan", ("", ""))[1],
                     kp_tilt=gan.get("tilt", ("", ""))[0], ki_tilt=gan.get("tilt", ("", ""))[1],
                     manual=int(manual is not None), experimento=grab.etiqueta)
            reg.fila(**fila_csv)
            if grab.activa:
                grab.fila(**fila_csv)            #* experimento en curso: las mismas filas, en su propio CSV

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
        hecho = grab.detener()
        if hecho:
            print(f"Grabación cerrada al terminar: {hecho[1]} ({hecho[2]} filas)")
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
