"""
monitor_red.py — tablero de monitoreo por la red local (MON-1 / MON-2), en un hilo aparte.

Sirve una página web (pt/monitor_web/index.html) desde la propia RPi. Se ve desde el navegador
de cualquier equipo en la misma red, sin monitor HDMI:  http://g9-cap.local:8000

  /                 tablero
  /config           columnas, unidades, límites y geometría (JSON, una vez)
  /flujo            Server-Sent Events: cada 1/hz s manda TODAS las filas nuevas (una por
                    cuadro, ~80 Hz) y el estado lento. El navegador ve la dinámica completa.
  /estado           último estado lento (JSON; útil con curl)
  /imagen.jpg       último cuadro en escala de grises (sólo si alguien lo pide)
  /historial.csv    últimos `segundos` de filas por cuadro (variables rápidas, MON-2)
  /telemetria.csv   telemetría del ESP32 a 200 Hz (θ_med, θ_ref, ω_cmd, estado, fallas)
  /registro.csv     el CSV completo de la corrida en curso (pt/registro.py)

Sólo depende de la biblioteca estándar. La imagen usa OpenCV (o Pillow) si están instalados.
"""
#! El tablero es de SOLO LECTURA: no hay ninguna ruta que mande comandos al ESP32 ni cambie
#! parámetros. Mover motores desde un navegador de la red sería un riesgo (cualquiera conectado
#! al mismo hotspot podría hacerlo). La parada sigue siendo Ctrl+C en la RPi o 'x' en la consola.
#! Escucha en 0.0.0.0 (toda la red local). Nunca abrir el puerto hacia internet.
import collections
import io
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CARPETA_WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "monitor_web")

#* Variables rápidas, una fila por cuadro. El orden es el del CSV y el del arreglo que recibe
#* el navegador. Ángulos en grados, píxeles en px, ω en °/s.
COLUMNAS_MONITOR = [
    ("t", "s", "tiempo desde el primer cuadro"),
    ("k", "", "número de cuadro"),
    ("valido", "", "1 = objetivo detectado en este cuadro"),
    ("u", "px", "posición del objeto en la imagen (columna)"),
    ("w", "px", "posición del objeto en la imagen (fila)"),
    ("ex_px", "px", "error horizontal respecto del centro de la imagen"),
    ("ey_px", "px", "error vertical respecto del centro de la imagen"),
    ("err_pan", "deg", "error de centrado en pan (ángulo del objeto respecto del eje óptico)"),
    ("err_tilt", "deg", "error de centrado en tilt"),
    ("pan_med", "deg", "ángulo medido de pan (AS5600) en la exposición"),
    ("tilt_med", "deg", "ángulo medido de tilt"),
    ("pan_obj", "deg", "dirección del objeto en el mundo, pan"),
    ("tilt_obj", "deg", "dirección del objeto en el mundo, tilt"),
    ("pan_ref", "deg", "referencia enviada al ESP32, pan"),
    ("tilt_ref", "deg", "referencia enviada al ESP32, tilt"),
    ("w_pan", "deg/s", "señal de control ω_cmd del PI del ESP32, pan"),
    ("w_tilt", "deg/s", "señal de control ω_cmd del PI del ESP32, tilt"),
    ("lat_ms", "ms", "latencia exposición -> consigna"),
    ("conf", "", "confianza del estimador (0 a 1)"),
    ("estado", "", "estado del supervisor"),
]
NOMBRES = [c[0] for c in COLUMNAS_MONITOR]

COLUMNAS_TELEMETRIA = ["t_rpi_ns", "t_us", "pan_med", "tilt_med", "pan_ref", "tilt_ref",
                       "w_pan", "w_tilt", "estado", "fallas", "seq_consigna"]


def limpiar(x, nd=4):
    """Para JSON: NaN/inf -> None (JSON no admite NaN) y floats redondeados."""
    if isinstance(x, float):
        return round(x, nd) if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: limpiar(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [limpiar(v, nd) for v in x]
    if hasattr(x, "item"):                       # escalares de numpy
        return limpiar(x.item(), nd)
    return x


def ips_locales():
    """IP de la RPi en la red local (para imprimir la dirección a abrir)."""
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))         #? no manda paquetes: sólo elige la interfaz de salida
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return [ip for ip in ips if not ip.startswith("127.")]


class _Codificador:
    """JPEG del último cuadro. OpenCV si está; si no, Pillow; si no, sin imagen."""

    def __init__(self, calidad=60):
        self.calidad = calidad
        self.cv2 = self.pil = None
        try:
            import cv2
            self.cv2 = cv2
        except ImportError:
            try:
                from PIL import Image
                self.pil = Image
            except ImportError:
                pass

    @property
    def disponible(self):
        return bool(self.cv2 or self.pil)

    def jpeg(self, img):
        if self.cv2 is not None:
            ok, buf = self.cv2.imencode(".jpg", img, [int(self.cv2.IMWRITE_JPEG_QUALITY), self.calidad])
            return buf.tobytes() if ok else None
        if self.pil is not None:
            b = io.BytesIO()
            self.pil.fromarray(img).save(b, format="JPEG", quality=self.calidad)
            return b.getvalue()
        return None


class ServidorMonitor:
    #* Reparto de trabajo: el hilo principal (lazo a 80 fps) sólo agrega una tupla a una cola y
    #* reemplaza una referencia al estado. Todo lo caro (JSON, JPEG, CSV, sockets) ocurre en los
    #* hilos del servidor. Si el navegador se atrasa, se pierden refrescos de vista, nunca cuadros
    #* del lazo ni filas del registro.
    def __init__(self, puerto=8000, host="0.0.0.0", hz=20.0, imagen_hz=5.0, segundos=120.0,
                 fps=80.0, config=None, archivos=None, telemetria=None, max_clientes=5):
        self.puerto, self.host = int(puerto), host
        self.periodo = 1.0 / max(float(hz), 1.0)
        self.imagen_hz = float(imagen_hz)
        self.config = dict(config or {})
        self.archivos = dict(archivos or {})       # nombre -> función que entrega la ruta
        self.telemetria = telemetria               # deque de tuplas COLUMNAS_TELEMETRIA (o None)
        self.max_clientes = max_clientes
        self.filas = collections.deque(maxlen=int(segundos * fps))
        self.seq = 0                               # número de la última fila publicada
        self.lock = threading.Lock()
        self.estado = {}
        self._img, self._img_n = None, 0
        self._jpeg, self._jpeg_n = None, -1
        self._lock_jpeg = threading.Lock()
        self.cod = _Codificador()
        self.clientes = 0
        self.parar = threading.Event()
        self.httpd = None
        self._hilo = None

    # ---- lado del lazo (hilo principal) ------------------------------------------------------
    def publicar_fila(self, fila):
        """fila: tupla en el orden de COLUMNAS_MONITOR."""
        with self.lock:
            self.seq += 1
            self.filas.append((self.seq, fila))

    def publicar_estado(self, estado):
        self.estado = estado                       # reemplazo atómico de la referencia

    def quiere_imagen(self):
        return self.cod.disponible and self.imagen_hz > 0 and self.clientes > 0

    def publicar_imagen(self, img):
        self._img = img                            #! debe ser una COPIA (la cámara reusa buffers)
        self._img_n += 1

    # ---- lado del servidor --------------------------------------------------------------------
    def filas_desde(self, ultimo):
        with self.lock:
            if not self.filas or self.filas[-1][0] <= ultimo:
                return [], ultimo
            nuevas = [f for (s, f) in self.filas if s > ultimo]
            return nuevas, self.filas[-1][0]

    def jpeg(self):
        with self._lock_jpeg:
            if self._img is None:
                return None
            if self._jpeg_n != self._img_n:
                self._jpeg, self._jpeg_n = self.cod.jpeg(self._img), self._img_n
            return self._jpeg

    def config_json(self):
        c = dict(self.config)
        c.update(columnas=[{"nombre": n, "unidad": u, "desc": d} for (n, u, d) in COLUMNAS_MONITOR],
                 hz=1.0 / self.periodo, imagen_hz=self.imagen_hz if self.cod.disponible else 0,
                 segundos=self.filas.maxlen / max(self.config.get("fps", 80.0), 1.0),
                 telemetria=self.telemetria is not None)
        return limpiar(c)

    def iniciar(self):
        servidor = self

        class Manejador(BaseHTTPRequestHandler):
            server_version = "MonitorG9/1.0"

            def log_message(self, *args):          # sin una línea por petición en la consola
                pass

            def _enviar(self, codigo, tipo, cuerpo, extra=None):
                self.send_response(codigo)
                self.send_header("Content-Type", tipo)
                self.send_header("Content-Length", str(len(cuerpo)))
                self.send_header("Cache-Control", "no-store")
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(cuerpo)

            def _json(self, obj):
                self._enviar(200, "application/json; charset=utf-8",
                             json.dumps(obj, separators=(",", ":"), allow_nan=False).encode())

            def _csv(self, nombre, filas, cabecera):
                b = io.StringIO()
                b.write(",".join(cabecera) + "\n")
                for f in filas:
                    b.write(",".join("" if v is None else (f"{v:.6g}" if isinstance(v, float) else str(v))
                                     for v in limpiar(list(f), 6)) + "\n")
                self._enviar(200, "text/csv; charset=utf-8", b.getvalue().encode(),
                             {"Content-Disposition": f'attachment; filename="{nombre}"'})

            def do_GET(self):
                ruta = self.path.split("?", 1)[0]
                try:
                    if ruta in ("/", "/index.html"):
                        with open(os.path.join(CARPETA_WEB, "index.html"), "rb") as f:
                            self._enviar(200, "text/html; charset=utf-8", f.read())
                    elif ruta == "/config":
                        self._json(servidor.config_json())
                    elif ruta == "/estado":
                        self._json(limpiar(servidor.estado))
                    elif ruta == "/flujo":
                        self._flujo()
                    elif ruta == "/imagen.jpg":
                        j = servidor.jpeg()
                        if j is None:
                            self._enviar(204, "text/plain", b"")
                        else:
                            self._enviar(200, "image/jpeg", j)
                    elif ruta == "/historial.csv":
                        with servidor.lock:
                            filas = [f for (_, f) in servidor.filas]
                        self._csv(f"historial_{time.strftime('%Y%m%d_%H%M%S')}.csv", filas, NOMBRES)
                    elif ruta == "/telemetria.csv":
                        if servidor.telemetria is None:
                            self._enviar(404, "text/plain; charset=utf-8", "Sin telemetría del ESP32 en este modo.".encode())
                        else:
                            filas = list(servidor.telemetria)
                            self._csv(f"telemetria_{time.strftime('%Y%m%d_%H%M%S')}.csv", filas, COLUMNAS_TELEMETRIA)
                    elif ruta.lstrip("/") in servidor.archivos:
                        r = servidor.archivos[ruta.lstrip("/")]()
                        if not r or not os.path.exists(r):
                            self._enviar(404, "text/plain; charset=utf-8", b"Archivo no disponible.")
                        else:
                            with open(r, "rb") as f:
                                datos = f.read()
                            self._enviar(200, "text/csv; charset=utf-8", datos,
                                         {"Content-Disposition": f'attachment; filename="{os.path.basename(r)}"'})
                    else:
                        self._enviar(404, "text/plain; charset=utf-8", b"No encontrado.")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _flujo(self):
                if servidor.clientes >= servidor.max_clientes:
                    self._enviar(503, "text/plain; charset=utf-8", "Demasiados tableros abiertos.".encode())
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                with servidor.lock:
                    servidor.clientes += 1
                #* Al conectarse recibe los últimos ~3 s, para que los gráficos no partan vacíos.
                ultimo = max(servidor.seq - int(3 * servidor.config.get("fps", 80.0)), 0)
                try:
                    while not servidor.parar.is_set():
                        filas, ultimo = servidor.filas_desde(ultimo)
                        msg = {"e": limpiar(servidor.estado), "f": limpiar(filas)}
                        self.wfile.write(b"data: " + json.dumps(msg, separators=(",", ":"), allow_nan=False).encode() + b"\n\n")
                        self.wfile.flush()
                        servidor.parar.wait(servidor.periodo)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    with servidor.lock:
                        servidor.clientes -= 1

        class Servidor(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        try:
            self.httpd = Servidor((self.host, self.puerto), Manejador)
        except OSError as ex:
            raise SystemExit(f"Monitor: no se pudo abrir el puerto {self.puerto} ({ex}). "
                             f"¿Otro main.py corriendo? Probar con --monitor-puerto {self.puerto + 1}.")
        self._hilo = threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.2},
                                      name="monitor_red", daemon=True)
        self._hilo.start()
        return self.urls()

    def urls(self):
        if self.host == "127.0.0.1":
            return [f"http://localhost:{self.puerto}  (sólo por túnel SSH)"]
        nombre = socket.gethostname()
        u = [f"http://{nombre}.local:{self.puerto}"]
        u += [f"http://{ip}:{self.puerto}" for ip in ips_locales()]
        return u

    def detener(self):
        self.parar.set()
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.httpd = None
