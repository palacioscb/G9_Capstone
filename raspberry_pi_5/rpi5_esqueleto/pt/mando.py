"""
mando.py — órdenes a la plataforma desde el tablero (pt/monitor_red.py, ruta /mando).

Con esto no hace falta entrar a la consola del ESP32 para lo habitual de una sesión de
pruebas: cortar o habilitar los motores, rearmar tras una falla, fijar el cero, ver el estado
de los sensores, probar los signos, cambiar las ganancias del PI, dar una referencia de prueba
y grabar un experimento. main.py se ejecuta una vez y el resto se hace desde el navegador.

Igual que pt/ajustes.py, trabajan dos hilos y cada uno hace sólo su parte:
  - el del servidor web VALIDA la orden (acción declarada, tipos y rangos) y la deja en una cola;
  - el del lazo (main.py, MandoEnVivo) saca la cola al comienzo de cada cuadro, la EJECUTA y
    anota qué pasó. El lazo nunca espera al navegador ni recibe una orden sin validar.
"""
#! DECISIÓN DEL EQUIPO (19-11-2026): el tablero puede MOVER LOS MOTORES. Hasta el 18-11 no
#! había ruta de red para habilitar motores, mandar referencias ni cambiar ganancias. El equipo
#! lo pidió porque el espacio de trabajo es controlado y de aprendizaje.
#! RIESGO ASUMIDO: cualquier equipo conectado al mismo hotspot puede habilitar y mover la plataforma.
#! Lo que lo acota:
#!   - sólo existen las acciones de ACCIONES, con sus argumentos validados en tipo y rango;
#!   - la referencia de prueba se recorta a los límites de recorrido de parametros.py, y el
#!     firmware aplica además los suyos, ω_max y α_max;
#!   - el cero sólo se fija con los motores cortados (se cortan antes), como exige el firmware;
#!   - "Cortar motores" está siempre disponible y no pasa por la ventana de consola;
#!   - main.py --sin-mando deja el tablero sin estas órdenes, y --monitor-solo-lectura, sin escritura.
#! El firmware NO se modificó: lo que no tiene comando binario (ganancias, cero, estado, prueba
#! de signos) viaja por su consola de texto, en una pausa breve del enlace (pt/enlace.py).
import collections
import math
import re
import threading
import time

EJES = ("pan", "tilt")
_ETIQUETA = re.compile(r"^[A-Za-z0-9_-]{1,32}$")

#* acción -> argumentos: nombre -> ("eje",) | ("num", mín, máx) | ("etiqueta",)
#? Rangos de las ganancias: holgados a propósito, para poder explorar. La cota de estabilidad de
#? pid_t_2 para pan es Kp < 284·ζ (con ζ = 0,05 supuesto, unos 14 1/s): pasarla hace oscilar el eje.
ACCIONES = {
    "habilitar": {},                 # energiza los drivers y rearma
    "deshabilitar": {},              # corta los motores
    "retener": {},                   # mantiene la posición actual (referencia manual = donde está)
    "rearmar": {},                   # sale de MODO_SEGURO y repite el arranque
    "estado": {},                    # comando 'p' del firmware: sensores, signos, ganancias
    "cero": {},                      # comando 'z': cero en la posición actual, guardado en el ESP32
    "prueba_signos": {"eje": ("eje",)},
    "ganancias": {"eje": ("eje",), "kp": ("num", 0.0, 100.0), "ki": ("num", 0.0, 2000.0)},
    "aceleracion": {"eje": ("eje",), "alfa": ("num", 0.0, 20000.0)},     # 0 = sin límite
    "referencia": {"pan": ("num", None, None), "tilt": ("num", None, None)},   # límites: los de recorrido
    "seguimiento": {},               # fin de la referencia manual
    "grabar": {"etiqueta": ("etiqueta",)},
    "detener_grabacion": {},
}


class Mando:
    def __init__(self, lim_pan, lim_tilt):
        self.lim = {"pan": (float(lim_pan[0]), float(lim_pan[1])), "tilt": (float(lim_tilt[0]), float(lim_tilt[1]))}
        self._cola = collections.deque()                 # append/popleft son atómicos
        self._lock = threading.Lock()
        self.historial = collections.deque(maxlen=10)    # (hora, origen, texto)
        self.vigente = {}                                # lo que informa el lazo (MandoEnVivo.estado)

    # ---- validación (hilo del servidor) --------------------------------------------------------
    def validar(self, pedido):
        """Devuelve (True, (acción, argumentos normalizados)) o (False, motivo)."""
        if not isinstance(pedido, dict):
            return False, "se esperaba un objeto {accion: ..., argumentos}"
        accion = pedido.get("accion")
        spec = ACCIONES.get(accion) if isinstance(accion, str) else None
        if spec is None:
            return False, f"acción desconocida; acciones: {sorted(ACCIONES)}"
        sobran = set(pedido) - set(spec) - {"accion"}
        if sobran:
            return False, f"argumentos que no corresponden: {sorted(sobran)}"
        args = {}
        for nombre, regla in spec.items():
            v = pedido.get(nombre)
            if regla[0] == "eje":
                if v not in EJES:
                    return False, f"{nombre}: se esperaba 'pan' o 'tilt'"
            elif regla[0] == "etiqueta":
                if not isinstance(v, str) or not _ETIQUETA.match(v):
                    return False, f"{nombre}: letras, números, '_' o '-' (máx. 32), sin espacios"
            else:
                #! bool es subclase de int en Python: se descarta aparte para que true no valga 1.
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    return False, f"{nombre}: se esperaba un número"
                lo, hi = (self.lim[nombre] if accion == "referencia" else regla[1:3])
                if v < lo or v > hi:
                    return False, f"{nombre}: fuera de rango [{lo:g}, {hi:g}]"
                v = float(v)
            args[nombre] = v
        return True, (accion, args)

    def pedir(self, pedido, origen=""):
        """Valida y encola. Devuelve (ok, detalle)."""
        ok, r = self.validar(pedido)
        if not ok:
            return False, r
        if len(self._cola) >= 8:
            return False, "hay órdenes sin atender: esperar un momento"
        self._cola.append((r[0], r[1], origen))
        return True, f"{r[0]}: en cola"

    # ---- lado del lazo (hilo principal) --------------------------------------------------------
    def pendientes(self):
        fuera = []
        while self._cola:
            fuera.append(self._cola.popleft())
        return fuera

    def anotar(self, texto, origen=""):
        with self._lock:
            self.historial.append((time.strftime("%H:%M:%S"), origen, texto))

    def publicar(self, **vigente):
        """El lazo informa lo vigente: motores, referencia manual, ganancias, grabación."""
        self.vigente = vigente

    # ---- para el tablero -----------------------------------------------------------------------
    def estado(self):
        with self._lock:
            hist = [list(h) for h in self.historial]
        return {**self.vigente, "lim": {e: list(v) for e, v in self.lim.items()}, "hist": hist}
