"""
ajustes.py — parámetros que se cambian EN CALIENTE desde el tablero (pt/monitor_red.py).

Sirve para no volver a ejecutar main.py al pasar de un objetivo a otro (láser, luz, avión):
la exposición, la ganancia, el detector, sus umbrales y el predictor se ajustan desde el
navegador mientras el lazo sigue corriendo.

Trabajan dos hilos, y cada uno hace sólo su parte:
  - el del servidor web VALIDA el pedido (clave declarada, tipo y rango) y lo deja en una cola;
  - el del lazo (main.py) saca la cola al comienzo de cada cuadro, APLICA y confirma.
Así el lazo nunca espera al navegador y nunca recibe un valor sin validar.

PERFILES: un perfil es un juego de valores por objetivo (detector + cámara + umbrales + λ).
Se aplica con un clic y se puede regrabar con lo que esté funcionando; queda en perfiles.json.
"""
#! Este módulo es la ÚNICA puerta de escritura del tablero. Lo que no esté declarado como
#! Ajuste no se puede cambiar por la red: no hay clave para habilitar motores, mandar
#! referencias ni comandos al ESP32, ni para tocar límites de recorrido o ganancias del PI.
import collections
import json
import math
import os
import threading
import time
from dataclasses import dataclass


@dataclass
class Ajuste:
    clave: str
    nombre: str
    grupo: str                 # tarjeta del tablero: "Cámara", "Objetivo", "Detector", "Predicción"
    tipo: str                  # "int" | "float" | "bool" | "opcion"
    valor: object              # valor inicial
    min: float = None
    max: float = None
    paso: float = None
    unidad: str = ""
    opciones: tuple = ()       # sólo tipo "opcion"
    detector: str = ""         # si no está vacío, pertenece a ese detector (se muestra sólo con él)
    en_perfil: bool = True     # se guarda en los perfiles por objetivo
    ayuda: str = ""


class Ajustes:
    def __init__(self, lista, perfiles=None, ruta_perfiles=None):
        self.specs = {a.clave: a for a in lista}
        self.valores = {a.clave: a.valor for a in lista}        # lo APLICADO (lo escribe el lazo)
        self.perfiles = {n: dict(v) for n, v in (perfiles or {}).items()}
        self.ruta_perfiles = ruta_perfiles
        self.perfil_activo = None
        self.perfil_modificado = False                           # hubo cambios después de aplicarlo
        self.version = 0                                         # sube con cada cambio aplicado
        self.historial = collections.deque(maxlen=12)            # (hora, origen, texto)
        self._cola = collections.deque()                         # append/popleft son atómicos
        self._lock = threading.Lock()
        self._cargar_perfiles()

    # ---- validación (hilo del servidor) --------------------------------------------------------
    def validar(self, clave, valor):
        """Devuelve (True, valor_normalizado) o (False, motivo)."""
        a = self.specs.get(clave)
        if a is None:
            return False, "clave desconocida"
        if a.tipo == "bool":
            return (True, valor) if isinstance(valor, bool) else (False, "se esperaba true o false")
        if a.tipo == "opcion":
            return (True, valor) if valor in a.opciones else (False, f"opciones: {list(a.opciones)}")
        #! bool es subclase de int en Python: se descarta aparte para que true no valga 1.
        if isinstance(valor, bool) or not isinstance(valor, (int, float)) or not math.isfinite(valor):
            return False, "se esperaba un número"
        if a.tipo == "int":
            if float(valor) != int(valor):
                return False, "se esperaba un entero"
            valor = int(valor)
        else:
            valor = float(valor)
        if (a.min is not None and valor < a.min) or (a.max is not None and valor > a.max):
            return False, f"fuera de rango [{a.min}, {a.max}]"
        return True, valor

    def pedir(self, cambios, origen=""):
        """Valida y encola. Devuelve (aceptados, rechazados); un rechazo no bloquea al resto."""
        aceptados, rechazados = {}, {}
        if not isinstance(cambios, dict):
            return aceptados, {"": "se esperaba un objeto {clave: valor}"}
        for clave, valor in cambios.items():
            ok, r = self.validar(clave, valor)
            if ok:
                aceptados[clave] = r
            else:
                rechazados[str(clave)] = r
        #* El detector va primero: sus umbrales se aplican después, ya con el detector nuevo.
        for clave in sorted(aceptados, key=lambda c: c != "detector"):
            self._cola.append((clave, aceptados[clave], origen))
        return aceptados, rechazados

    def pedir_perfil(self, nombre, origen=""):
        perfil = self.perfiles.get(nombre)
        if perfil is None:
            return {}, {"perfil": f"no existe; perfiles: {sorted(self.perfiles)}"}
        #* Un perfil puede traer claves que esta corrida no tiene (cámara, con fuente sintética).
        conocidos = {c: v for c, v in perfil.items() if c in self.specs}
        aceptados, rechazados = self.pedir(conocidos, origen)
        if aceptados:
            self._cola.append(("_perfil", nombre, origen))
        return aceptados, rechazados

    def guardar_perfil(self, nombre, origen=""):
        """Guarda en el perfil los valores APLICADOS: los comunes y los del detector activo."""
        if not isinstance(nombre, str) or not nombre.isidentifier() or len(nombre) > 24:
            return False, "nombre de perfil inválido (letras, números y _; máx. 24)"
        det = self.valores.get("detector")
        with self._lock:
            #* Se conservan las claves del perfil que esta corrida no maneja (p. ej. la cámara,
            #* si se guarda desde una corrida con fuente sintética): no se pierden al regrabar.
            nuevo = {c: v for c, v in self.perfiles.get(nombre, {}).items() if c not in self.specs}
            nuevo.update({c: self.valores[c] for c, a in self.specs.items()
                          if a.en_perfil and a.detector in ("", det)})
            self.perfiles[nombre] = nuevo
            self.perfil_activo, self.perfil_modificado = nombre, False
            self.version += 1
            self.historial.append((time.strftime("%H:%M:%S"), origen, f"perfil «{nombre}» guardado"))
            if not self.ruta_perfiles:
                return True, "guardado sólo en memoria (sin archivo de perfiles)"
            try:
                tmp = self.ruta_perfiles + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(self.perfiles, f, indent=2, ensure_ascii=False)
                os.replace(tmp, self.ruta_perfiles)          # reemplazo atómico: nunca queda a medias
            except OSError as ex:
                return False, f"no se pudo escribir {self.ruta_perfiles}: {ex}"
        return True, self.ruta_perfiles

    def _cargar_perfiles(self):
        if not self.ruta_perfiles or not os.path.exists(self.ruta_perfiles):
            return
        try:
            with open(self.ruta_perfiles, encoding="utf-8") as f:
                datos = json.load(f)
        except (OSError, ValueError) as ex:
            print(f"Ajustes: {self.ruta_perfiles} ilegible ({ex}); se usan los perfiles de parametros.py.")
            return
        if isinstance(datos, dict):
            #* El archivo manda sobre la tabla de parametros.py. Cada valor se valida al aplicar.
            self.perfiles.update({n: v for n, v in datos.items() if isinstance(v, dict)})

    # ---- lado del lazo (hilo principal) --------------------------------------------------------
    def pendientes(self):
        """Pedidos ya validados, en orden de llegada: [(clave, valor, origen), ...]."""
        fuera = []
        while self._cola:
            fuera.append(self._cola.popleft())
        return fuera

    def confirmar(self, clave, valor, origen=""):
        """El lazo ya aplicó el cambio: pasa a ser el valor vigente que ven todos los tableros."""
        with self._lock:
            if clave == "_perfil":
                self.perfil_activo, self.perfil_modificado = valor, False
                self.historial.append((time.strftime("%H:%M:%S"), origen, f"perfil «{valor}» aplicado"))
            else:
                self.valores[clave] = valor
                self.perfil_modificado = self.perfil_activo is not None
                a = self.specs[clave]
                self.historial.append((time.strftime("%H:%M:%S"), origen,
                                       f"{a.nombre} = {valor}{(' ' + a.unidad) if a.unidad else ''}"))
            self.version += 1

    # ---- para el tablero -----------------------------------------------------------------------
    def descripcion(self):
        """Esquema completo (una vez): qué controles dibujar y con qué rangos."""
        return {"habilitado": True,
                "ajustes": [{"clave": a.clave, "nombre": a.nombre, "grupo": a.grupo, "tipo": a.tipo,
                             "min": a.min, "max": a.max, "paso": a.paso, "unidad": a.unidad,
                             "opciones": list(a.opciones), "detector": a.detector, "ayuda": a.ayuda}
                            for a in self.specs.values()],
                "perfiles": sorted(self.perfiles), **self.estado()}

    def estado(self):
        """Lo que cambia: va en cada refresco del tablero."""
        return {"v": self.version, "val": dict(self.valores), "perfil": self.perfil_activo,
                "perfil_mod": self.perfil_modificado, "perfiles": sorted(self.perfiles),
                "hist": [list(h) for h in self.historial]}
