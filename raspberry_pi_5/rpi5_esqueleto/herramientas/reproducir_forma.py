"""
reproducir_forma.py — Prueba el detector por forma sobre cuadros GRABADOS, sin cámara ni torreta.

Los cuadros salen de herramientas/grabar_cuadros.py (o de cualquier foto en png/jpg o .npz con cuadros Y).
Cada CARPETA es una etiqueta; las que empiezan por "sin_" son escenas SIN avión: cualquier detección
válida en ellas cuenta como falso positivo.

Uso:
  python3 herramientas/reproducir_forma.py registros/capturas/*
  python3 herramientas/reproducir_forma.py capturas/avion_lado_2m --set dist_max=0.25 --set contraste_min=40
  python3 herramientas/reproducir_forma.py capturas/* --barrer dist_max=0.15,0.2,0.25,0.3
  python3 herramientas/reproducir_forma.py capturas/avion_lado_2m --secuencia     # cuadros seguidos: ventana y salto
Opciones útiles:
  --set clave=valor     cambia un parámetro de DeteccionForma sólo para esta prueba (se repite);
                        p. ej. dist_max, contraste_min, area_min, tam_max_px, polaridad, salto_max_px
  --barrer clave=v1,v2  repite todo con cada valor y compara el % de aciertos y de falsos
  --familia ovni        usa los parámetros del objeto circular (por defecto, avion)
Deja en registros/reproduccion/: reproduccion.csv y, por cada cuadro, un PNG con las figuras que el
detector midió (amarillo, con plantilla y distancia), la elegida (verde) y el motivo si se rechazó.
Para ver cómo cambia con otros umbrales no hace falta volver a grabar: se repite esto con otro --set.
"""
import argparse
import collections
import csv
import dataclasses
import glob
import os
import sys

import cv2
import numpy as np

RAIZ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, RAIZ)
from pt import formas as fm                                       # noqa: E402
from pt.deteccion import DetectorForma                            # noqa: E402
from pt.parametros import Parametros                              # noqa: E402

EXT = (".png", ".jpg", ".jpeg", ".bmp")


def cargar(rutas):
    """[(etiqueta, nombre_de_cuadro, imagen_Y)] en orden; una carpeta = una etiqueta."""
    cuadros = []
    for r in rutas:
        for ruta in sorted(glob.glob(r)) or [r]:
            if os.path.isdir(ruta):
                etiqueta = os.path.basename(os.path.normpath(ruta))
                archivos = sorted(f for f in os.listdir(ruta) if f.lower().endswith(EXT) and f != "contacto.jpg")
                archivos = [os.path.join(ruta, f) for f in archivos]
            else:
                etiqueta, archivos = os.path.basename(os.path.dirname(os.path.abspath(ruta))), [ruta]
            for f in archivos:
                if f.endswith(".npz"):
                    continue
                img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)         # luminancia: el color no se usa
                if img is None:
                    print(f"  no se pudo leer {f}")
                    continue
                cuadros.append((etiqueta, os.path.basename(f), img))
    return cuadros


def parametros(familia, cambios):
    P = Parametros()
    p = dataclasses.replace(P.forma_avion if familia == "avion" else P.forma_ovni)
    tipos = {f.name: type(getattr(p, f.name)) for f in dataclasses.fields(p)}
    for par in cambios:
        clave, _, valor = par.partition("=")
        clave = clave.split(".")[-1].strip()                      # acepta "avion.dist_max"
        if clave not in tipos or not _:
            sys.exit(f"--set {par!r}: parámetro desconocido. Válidos: {', '.join(tipos)}")
        if tipos[clave] is tuple:
            sys.exit(f"--set {clave}: no se puede cambiar desde aquí (es una lista)")
        setattr(p, clave, tipos[clave](valor))
    return p


def correr(cuadros, p, banco, secuencia, dibujar=None):
    """Pasa todos los cuadros por un detector nuevo. Devuelve una lista de dicts, uno por cuadro."""
    det = DetectorForma(p, banco)
    filas, etiqueta_ant = [], None
    for etiqueta, nombre, img in cuadros:
        if not secuencia or etiqueta != etiqueta_ant:
            det.reiniciar()
        etiqueta_ant = etiqueta
        det.detalle = []
        r = det.procesar(img, usar_ventana=secuencia)
        filas.append({"etiqueta": etiqueta, "cuadro": nombre, "valido": int(r.valido), "u": r.u, "w": r.w,
                      "area": r.area, "forma": r.forma, "dist": r.dist, "motivo": r.motivo, "t_ms": r.t_proc_ms})
        if dibujar:
            dibujar(img, r, det.detalle, etiqueta, nombre)
    return filas


def resumen(filas):
    por = collections.OrderedDict()
    for f in filas:
        por.setdefault(f["etiqueta"], []).append(f)
    out = []
    for etiqueta, fs in por.items():
        validos = [f for f in fs if f["valido"]]
        d = [f["dist"] for f in fs if f["dist"] == f["dist"]]
        motivos = collections.Counter(f["motivo"].split(" (")[0] or "—" for f in fs if not f["valido"])
        out.append({"etiqueta": etiqueta, "n": len(fs), "validos": len(validos), "pct": 100.0 * len(validos) / len(fs),
                    "negativo": etiqueta.startswith("sin_"), "dist_med": float(np.median(d)) if d else float("nan"),
                    "motivos": motivos.most_common(3), "t_med": float(np.median([f["t_ms"] for f in fs]))})
    return out


def imprimir_resumen(res, titulo=""):
    print(f"\n{titulo}{'etiqueta':<24}{'cuadros':>8}{'válidos':>9}{'%':>7}  {'dist med':>8}  {'ms':>5}  motivos más frecuentes")
    for r in res:
        etiqueta = r["etiqueta"] + ("  [sin avión: lo válido es FALSO]" if r["negativo"] else "")
        m = "; ".join(f"{k} x{v}" for k, v in r["motivos"])
        print(f"{r['etiqueta']:<24}{r['n']:>8}{r['validos']:>9}{r['pct']:>6.0f}%  {r['dist_med']:>8.2f}  {r['t_med']:>5.1f}  {m}"
              + ("   <- falsos positivos" if r["negativo"] and r["validos"] else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rutas", nargs="+", help="carpetas (una por etiqueta), archivos o patrones")
    ap.add_argument("--familia", choices=["avion", "ovni"], default="avion")
    ap.add_argument("--set", dest="cambios", action="append", default=[], metavar="CLAVE=VALOR")
    ap.add_argument("--barrer", metavar="CLAVE=V1,V2,...", help="repite la prueba con cada valor")
    ap.add_argument("--secuencia", action="store_true", help="tratar los cuadros de cada carpeta como una secuencia (ventana y salto)")
    ap.add_argument("--formas", default=os.path.join(RAIZ, "formas.json"), help="plantillas aprendidas (por defecto formas.json)")
    ap.add_argument("--salida", default=os.path.join(RAIZ, "registros", "reproduccion"))
    ap.add_argument("--sin-imagenes", action="store_true", help="no escribir los PNG anotados")
    a = ap.parse_args()

    cuadros = cargar(a.rutas)
    if not cuadros:
        sys.exit("No se encontraron cuadros.")
    banco = fm.BancoDeFormas(a.formas)
    p = parametros(a.familia, a.cambios)
    print(f"{len(cuadros)} cuadros en {len({c[0] for c in cuadros})} etiquetas | plantillas de '{a.familia}': "
          f"{', '.join(banco.de_la_familia(p.formas))}"
          f"{'' if banco.aprendidas else '  (AVISO: sólo genéricas; aprender las del avión real)'}")

    if a.barrer:
        clave, _, valores = a.barrer.partition("=")
        if not _:
            sys.exit("--barrer clave=v1,v2,...")
        print(f"\nBarrido de {clave}: % de cuadros con detección válida por etiqueta")
        etiquetas = list(dict.fromkeys(c[0] for c in cuadros))
        print(f"{clave:>12}  " + "  ".join(f"{e[:16]:>16}" for e in etiquetas))
        for v in valores.split(","):
            pv = parametros(a.familia, a.cambios + [f"{clave}={v}"])
            res = {r["etiqueta"]: r for r in resumen(correr(cuadros, pv, banco, a.secuencia))}
            print(f"{v:>12}  " + "  ".join(f"{res[e]['pct']:>15.0f}%" + ("!" if res[e]["negativo"] and res[e]["validos"] else " ")
                                           for e in etiquetas))
        print("('!' = falsos positivos en una escena sin avión)")
        return

    def dibujar(img, r, detalle, etiqueta, nombre):
        if a.sin_imagenes:
            return
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for c in detalle:
            cv2.polylines(bgr, [np.round(c["contorno"]).astype(np.int32)], True, (0, 220, 255), 1)
            cv2.putText(bgr, f"{c['forma']} d={c['dist']:.2f}", (int(c["u"]) + 6, int(c["w"]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1, cv2.LINE_AA)
        if r.valido:
            cv2.drawMarker(bgr, (int(r.u), int(r.w)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(bgr, ("OK " + r.forma if r.valido else r.motivo)[:70], (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if r.valido else (0, 0, 255), 1, cv2.LINE_AA)
        carpeta = os.path.join(a.salida, etiqueta)
        os.makedirs(carpeta, exist_ok=True)
        cv2.imwrite(os.path.join(carpeta, os.path.splitext(nombre)[0] + ".png"), bgr)

    filas = correr(cuadros, p, banco, a.secuencia, dibujar)
    os.makedirs(a.salida, exist_ok=True)
    ruta_csv = os.path.join(a.salida, "reproduccion.csv")
    with open(ruta_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0]))
        w.writeheader()
        w.writerows({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in fila.items()} for fila in filas)
    imprimir_resumen(resumen(filas))
    print(f"\nCSV: {ruta_csv}" + ("" if a.sin_imagenes else f"\nImágenes anotadas: {a.salida}/<etiqueta>/"))
    if a.cambios:
        print("Parámetros cambiados sólo para esta prueba: " + ", ".join(a.cambios))


if __name__ == "__main__":
    main()
