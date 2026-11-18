#!/usr/bin/env python3
"""
aprender_forma.py — enseña al detector por forma una VISTA del objetivo real.

El detector (pt/deteccion.py, DetectorForma) compara la silueta que ve con plantillas. Las
incorporadas ("avion_planta") son dibujos genéricos: con otro modelo de avión
no sirven. Este programa mide la silueta del objetivo verdadero y la guarda en formas.json,
junto a main.py. Desde ese momento, las plantillas aprendidas de una familia ("avion_*")
reemplazan a las genéricas.

Cómo tomar la muestra
  - el objetivo solo, completo y cerca del centro del cuadro, sobre un fondo liso;
  - sin el palo a la vista (o con el mismo soporte delgado que se usará en la demostración);
  - misma exposición y ganancia que en la demostración;
  - durante la captura, girarlo e inclinarlo un poco SIN cambiar de cara. Una plantilla por
    cara: desde arriba, de lado, de frente...

Uso
  python3 herramientas/aprender_forma.py --nombre avion_arriba --camara --exposicion 6000 --ganancia 2
  python3 herramientas/aprender_forma.py --nombre avion_lado --imagen fotos/lado_*.jpg
  python3 herramientas/aprender_forma.py --nombre ovni --camara --familia ovni   # para el objeto circular no hace falta: basta "circulo"
  python3 herramientas/aprender_forma.py --listar
  python3 herramientas/aprender_forma.py --borrar avion_lado

Al terminar imprime qué tan estable salió la figura entre cuadros (de ahí sale el dist_max
recomendado) y deja una imagen con el contorno medido, para revisarlo a ojo.
"""
#! El nombre decide la familia: "avion_arriba" y "avion_lado" valen para el detector "avion";
#! un nombre que no empiece por "avion_" no lo usará ese detector.
import argparse
import dataclasses
import glob
import json
import os
import sys

import cv2
import numpy as np

RAIZ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, RAIZ)
from pt import formas as fm                                       # noqa: E402
from pt.deteccion import DetectorForma                            # noqa: E402
from pt.parametros import Parametros                              # noqa: E402

RUTA_FORMAS = os.path.join(RAIZ, "formas.json")


def cuadros_de_imagenes(patrones):
    for patron in patrones:
        for ruta in sorted(glob.glob(patron)) or [patron]:
            if ruta.endswith(".npz"):                      # p. ej. cuadros.npz de medir_avion.py
                with np.load(ruta) as z:
                    for clave in (["y"] if "y" in z.files else z.files):   # medir_avion.py guarda y (luminancia) y v
                        arr = z[clave]
                        if arr.dtype == np.uint8 and arr.ndim in (2, 3) and arr.shape[-1] >= 320:
                            for y in (arr if arr.ndim == 3 else [arr]):
                                yield y
                continue
            img = cv2.imread(ruta, cv2.IMREAD_GRAYSCALE)   # luminancia: el color no se usa
            if img is None:
                print(f"  no se pudo leer {ruta}")
                continue
            yield img


def cuadros_de_camara(P, n):
    from pt.camara import FuenteCamara
    fuente = FuenteCamara(P.captura, None)
    try:
        for k, c in enumerate(fuente):
            if k >= 20:                                    # los primeros cuadros: la exposición se asienta
                yield np.ascontiguousarray(c.img)
            if k >= 20 + n:
                break
    finally:
        fuente.cerrar()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nombre", help="nombre de la plantilla, p. ej. avion_arriba, avion_lado")
    ap.add_argument("--camara", action="store_true", help="capturar con la cámara de la RPi")
    ap.add_argument("--imagen", nargs="+", help="fotos (jpg, png) o un .npz con cuadros Y")
    ap.add_argument("--cuadros", type=int, default=120, help="cuadros a capturar con --camara (por defecto 120 = 1,5 s)")
    ap.add_argument("--exposicion", type=int, help="µs (con --camara)")
    ap.add_argument("--ganancia", type=float, help="ganancia analógica (con --camara)")
    ap.add_argument("--familia", choices=["avion", "ovni"], default="avion", help="parámetros de segmentación a usar")
    ap.add_argument("--polaridad", choices=["oscuro", "claro", "ambos"], help="por defecto, la de parametros.py")
    ap.add_argument("--contraste-min", type=int)
    ap.add_argument("--elegir", choices=["centro", "mayor"], default="centro",
                    help="qué figura del cuadro es el objetivo: la más cercana al centro o la más grande")
    ap.add_argument("--listar", action="store_true")
    ap.add_argument("--borrar", metavar="NOMBRE")
    a = ap.parse_args()

    banco = fm.BancoDeFormas(RUTA_FORMAS)
    if a.listar:
        print(f"Plantillas incorporadas: {', '.join(fm.INCORPORADAS)}")
        print(f"Aprendidas ({RUTA_FORMAS}): {', '.join(sorted(banco.aprendidas)) or 'ninguna'}")
        return
    if a.borrar:
        if a.borrar not in banco.aprendidas:
            sys.exit(f"'{a.borrar}' no está entre las aprendidas: {sorted(banco.aprendidas)}")
        datos = json.load(open(RUTA_FORMAS, encoding="utf-8"))
        datos.pop(a.borrar)
        json.dump(datos, open(RUTA_FORMAS, "w", encoding="utf-8"), indent=1)
        print(f"Borrada '{a.borrar}'. Quedan: {sorted(datos) or 'ninguna (vuelven las genéricas)'}")
        return
    if not a.nombre or not (a.camara or a.imagen):
        ap.error("indicar --nombre y de dónde sale la muestra: --camara o --imagen")

    P = Parametros()
    if a.exposicion:
        P.captura.exposicion_us = a.exposicion
    if a.ganancia:
        P.captura.ganancia = a.ganancia
    p = dataclasses.replace(P.forma_avion if a.familia == "avion" else P.forma_ovni, abrir_frac=P.forma_avion.abrir_frac
                            if a.familia == "avion" else 0.0)
    if a.polaridad:
        p.polaridad = a.polaridad
    if a.contraste_min:
        p.contraste_min = a.contraste_min
    det = DetectorForma(p, banco)

    descs, ultimo, vistos = [], None, 0
    for y in (cuadros_de_camara(P, a.cuadros) if a.camara else cuadros_de_imagenes(a.imagen)):
        vistos += 1
        det.detalle = []
        for con, g, signo in det._contraste(np.ascontiguousarray(y), 1):       # a resolución completa
            det._candidatos_de(con, 1, 0, 0, y.shape, g, signo)
        if not det.detalle:
            continue
        H, W = y.shape
        f = (max(det.detalle, key=lambda d: d["area"]) if a.elegir == "mayor"
             else min(det.detalle, key=lambda d: np.hypot(d["u"] - W / 2, d["w"] - H / 2)))
        descs.append(f["descriptor"])
        ultimo = (y, f)
    if len(descs) < 1:
        sys.exit(f"En {vistos} cuadros no apareció ninguna figura. Revisar: polaridad ({p.polaridad}), contraste mínimo "
                 f"({p.contraste_min}), que el objetivo se vea completo y que su área supere {p.area_min} px.")

    D = np.array(descs)
    plantilla = np.median(D, axis=0)
    dist = np.array([fm.distancia(d, plantilla) for d in D])
    print(f"\nFigura medida en {len(descs)} de {vistos} cuadros. Área {ultimo[1]['area']} px, alargamiento {ultimo[1]['alargamiento']:.1f}.")
    if len(descs) >= 5:
        p95 = float(np.percentile(dist, 95))
        print(f"Estabilidad entre cuadros (distancia a la plantilla): mediana {np.median(dist):.3f}, p95 {p95:.3f}, máx {dist.max():.3f}")
        print(f"  -> dist_max recomendado: {max(0.10, round(1.5 * p95 + 0.03, 2)):.2f}  (hoy: {p.dist_max:.2f}; se cambia en el tablero o en parametros.py)")
    otras = {n: fm.distancia(plantilla, q) for n, q in banco.plantillas.items() if n != a.nombre}
    if otras:
        print("Parecido con las demás plantillas: " + ", ".join(f"{n} {v:.2f}" for n, v in sorted(otras.items(), key=lambda kv: kv[1])))
    if ultimo[1]["area"] < 400:
        print("AVISO: la figura mide menos de 400 px de área. Acercar el objetivo: de lejos la silueta pierde detalle.")
    if ultimo[1]["alargamiento"] > 4:
        print("AVISO: figura muy alargada (más de 4 a 1). Las vistas delgadas se reconocen mal; preferir otra cara.")

    banco.guardar(a.nombre, plantilla)
    y, f = ultimo
    bgr = cv2.cvtColor(y, cv2.COLOR_GRAY2BGR)
    cv2.polylines(bgr, [np.round(f["contorno"]).astype(np.int32)], True, (0, 220, 0), 1)
    cv2.drawMarker(bgr, (int(f["u"]), int(f["w"])), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)
    salida = os.path.join(RAIZ, "registros", f"forma_{a.nombre}.png")
    os.makedirs(os.path.dirname(salida), exist_ok=True)
    cv2.imwrite(salida, bgr)
    print(f"\nGuardada '{a.nombre}' en {RUTA_FORMAS}")
    print(f"Contorno medido (revisarlo): {salida}")
    print(f"Plantillas que usará el detector '{a.familia}': {', '.join(banco.de_la_familia(p.formas))}")


if __name__ == "__main__":
    main()
