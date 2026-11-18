#!/usr/bin/env python3
"""
preparar_fotos.py — convierte fotos del celular en cuadros como los de la cámara de la RPi,
para probar el detector por forma en el PC ANTES de ir al montaje.

Una foto de celular mide miles de píxeles y el avión sale enorme: el detector lo descarta por
tamaño. Este programa deja cada foto en 640x480, en gris, con el avión de unos 150 px de
largo (el tamaño de trabajo), en registros/capturas/<etiqueta>/. De ahí en adelante se usan
las herramientas de siempre.

Cómo sacar las fotos
  - el avión COMPLETO sobre un fondo liso que contraste con él (pared, piso, una sábana);
  - sin la mano ni el palo tocándolo en la imagen (apoyado, o colgado de un hilo);
  - una carpeta (etiqueta) por cara: desde arriba, de lado, de frente;
  - varias fotos por cara, girándolo e inclinándolo un poco.

Uso (PC)
  python3 herramientas/preparar_fotos.py --etiqueta foto_arriba  fotos/arriba_*.jpg
  python3 herramientas/preparar_fotos.py --etiqueta foto_lado    fotos/lado_*.jpg
  python3 herramientas/preparar_fotos.py --etiqueta sin_avion_foto fotos/fondo_*.jpg --sin-objeto

Después:
  python3 herramientas/aprender_forma.py --nombre avion_arriba --imagen "registros/capturas/foto_arriba/*.png"
  python3 herramientas/reproducir_forma.py registros/capturas/foto_* registros/capturas/sin_avion_foto
"""
#? Sirve para comprobar que la silueta del avión real se distingue y que el flujo funciona.
#? No reemplaza la captura con la cámara de la RPi: el celular tiene otra nitidez y otro
#? contraste. La FORMA aprendida de una foto sí sirve de punto de partida, porque el
#? descriptor no depende del tamaño ni del giro.
import argparse
import glob
import os
import sys

import cv2
import numpy as np

RAIZ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
W, H = 640, 480


def objeto_principal(gris, polaridad="oscuro"):
    """Caja (x, y, w, h) de la figura más grande que no toca el borde, o None."""
    #* Contraste LOCAL (realce morfológico con un núcleo grande), igual que el detector: una
    #* foto con sombras o reflejos no tiene un nivel de fondo único, y un umbral global falla.
    chico = cv2.GaussianBlur(gris, (5, 5), 0)
    nucleo = cv2.getStructuringElement(cv2.MORPH_RECT, (201, 201))
    mejor = None
    #! Sólo la polaridad pedida: con las dos, el reflejo de una lámpara sobre la mesa (una mancha
    #! clara y grande) le ganaba al avión y la foto quedaba escalada según el reflejo.
    operaciones = {"oscuro": (cv2.MORPH_BLACKHAT,), "claro": (cv2.MORPH_TOPHAT,)}[polaridad]
    for operacion in operaciones:
        con = cv2.morphologyEx(chico, operacion, nucleo)
        _, m = cv2.threshold(con, max(15.0, 0.4 * float(con.max())), 255, cv2.THRESH_BINARY)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, _, st, _ = cv2.connectedComponentsWithStats(m, 8)
        for k in range(1, n):
            x, y, w, h, a = st[k]
            if x <= 0 or y <= 0 or x + w >= gris.shape[1] or y + h >= gris.shape[0]:
                continue
            if mejor is None or a > mejor[4]:
                mejor = (x, y, w, h, a)
    return mejor[:4] if mejor is not None and mejor[4] > 0.0005 * gris.size else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fotos", nargs="+", help="fotos (jpg, png) o patrones")
    ap.add_argument("--etiqueta", required=True, help="carpeta de salida dentro de registros/capturas/")
    ap.add_argument("--largo", type=int, default=150, help="largo que debe medir el objeto en la salida [px] (por defecto 150)")
    ap.add_argument("--sin-objeto", action="store_true", help="fotos del fondo vacío: sólo se llevan a 640x480")
    ap.add_argument("--polaridad", choices=["oscuro", "claro"], default="oscuro",
                    help="el objeto es más oscuro (por defecto) o más claro que el fondo")
    a = ap.parse_args()
    rutas = [r for p in a.fotos for r in (sorted(glob.glob(p)) or [p])]
    salida = os.path.join(RAIZ, "registros", "capturas", a.etiqueta)
    os.makedirs(salida, exist_ok=True)
    hechas = 0
    for ruta in rutas:
        gris = cv2.imread(ruta, cv2.IMREAD_GRAYSCALE)          # luminancia: el color no se usa
        if gris is None:
            print(f"  no se pudo leer {ruta}")
            continue
        if gris.shape[0] > gris.shape[1]:                      # foto vertical: se acuesta
            gris = cv2.rotate(gris, cv2.ROTATE_90_CLOCKWISE)
        base = cv2.resize(gris, (W, int(round(W * gris.shape[0] / gris.shape[1]))), interpolation=cv2.INTER_AREA)
        esc, centro = 1.0, (base.shape[1] / 2, base.shape[0] / 2)
        if not a.sin_objeto:
            caja = objeto_principal(base, a.polaridad)
            if caja is None:
                print(f"  {os.path.basename(ruta)}: no se distingue un objeto completo (¿toca el borde?, ¿fondo liso?). Se omite.")
                continue
            x, y, w, h = caja
            esc = a.largo / max(w, h)
            centro = (x + w / 2, y + h / 2)
        #* Escala desde la foto original (no desde la reducida), para no perder nitidez, y
        #* recorte de 640x480 centrado en el objeto. Lo que falte se rellena repitiendo el borde.
        f = esc * W / gris.shape[1]
        M = np.float32([[f, 0, W / 2 - centro[0] * esc], [0, f, H / 2 - centro[1] * esc]])
        fuente = cv2.GaussianBlur(gris, (0, 0), 0.5 / f) if f < 0.5 else gris      # evita el aliasing al reducir mucho
        out = cv2.warpAffine(fuente, M, (W, H), flags=cv2.INTER_AREA if f < 1 else cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REPLICATE)
        cv2.imwrite(os.path.join(salida, f"{hechas:03d}.png"), out)
        hechas += 1
    print(f"{hechas} de {len(rutas)} fotos -> {salida}")
    if hechas and not a.sin_objeto:
        print(f"El objeto quedó de unos {a.largo} px de largo, centrado. Revisar un par de PNG antes de seguir.")


if __name__ == "__main__":
    main()
