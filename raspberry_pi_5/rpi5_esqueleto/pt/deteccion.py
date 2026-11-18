"""
deteccion.py — Procesamiento de imagen (PI-2 a PI-6): detectores de los tres objetivos.

Todos con la misma interfaz `procesar(img, usar_ventana, v=None) -> Resultado`:

  DetectorForma      por FORMA, sobre el plano Y: silueta por contraste local y filtro de
                     figura con descriptores de Fourier (pt/formas.py). Es el detector del
                     AVIÓN y del objeto circular (revisión de los ayudantes, 18-11-2026:
                     "detector de imagen, no de luz"). Requiere OpenCV.
  DetectorLaserRojo  por COLOR, sobre el plano V, con confirmación de brillo en Y
                     (algoritmo de mi_proyecto_camara/laser_op.py). Punto láser.
  DetectorLuz        por BRILLO con búsqueda por bloques (flash del celular).
  DetectorLaser      por BRILLO, sobre el plano Y (el original del esqueleto).

Los detectores de punto usan sólo NumPy; el de forma además OpenCV.

  PI-2  recorte a la ventana de interés (seguimiento) o cuadro completo
  PI-3  candidato: pico de brillo (Y) o zona más roja por bloques (V)
  PI-4  la ventana se centra en la detección anterior (asociación trivial: 1 candidato)
  PI-5  centroide subpíxel ponderado alrededor del candidato
  PI-6  validación: área de la mancha y salto máximo respecto del cuadro anterior

El avión NO se detecta por color ni por brillo: el detector por color rojo se retiró el
18-11-2026. Las vistas del avión se APRENDEN del avión real (herramientas/aprender_forma.py,
quedan en formas.json); no hay siluetas de avión incorporadas.

Pendiente: validar el detector por forma con la cámara de la RPi; corrección de distorsión
aplicada al punto.
"""
#* Por qué el detector rojo es el de trabajo: un rojo puro tiene Y ≈ 0,30·R ≈ 76, bajo el
#* umbral de 180 del detector por brillo (salvo que el centro del punto sature), y
#* cualquier lámpara o reflejo blanco le gana al láser en brillo. En el plano V, una
#* fuente blanca queda en ≈ 128 (neutro) y no dispara.
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Resultado:
    valido: bool
    u: float = float("nan")     # centroide subpíxel [px]
    w: float = float("nan")
    pico: int = 0               # nivel máximo en la ventana [0..255]
    area: int = 0               # píxeles de la mancha
    confianza: float = 0.0      # [0, 1]
    motivo: str = ""            # por qué se rechazó (diagnóstico PI-8)
    t_proc_ms: float = 0.0
    forma: str = ""             # detector por forma: plantilla más parecida
    dist: float = float("nan")  # y su distancia (0 = idéntica; se acepta si <= dist_max)


class DetectorLaser:
    def __init__(self, p_det):
        self.p = p_det
        self.ultimo = None      # (u, w) de la última detección válida

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True, v=None):
        """img: matriz 2-D uint8 (luminancia, alto x ancho). v se ignora. Devuelve Resultado."""
        t0 = time.perf_counter()
        r = self._procesar(img, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    # ------------------------------------------------------------------
    def _procesar(self, img, usar_ventana):
        p = self.p
        H, W = img.shape

        # PI-2 / PI-4: ventana de interés
        x0, y0, x1, y1 = 0, 0, W, H
        if usar_ventana and self.ultimo is not None:
            m = p.ventana_seguimiento // 2
            uc, wc = int(round(self.ultimo[0])), int(round(self.ultimo[1]))
            x0, x1 = max(0, uc - m), min(W, uc + m)
            y0, y1 = max(0, wc - m), min(H, wc + m)
        roi = img[y0:y1, x0:x1]

        # PI-3: candidato = pico de brillo
        idx = int(np.argmax(roi))
        wp, up = divmod(idx, roi.shape[1])
        pico = int(roi[wp, up])
        if pico < p.umbral_abs:
            return Resultado(False, pico=pico, motivo="sin pico sobre umbral")

        # PI-5: centroide ponderado en una ventana alrededor del pico
        n = p.medio_lado_mancha
        a0, a1 = max(0, up - n), min(roi.shape[1], up + n + 1)
        b0, b1 = max(0, wp - n), min(roi.shape[0], wp + n + 1)
        parche = roi[b0:b1, a0:a1].astype(np.float32)
        umbral = p.frac_pico * pico
        peso = parche - umbral
        peso[peso < 0] = 0.0
        area = int(np.count_nonzero(peso))
        s = float(peso.sum())
        if s <= 0:
            return Resultado(False, pico=pico, motivo="mancha vacía")
        vv, uu = np.mgrid[b0:b1, a0:a1]
        u = float((peso * uu).sum() / s) + x0
        w = float((peso * vv).sum() / s) + y0

        # PI-6: validación
        if not (p.area_min <= area <= p.area_max):
            return Resultado(False, u, w, pico, area, motivo=f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = np.hypot(u - self.ultimo[0], w - self.ultimo[1])
            if salto > p.salto_max_px:
                return Resultado(False, u, w, pico, area, motivo=f"salto {salto:.0f} px")

        # confianza simple: contraste del pico respecto del umbral
        fondo = float(np.median(roi[::8, ::8]))
        conf = float(np.clip((pico - fondo) / 255.0 * 2.0, 0.0, 1.0))
        return Resultado(True, u, w, pico, area, conf)


# ===========================================================================
class DetectorLaserRojo:
    """Detector de láser ROJO por crominancia (plano V del YUV420), de laser_op.py.
    """
    #* Trabaja en el plano V (media resolución: 4 veces menos píxeles) y devuelve el centroide
    #* en píxeles del plano Y. La muestra V (i, j) cubre los píxeles 2j y 2j+1, cuyo centro es
    #* 2j + 0,5 (con la convención de que el píxel entero n tiene su centro en n).
    #* Pasos: búsqueda por bloques (inmune a un píxel caliente) -> centroide ponderado
    #* alrededor del mejor bloque -> confirmación de brillo en Y (descarta ropa o carteles
    #* rojos oscuros) -> validación de área y salto (PI-6, igual que el detector por brillo).

    V_NEUTRO = 128

    def __init__(self, p_roja):
        self.p = p_roja
        self.ultimo = None      # (u, w) de la última detección válida, en px del plano Y

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True, v=None):
        """img: plano Y (alto x ancho). v: plano V (alto/2 x ancho/2). Devuelve Resultado."""
        t0 = time.perf_counter()
        if v is None:
            r = Resultado(False, motivo="sin plano V (fuente sin color)")
        else:
            r = self._procesar(img, v, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    # ------------------------------------------------------------------
    def _procesar(self, y, v, usar_ventana):
        p = self.p
        h, w = v.shape

        # PI-2 / PI-4: ventana de interés, en coordenadas del plano V
        x0, y0, x1, y1 = 0, 0, w, h
        if usar_ventana and self.ultimo is not None:
            m = max(p.ventana_seguimiento // 4, p.bloque)       # medio lado en px de V
            uc = int(round((self.ultimo[0] - 0.5) / 2))
            wc = int(round((self.ultimo[1] - 0.5) / 2))
            x0, x1 = max(0, uc - m), min(w, uc + m)
            y0, y1 = max(0, wc - m), min(h, wc + m)
        roi = v[y0:y1, x0:x1]
        if roi.size == 0:
            return Resultado(False, motivo="ventana vacía")
        pico_v = int(roi.max())
        if pico_v < p.v_umbral:                       # salida rápida: nada rojo
            return Resultado(False, pico=pico_v, motivo="sin rojo sobre umbral")

        # PI-3: bloque b x b más rojo (suma por bloques)
        b = p.bloque
        hb, wb = roi.shape[0] // b, roi.shape[1] // b
        if hb == 0 or wb == 0:
            idx = int(np.argmax(roi))
            cy, cx = divmod(idx, roi.shape[1])
        else:
            suma = roi[:hb * b, :wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.uint32)
            by, bx = divmod(int(suma.argmax()), wb)
            cx, cy = bx * b + b // 2, by * b + b // 2

        # PI-5: centroide ponderado en un radio 1,5·b alrededor del bloque
        rad = (3 * b) // 2
        a0, a1 = max(0, cx - rad), min(roi.shape[1], cx + rad)
        c0, c1 = max(0, cy - rad), min(roi.shape[0], cy + rad)
        ven = roi[c0:c1, a0:a1]
        pico = int(ven.max())
        if pico < p.v_umbral:
            return Resultado(False, pico=pico, motivo="bloque rojo sin pico")
        corte = self.V_NEUTRO + p.frac_corte * (pico - self.V_NEUTRO)
        peso = ven.astype(np.float32) - corte
        peso[peso < 0] = 0.0
        area_v = int(np.count_nonzero(peso))
        s = float(peso.sum())
        if s <= 0:
            return Resultado(False, pico=pico, motivo="mancha vacía")
        vv, uu = np.mgrid[c0:c1, a0:a1]
        mx = float((peso * uu).sum() / s) + x0
        my = float((peso * vv).sum() / s) + y0
        u_real, w_real = 2.0 * mx + 0.5, 2.0 * my + 0.5
        area = 4 * area_v                               # en px del plano Y

        # Confirmación de brillo en Y (ventana 7x7)
        if p.y_min > 0:
            ui, wi = int(u_real), int(w_real)
            brillo = int(y[max(wi - 3, 0): wi + 4, max(ui - 3, 0): ui + 4].max())
            if brillo < p.y_min:
                return Resultado(False, u_real, w_real, pico, area, motivo=f"rojo sin brillo (Y={brillo})")

        # PI-6: validación
        if not (p.area_min <= area <= p.area_max):
            return Resultado(False, u_real, w_real, pico, area, motivo=f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = np.hypot(u_real - self.ultimo[0], w_real - self.ultimo[1])
            if salto > p.salto_max_px:
                return Resultado(False, u_real, w_real, pico, area, motivo=f"salto {salto:.0f} px")

        conf = float(np.clip((pico - self.V_NEUTRO) / 127.0, 0.0, 1.0))
        return Resultado(True, u_real, w_real, pico, area, conf)


# ===========================================================================
class DetectorLuz:
    """Detector de una FUENTE DE LUZ (flash del celular) en el plano Y. Viene de
    mi_proyecto_camara/luz_2.py (Detector, buscar_zona y centroide_local), con la misma
    interfaz que los otros detectores y la validación PI-6 (área y salto).
    """
    #* Pasos (luz_2): salida rápida si el máximo no llega al umbral -> suma por bloques b×b y
    #* mejor bloque (un píxel caliente aislado no gana a una mancha) -> centroide ponderado con
    #* pesos (Y − 0,7·pico) alrededor del bloque. En SEGUIMIENTO busca primero en una ventana
    #* alrededor de la última detección y, si no encuentra, en el cuadro completo.
    #! Cualquier luz blanca intensa compite (lámparas, ventanas, reflejos): este detector no
    #! distingue color. Para el láser rojo usar DetectorLaserRojo (--detector rojo).

    def __init__(self, p_luz):
        self.p = p_luz
        self.ultimo = None
        self._idx = np.arange(4096, dtype=np.int64)

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True, v=None):
        t0 = time.perf_counter()
        r = self._procesar(img, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    def _centroide(self, y, cx, cy):
        p = self.p
        h, w = y.shape
        r = p.radio_centroide
        x0, x1 = max(cx - r, 0), min(cx + r + 1, w)
        y0, y1 = max(cy - r, 0), min(cy + r + 1, h)
        v = y[y0:y1, x0:x1]
        pico = int(v.max())
        if pico < p.umbral:
            return None
        q = v.astype(np.int16) - int(p.frac_corte * pico)
        np.maximum(q, 0, out=q)
        area = int(np.count_nonzero(q))
        s = int(q.sum())
        if area < p.min_pix or s <= 0:
            return None
        u = x0 + float(q.sum(axis=0) @ self._idx[: x1 - x0]) / s
        ww = y0 + float(q.sum(axis=1) @ self._idx[: y1 - y0]) / s
        return u, ww, pico, area

    def _buscar(self, y, b, x_off=0, y_off=0, y_completa=None):
        if int(y.max()) < self.p.umbral:
            return None
        h, w = y.shape
        hb, wb = h // b, w // b
        if hb == 0 or wb == 0:
            return None
        suma = y[: hb * b, : wb * b].reshape(hb, b, wb, b).sum(axis=(1, 3), dtype=np.uint32)
        by, bx = divmod(int(suma.argmax()), wb)
        #* El centroide se calcula en la imagen completa (la mancha puede salirse de la ventana).
        return self._centroide(y_completa, x_off + bx * b + b // 2, y_off + by * b + b // 2)

    def _procesar(self, img, usar_ventana):
        p = self.p
        H, W = img.shape
        res = None
        if usar_ventana and self.ultimo is not None:
            m = p.ventana_seguimiento // 2
            uc, wc = int(round(self.ultimo[0])), int(round(self.ultimo[1]))
            x0, x1 = max(0, uc - m), min(W, uc + m)
            y0, y1 = max(0, wc - m), min(H, wc + m)
            res = self._buscar(img[y0:y1, x0:x1], p.bloque_ventana, x0, y0, img)
        if res is None:
            res = self._buscar(img, p.bloque_global, 0, 0, img)
        if res is None:
            return Resultado(False, pico=int(img.max()), motivo="sin luz sobre umbral")
        u, w, pico, area = res
        if not (p.area_min <= area <= p.area_max):
            return Resultado(False, u, w, pico, area, motivo=f"área {area} fuera de rango")
        if self.ultimo is not None:
            salto = float(np.hypot(u - self.ultimo[0], w - self.ultimo[1]))
            if salto > p.salto_max_px:
                return Resultado(False, u, w, pico, area, motivo=f"salto {salto:.0f} px")
        conf = float(np.clip((pico - p.umbral) / max(255 - p.umbral, 1) * 0.5 + 0.5, 0.0, 1.0))
        return Resultado(True, u, w, pico, area, conf)


# ===========================================================================
class DetectorForma:
    """Detector por FORMA sobre el plano Y: reconoce la silueta del objetivo, no su brillo ni
    su color. Sirve para el avión (plantillas 'avion_*') y para un objeto circular ('circulo').
    """
    #* Requiere OpenCV (ya está en la RPi: lo usan el tablero y los scripts de cámara).
    #* Pasos, sobre la luminancia (no se usa el plano V ni ningún canal de color):
    #*   PI-2  ventana alrededor de la detección anterior, o cuadro completo a media resolución;
    #*   PI-3  segmentación por CONTRASTE LOCAL: realce morfológico (black-hat para objetos más
    #*         oscuros que su entorno, top-hat para más claros). Mide cuánto se separa cada
    #*         píxel del fondo que lo rodea, así que no depende del nivel de luz de la escena
    #*         ni de un degradado de iluminación. Sólo aparecen objetos más angostos que tam_max_px;
    #*   PI-5  contorno de cada mancha y su centroide de área (subpíxel);
    #*   PI-6  filtro de figura: descriptores de Fourier del contorno (pt/formas.py) comparados
    #*         con las plantillas pedidas; además área, alargamiento y salto entre cuadros.
    #! Condiciones de operación (DET-6): el objetivo debe verse completo, contrastar con el fondo
    #! que lo rodea y no tocar otra figura de contraste parecido (una mano, un palo grueso).
    #! Un fondo liso da la silueta más limpia; con fondo recargado suben los rechazos.
    #* Las plantillas del avión no vienen incorporadas: se aprenden del avión real con
    #* herramientas/aprender_forma.py (formas.json). Sin plantillas de su familia, el detector
    #* no entrega detecciones y lo dice en el motivo. La única incorporada es el círculo.
    #? Sin validar aún con la cámara de la RPi: las plantillas vigentes salen de fotos de celular.

    def __init__(self, p_forma, banco=None):
        import cv2                                   # perezoso: los demás detectores no lo necesitan
        from .formas import BancoDeFormas, descriptor, momentos, normalizar_afin
        self.cv2, self.p = cv2, p_forma
        self.banco = banco if banco is not None else BancoDeFormas()
        self._descriptor, self._momentos, self._normalizar = descriptor, momentos, normalizar_afin
        self._nucleos = {}
        self.ultimo = None
        self.diagnostico = []        # candidatos del último cuadro: (u, w, área, plantilla, distancia)
        #* Para aprender formas (herramientas/aprender_forma.py): si es una lista, cada figura
        #* medida se agrega como dict con su descriptor y su contorno en píxeles del cuadro.
        self.detalle = None

    def reiniciar(self):
        self.ultimo = None

    def procesar(self, img, usar_ventana=True, v=None):
        """img: plano Y (alto x ancho, uint8). v se ignora. Devuelve Resultado."""
        t0 = time.perf_counter()
        r = self._procesar(img, usar_ventana)
        r.t_proc_ms = (time.perf_counter() - t0) * 1e3
        if r.valido:
            self.ultimo = (r.u, r.w)
        return r

    # ------------------------------------------------------------------
    def _nucleo(self, k, forma=None):
        cv2 = self.cv2
        forma = cv2.MORPH_RECT if forma is None else forma
        if (k, forma) not in self._nucleos:
            self._nucleos[(k, forma)] = cv2.getStructuringElement(forma, (k, k))
        return self._nucleos[(k, forma)]

    def _contraste(self, gris, esc):
        """Mapas de contraste local (0..255): cuánto se aparta cada píxel de su entorno.
        Uno por polaridad: objetos más oscuros que el fondo y objetos más claros.
        Devuelve [(contraste, gris suavizado, signo)], con signo −1 = oscuro y +1 = claro."""
        cv2, p = self.cv2, self.p
        #* El realce morfológico amplifica el ruido de píxel: se suaviza antes. A media
        #* resolución la reducción ya promedió 2x2, así que basta un suavizado menor.
        g = cv2.GaussianBlur(gris, (5, 5), 0) if esc == 1 else cv2.blur(gris, (3, 3))
        nuc = self._nucleo(max(3, int(p.tam_max_px / esc)) | 1)
        #! Las dos polaridades NO se mezclan en un solo mapa: junto a un objeto oscuro con
        #! entrantes (entre el ala y la cola), el fondo claro encerrado aparece como "objeto
        #! claro angosto" y, sumado al avión, le rellena la silueta. Se buscan por separado.
        mapas = []
        if p.polaridad in ("oscuro", "ambos"):
            mapas.append((cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, nuc), g, -1))
        if p.polaridad in ("claro", "ambos"):
            mapas.append((cv2.morphologyEx(g, cv2.MORPH_TOPHAT, nuc), g, +1))
        return mapas

    def _candidatos(self, gris, esc, x0, y0):
        """Figuras de la región `gris` (reducida `esc` veces, con origen (x0, y0) en el cuadro).
        Devuelve (lista de (distancia, u, w, área, plantilla), contraste máximo, nº de manchas)."""
        salida, pico, n = [], 0, 0
        for con, g, signo in self._contraste(gris, esc):
            s, pk, nn = self._candidatos_de(con, esc, x0, y0, gris.shape, g, signo)
            salida += s
            pico, n = max(pico, pk), n + nn
        salida.sort(key=lambda s: s[0])
        return salida, pico, n

    def _candidatos_de(self, con, esc, x0, y0, forma_gris, g=None, signo=-1):
        cv2, p = self.cv2, self.p
        pico = int(con.max())
        if pico < p.contraste_min:
            return [], pico, 0
        _, mascara = cv2.threshold(con, p.contraste_min - 1, 255, cv2.THRESH_BINARY)
        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        H, W = forma_gris
        e2 = esc * esc
        manchas = []
        for c in contornos:
            area = cv2.contourArea(c) * e2
            if not (p.area_min <= area <= p.area_max):
                continue
            manchas.append((area, c, cv2.boundingRect(c)))
        manchas.sort(key=lambda m: -m[0])
        salida = []
        for area, c, (bx, by, bw, bh) in manchas[: p.max_candidatos]:
            #* Borde a media altura: la mancha se encontró con el umbral bajo (contraste_min),
            #* que la engorda 1 o 2 px por el desenfoque. La figura se mide donde el gris está
            #* a medio camino (frac_borde) entre el nivel del objeto y el del fondo, que es donde
            #* está el borde real. Sin esto, las alas quedan gruesas y la figura ya no es la misma.
            dentro = np.zeros((bh, bw), np.uint8)
            cv2.drawContours(dentro, [c], -1, 255, -1, offset=(-bx, -by))
            sub = con[by:by + bh, bx:bx + bw]
            pico_m = cv2.minMaxLoc(sub, mask=dentro)[1]
            if g is None:
                umbral = cv2.threshold(sub, max(p.contraste_min, p.frac_borde * pico_m) - 1, 255, cv2.THRESH_BINARY)[1]
            else:
                #* El fondo NO es parejo: en la grabación real del 18-11 iba de 64 a 99 de gris
                #* dentro del cuadro, con el avión en 50. Medir la "mitad" contra el máximo de
                #* contraste recortaba el avión donde el fondo era más oscuro. La mitad se toma
                #* píxel a píxel, contra el fondo LOCAL (el gris más el contraste reconstruye
                #* el fondo que el realce morfológico estimó en ese punto).
                gs = g[by:by + bh, bx:bx + bw].astype(np.float32)
                subf = sub.astype(np.float32)
                nivel = float(np.median(gs[(sub >= 0.7 * pico_m) & (dentro > 0)]))     # gris típico del objeto
                altura = (gs + subf - nivel) if signo < 0 else (nivel - gs + subf)     # |fondo local − objeto|
                umbral = ((subf >= p.frac_borde * altura) * 255).astype(np.uint8)
            figura = np.zeros((bh + 2, bw + 2), np.uint8)
            figura[1:-1, 1:-1] = cv2.bitwise_and(umbral, dentro)
            if p.abrir_frac > 0:
                #* Quitar el palo o hilo que sostiene al avión. La apertura (de tamaño proporcional
                #* a la figura, para no depender de la distancia) borra todo lo delgado: el palo,
                #* pero también las puntas de las alas. Por eso después se devuelve lo que quedó a
                #* menos de k píxeles del cuerpo: las puntas vuelven y del palo sólo queda un muñón.
                k = int(round(p.abrir_frac * (area / e2) ** 0.5)) | 1
                if k >= 3:
                    cuerpo = cv2.morphologyEx(figura, cv2.MORPH_OPEN, self._nucleo(k, cv2.MORPH_ELLIPSE))
                    figura = cv2.bitwise_and(cv2.dilate(cuerpo, self._nucleo(2 * k + 1, cv2.MORPH_ELLIPSE)), figura)
            partes, _ = cv2.findContours(figura, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not partes:
                continue
            pts = max(partes, key=cv2.contourArea).reshape(-1, 2).astype(np.float64) + (bx - 1, by - 1)
            if len(pts) < 8:
                continue
            #* Cortada por el borde de la región: su figura no es la real. Se revisa DESPUÉS de
            #* la apertura, para que un palo que llega al borde no descarte al avión.
            (fx0, fy0), (fx1, fy1) = pts.min(axis=0), pts.max(axis=0)
            if fx0 <= 0 or fy0 <= 0 or fx1 >= W - 1 or fy1 >= H - 1:
                continue
            norm = self._normalizar(pts)
            if norm is None or norm[1] > p.alargamiento_max:
                continue
            d = self._descriptor(norm[0], afin=False)
            mom = self._momentos(pts)
            if d is None or mom is None:
                continue
            nombre, dist = self.banco.mejor(d, p.formas)
            #* Coordenadas del cuadro completo: el píxel reducido j cubre esc píxeles reales.
            u = x0 + mom[1][0] * esc + (esc - 1) / 2.0
            w = y0 + mom[1][1] * esc + (esc - 1) / 2.0
            #* El detalle se entrega aunque todavía no haya plantillas de la familia (nombre None):
            #* así se puede aprender la PRIMERA vista (aprender_forma.py mide con este mismo código).
            if self.detalle is not None:
                self.detalle.append({"u": u, "w": w, "area": int(mom[0] * e2), "forma": nombre or "", "dist": dist,
                                     "descriptor": d, "alargamiento": norm[1],
                                     "contorno": np.array([x0, y0]) + pts * esc + (esc - 1) / 2.0})
            if nombre is None:
                continue
            salida.append((dist, u, w, int(mom[0] * e2), nombre))
        return salida, pico, len(manchas)

    def _en_ventana(self, img, centro):
        """Candidatos a resolución completa en la ventana centrada en `centro`."""
        H, W = img.shape
        m = self.p.ventana_seguimiento // 2
        uc, wc = int(round(centro[0])), int(round(centro[1]))
        x0, x1 = max(0, uc - m), min(W, uc + m)
        y0, y1 = max(0, wc - m), min(H, wc + m)
        if x1 - x0 < 16 or y1 - y0 < 16:
            return [], 0, 0
        return self._candidatos(np.ascontiguousarray(img[y0:y1, x0:x1]), 1, x0, y0)

    def _procesar(self, img, usar_ventana):
        cv2, p = self.cv2, self.p
        H, W = img.shape
        cand, pico, n = [], 0, 0
        if not self.banco.de_la_familia(p.formas):
            #! Sin plantillas no hay con qué comparar. Pasa con el avión si falta formas.json.
            self.diagnostico = []
            return Resultado(False, motivo=f"sin plantillas de {'/'.join(p.formas)}: aprender una vista "
                                           f"(herramientas/aprender_forma.py)")
        if usar_ventana and self.ultimo is not None:
            cand, pico, n = self._en_ventana(img, self.ultimo)
            cand = [c for c in cand if c[0] <= p.dist_max]
        if not cand:
            #* Búsqueda en el cuadro completo, reducido: cuesta la cuarta parte. Lo que se
            #* encuentre se vuelve a medir a resolución completa (centroide más fino).
            esc = max(1, int(p.reducir_busqueda))
            chico = cv2.resize(img, (W // esc, H // esc), interpolation=cv2.INTER_AREA) if esc > 1 \
                else np.ascontiguousarray(img)
            cand, pico, n = self._candidatos(chico, esc, 0, 0)
            if esc > 1:
                #* A media resolución la figura sale más tosca: se da un margen de 1,5 veces
                #* para decidir qué volver a medir, y el veredicto se toma a resolución completa.
                for c in [c for c in cand if c[0] <= 1.5 * p.dist_max][:2]:
                    fino = [f for f in self._en_ventana(img, c[1:3])[0] if f[0] <= p.dist_max]
                    if fino:
                        cand = fino
                        break
        self.diagnostico = [(c[1], c[2], c[3], c[4], c[0]) for c in cand[:5]]
        if not cand:
            if pico < p.contraste_min:
                return Resultado(False, pico=pico, motivo="sin contraste sobre umbral")
            return Resultado(False, pico=pico, motivo="sin figuras del tamaño pedido" if n == 0 else "figuras muy alargadas")
        buenos = [c for c in cand if c[0] <= p.dist_max]
        if not buenos:
            dist, u, w, area, nombre = cand[0]
            return Resultado(False, u, w, pico, area, motivo=f"ninguna figura parecida (mejor: {nombre}, d={dist:.2f})",
                             forma=nombre, dist=dist)
        if self.ultimo is not None:
            cerca = [c for c in buenos if np.hypot(c[1] - self.ultimo[0], c[2] - self.ultimo[1]) <= p.salto_max_px]
            if not cerca:
                dist, u, w, area, nombre = buenos[0]
                salto = float(np.hypot(u - self.ultimo[0], w - self.ultimo[1]))
                return Resultado(False, u, w, pico, area, motivo=f"salto {salto:.0f} px", forma=nombre, dist=dist)
            buenos = cerca
        dist, u, w, area, nombre = buenos[0]
        conf = float(np.clip(1.0 - dist / max(p.dist_max, 1e-6), 0.0, 1.0)) * 0.5 + 0.5
        return Resultado(True, u, w, pico, area, conf, forma=nombre, dist=dist)
