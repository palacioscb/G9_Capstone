"""
parametros.py — PI-7 (calibración y parámetros) y parámetros de todos los bloques.

Un solo lugar para todos los números con unidades. Los valores marcados
NOMINAL se reemplazan por valores medidos (benchmark de cámara, calibración).
"""
#* Convención de signos (igual que el firmware esp32_pan_tilt/include/config.h):
#*   pan  > 0 -> la cámara gira a la DERECHA;  tilt > 0 -> la cámara mira hacia ARRIBA.
#*   Imagen: u crece a la derecha, w crece hacia abajo, origen arriba a la izquierda.
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Óptica — Waveshare Pi5-IMX219-77 (IMX219, 3280x2464, píxel 1,12 µm)
# ---------------------------------------------------------------------------
@dataclass
class Optica:
    pix_um: float = 1.12            # tamaño de píxel nativo [µm]
    f_mm: float = 2.85              #todo NOMINAL: reemplazar por la focal calibrada
    sensor_px: tuple = (3280, 2464)  # arreglo activo [px nativos]


# ---------------------------------------------------------------------------
# PI-1 — Captura
# ---------------------------------------------------------------------------
@dataclass
class Captura:
    salida_px: tuple = (640, 480)        # tamaño de la matriz entregada (ancho, alto)
    #! Fijar SIEMPRE el modo 1640x1232: si no, a 640x480 libcamera escoge el modo
    #! recortado (28° de campo en vez de 65,6°). Es lo que hacen luz_op.py y laser_op.py.
    modo_sensor_px: tuple = (1640, 1232)  # binning 2x2, campo completo
    recorte_sensor: tuple = (0, 0, 3280, 2464)  # se reemplaza por ScalerCrop real
    fps: float = 80.0                    #* decisión del informe del 25 % (el modo llega a 81,07)
    exposicion_us: int = 2000            #* fija (25 %: menos latencia y desenfoque); 0 = automática
    ganancia: float = 1.0                # ganancia analógica con exposición fija
    #* Balance de blancos FIJO: el detector rojo usa el plano V (crominancia) y con AWB
    #* automático el nivel de V cambiaría con la escena.
    #? (rojo, azul). Con (1,0; 1,0) el láser puede no superar el umbral de V; laser_op.py
    #? nunca enganchó en sus capturas. Subir el rojo si la columna pico_v queda baja.
    ganancias_color: tuple = (2.0, 1.5)
    reduccion_ruido: bool = False        # sin NR: menos trabajo del ISP
    #? Tiempo de lectura de todas las filas (persiana rodante) en el modo 1640x1232. Estimado:
    #? el modo llega a 81,07 fps (12,3 ms por cuadro), casi todo lectura. Medirlo (p. ej. con
    #? el láser barriendo rápido: la mancha se inclina) y reemplazar.
    t_lectura_ms: float = 12.0


# ---------------------------------------------------------------------------
# PI-3..PI-6 — Detectores
# ---------------------------------------------------------------------------
@dataclass
class Deteccion:
    #* Detector por BRILLO (plano Y): el original del esqueleto.
    umbral_abs: int = 180         # nivel mínimo de gris del pico para considerar candidato [0..255]
    frac_pico: float = 0.6        # píxeles >= frac_pico * pico forman la mancha
    medio_lado_mancha: int = 8    # ventana (2n+1)² alrededor del pico para el centroide [px]
    ventana_seguimiento: int = 128  # lado de la ventana de búsqueda en SEGUIMIENTO [px]
    area_min: int = 2             # área válida de la mancha [px]
    area_max: int = 400
    salto_max_px: float = 80.0    # salto máximo entre cuadros consecutivos válidos [px]


@dataclass
class DeteccionAvion:
    v_umbral: int = 170          # Umbral fijo de seguridad
    min_pix: int = 3             # Píxeles mínimos en el plano V
    roi_r: int = 20
    b_global: int = 4
    bloque_ventana: int = 2
    bloque_global: int = 8
    ventana_seguimiento: int = 80
    salto_max_px: float = 80.0


@dataclass
class DeteccionForma:
    #* Detector por FORMA (plano Y): silueta por contraste local + descriptores de Fourier.
    #* Una instancia por objetivo: el avión y el objeto circular (ver Parametros).
    formas: tuple = ("avion",)    # familias de plantillas aceptadas: 'avion' -> las aprendidas avion_* (formas.json)
    polaridad: str = "oscuro"     # "oscuro": más oscuro que su entorno; "claro": más claro; "ambos"
    #! "ambos" cuesta el doble y trae falsos: el fondo claro encerrado entre el ala y la cola de
    #! un avión oscuro es, para el detector, una figura clara más. Elegir la polaridad que
    #! corresponda a la escena de la demostración.
    contraste_min: int = 25       # diferencia mínima de gris con el entorno [0..255]
    frac_borde: float = 0.5       # el borde de la figura se toma donde el contraste cae a esta fracción de su máximo
    tam_max_px: int = 91          # el objetivo debe ser más ANGOSTO que esto (ancho de su parte más gruesa) [px]
    area_min: int = 150           # área de la figura [px]. Bajo ~150 px la silueta no alcanza para reconocerla
    area_max: int = 40000
    dist_max: float = 0.18        # distancia máxima al descriptor de la plantilla (0 = idéntica)
    alargamiento_max: float = 6.0  # largo/ancho máximo de la figura (descarta palos y bordes)
    abrir_frac: float = 0.10      # quita lo más delgado que esta fracción de √área (el palo que sostiene al avión); 0 = no
    max_candidatos: int = 8       # figuras que se comparan por cuadro (las más grandes)
    reducir_busqueda: int = 2     # la búsqueda en el cuadro completo se hace a 1/2 de resolución
    ventana_seguimiento: int = 224  # lado de la ventana en SEGUIMIENTO [px]; debe contener al objetivo completo
    salto_max_px: float = 80.0
    #* dist_max, con las fotos del avión real (18-11-2026, celular, llevadas a 640x480): la vista
    #* desde arriba da 0,04–0,08 y la de lado 0,11–0,18; en las fotos de fondo sin avión, la figura
    #* más parecida queda en 0,21. El margen es estrecho: 0,18 acepta todo lo medido y nada del fondo.
    #? Falta repetirlo con la cámara de la RPi (herramientas/grabar_cuadros.py y reproducir_forma.py).


@dataclass
class DeteccionLuz:
    #* Detector de FUENTE DE LUZ (flash del celular), traído de mi_proyecto_camara/luz_2.py:
    #* búsqueda por bloques en el plano Y + centroide ponderado alrededor del mejor bloque.
    umbral: int = 220             # pico mínimo (0..255) para considerar que hay objetivo (luz_2)
    min_pix: int = 3              # píxeles mínimos sobre el corte (descarta píxeles calientes)
    frac_corte: float = 0.7       # pesos: sólo lo que supera 0,7·pico (luz_2)
    bloque_global: int = 8        # bloque de la búsqueda en el cuadro completo [px]
    bloque_ventana: int = 4       # bloque dentro de la ventana de seguimiento [px]
    ventana_seguimiento: int = 96  # lado de la ventana (luz_2: radio 48 px)
    #? luz_2 usa un radio de 1,5 bloques (12 px global, 6 px en la ventana). Con el flash cerca,
    #? la mancha saturada pasa de 12 px y el centroide quedaba amarrado al bloque. Se usa un radio
    #? fijo mayor; ajustar si el flash se ve mucho más grande.
    radio_centroide: int = 16     # [px]
    area_min: int = 3
    area_max: int = 4000
    salto_max_px: float = 80.0


@dataclass
class DeteccionRoja:
    #* Detector por COLOR (plano V), traído de mi_proyecto_camara/laser_op.py.
    v_umbral: int = 150           # pico mínimo del plano V (128 = neutro, 255 = rojo puro)
    y_min: int = 60               # brillo mínimo del punto en el plano Y (0 = desactivar)
    frac_corte: float = 0.7       # pesos del centroide: sólo lo que supera 128 + 0,7·(pico − 128)
    bloque: int = 4               # bloque de la búsqueda global en el plano V (= 8 px reales)
    ventana_seguimiento: int = 96  # lado de la ventana en SEGUIMIENTO, en px reales
    area_min: int = 2             # área válida, en px reales
    area_max: int = 1600
    salto_max_px: float = 80.0


# ---------------------------------------------------------------------------
# EP — Estimación y predicción (RLS del equipo + gating + oclusiones)
# ---------------------------------------------------------------------------
@dataclass
class Prediccion:
    #* Decisión del 25 %: orden 3 (posición, velocidad, aceleración) y λ = 0,80
    #* (N_ef = 5 cuadros = 62,5 ms a 80 fps). Re-sintonizar con el ruido de centroide real;
    #* si resulta cercano a 1 px, subir a λ = 0,85.
    orden: int = 3
    lam: float = 0.80
    #* Horizonte: en lazo cerrado se calcula en cada cuadro como (t_envío − t_exposición)
    #* + retardo del enlace (ver control.py). Este valor fijo sólo se usa sin enlace.
    horizonte_s: float = 0.025
    k_gating: float = 6.0         # rechazo si |innovación| > k·σ
    sigma_min_px: float = 1.0     # piso de σ para el gating en el estimador en píxeles [px]
    sigma_min_deg: float = 0.1    # ídem para el estimador en ángulos [°] (≈ 1 px a 0,115 °/px)
    max_rechazos: int = 3         # tras N rechazos seguidos se reinicia (maniobra real)
    t_max_s: float = 0.5          # EP-4: tiempo máximo extrapolando antes de declarar no confiable
    tau_conf_s: float = 0.2       # EP-5: constante de decaimiento de la confianza
    #* Revisión del 17-11-2026 (docs/prediccion_rls.pdf, ajustes 4 a 6). Con los tres primeros
    #* en False / 0 y gracia_gating = 0, prediccion.py se comporta como antes de la revisión.
    reinicio_informado: bool = True   # tras los rechazos, el estimador nuevo parte con esas mediciones (sin PÉRDIDA)
    gating_conjunto: bool = True      # pan y tilt se aceptan o rechazan juntos; la confianza mira los dos ejes
    t_acel_max_s: float = 0.0         # > 0: pasado ese tiempo sin medición aceptada, se extrapola a velocidad constante
    #! APAGADO por defecto (0). Medido el 17-11 (orden 3, λ = 0,80, error RMS a 300 ms de oclusión):
    #!   objetivo sin aceleración, σ = 0,10°:  0,69° sin tope -> 0,46° con tope de 0,04 s
    #!   objetivo con 30 °/s²:                 0,69° -> 1,10°;   con 80 °/s²: 0,69° -> 2,73°
    #! Quitar la aceleración sólo conviene con objetivos de velocidad casi constante y detección
    #! ruidosa. Si se usa, no bajar de 0,04 s (el horizonte normal es de 20–30 ms).
    gracia_gating: int = 3            # muestras extra (además de `orden`) antes de armar el gating tras arranque o reinicio


# ---------------------------------------------------------------------------
# SE — Supervisor de estados (subconjunto para la prueba de concepto)
# ---------------------------------------------------------------------------
@dataclass
class Supervisor:
    n_adquisicion: int = 5        # cuadros válidos consecutivos para pasar a SEGUIMIENTO
    t_perdida_s: float = 0.3      # sin detección válida -> PÉRDIDA
    t_reintento_s: float = 2.0    # en PÉRDIDA sin reaparición -> BÚSQUEDA
    conf_min: float = 0.2         # confianza de EP-5 bajo este valor -> PÉRDIDA
    t_rearme_auto_s: float = 1.0  # MODO_SEGURO con motores cortados: enlace sano este tiempo -> rearme solo
    t_gracia_rearme_s: float = 0.3  # tras un rearme, plazo para que el ESP32 informe su estado nuevo


# ---------------------------------------------------------------------------
# CS-2 — Lazo externo (referencia visual para el lazo interno del ESP32)
# ---------------------------------------------------------------------------
@dataclass
class LazoExterno:
    #* La ley de posición (PI de pid_t_2) corre en el ESP32. La RPi entrega θ_ref absoluto
    #* = dirección del objetivo en el mundo, predicha al instante en que la consigna llega.
    usar_feedforward: bool = False   #? False para validar el PI puro; True mejora el seguimiento
    retardo_enlace_s: float = 0.001  # USB + UART + intérprete (25 %: 0,5 + 0,16 + ~0,3 ms)
    omega_ff_max: float = 180.0      # [°/s] = ω_max del firmware
    #todo Igualar a los límites de config.h del firmware cuando se midan los recorridos.
    lim_pan_deg: tuple = (-60.0, 60.0)
    lim_tilt_deg: tuple = (-30.0, 30.0)


# ---------------------------------------------------------------------------
# Actuador (sólo para el enlace de texto heredado de la PoC)
# ---------------------------------------------------------------------------
@dataclass
class Actuador:
    paso_completo_deg: float = 1.8
    micropasos: int = 16          # 1/16 -> 0,1125 °/micropaso
    v_max_pasos_s: float = 8000.0  # saturación de seguridad para la PoC


# ---------------------------------------------------------------------------
# CS-3 — Enlace RPi <-> ESP32
# ---------------------------------------------------------------------------
@dataclass
class Enlace:
    #* UART del header de la RPi 5 (GPIO14 = TXD, pin 8; GPIO15 = RXD, pin 10). Requiere
    #* "dtparam=uart0=on" en /boot/firmware/config.txt. Por USB sería /dev/ttyUSB0.
    #! NO usar /dev/serial0: en la RPi 5 apunta al conector de depuración (ttyAMA10), no a los
    #! pines 8 y 10. Ése es probablemente el motivo por el que luz_2.py no llegaba al ESP32.
    puerto: str = "/dev/ttyAMA0"
    baudios: int = 921600         #* firmware esp32_pan_tilt (la PoC de texto usaba 115 200)
    periodo_min_s: float = 0.0    # trama binaria: una consigna por cuadro (80 Hz)
    watchdog_s: float = 0.100     # sin telemetría -> enlace caído (MODO_SEGURO)


# ---------------------------------------------------------------------------
# Perfiles por objetivo para el tablero (pt/ajustes.py, --monitor)
# ---------------------------------------------------------------------------
#* Un perfil = detector + cámara + umbrales para un objetivo. Se aplica con un clic desde el
#* tablero, sin volver a ejecutar main.py. Las claves son las de los ajustes (main.py,
#* lista_de_ajustes); una clave que falte deja ese parámetro como esté.
#* Además se aplica SOLO (pt/ajustes.py, perfil_de): al elegir ese detector en el tablero y al
#* partir main.py con --fuente camara --detector <ése>. Se apaga con --sin-perfil.
#? Valores de PARTIDA, no medidos en la plataforma: salen de los scripts de mi_proyecto_camara
#? (laser_op.py, luz_op.py: exposición 2000 µs y ganancia 2,0) y de los valores por defecto de
#? este archivo. Ajustarlos en el laboratorio desde el tablero y usar "Guardar perfil": quedan
#? en perfiles.json, que tiene prioridad sobre esta tabla.
PERFILES = {
    "laser": {"detector": "rojo", "exposicion_us": 2000, "ganancia": 1.0},
    "luz": {"detector": "luz", "exposicion_us": 2000, "ganancia": 2.0},
    #* Avión por FORMA (18-11-2026). Perfil COMPLETO: no depende del objetivo que se usó antes.
    #*   Umbrales: los de Parametros.forma_avion (más abajo está de dónde sale cada uno).
    #*   Predictor: λ y piso del gating por defecto, para deshacer lo que deje otro perfil.
    #? Cámara (8000 µs, ganancia 4,0) y contraste mínimo (12): valores de PARTIDA, sin respaldo
    #? vigente. Salieron de una grabación del 18-11 que el equipo retiró del repositorio como
    #? dato anterior. Ajustarlos en el tablero con el avión real frente a
    #? la cámara (el avión debe verse claramente más oscuro que el fondo, sin saturar el fondo)
    #? y usar "Guardar perfil" con el nombre avion.
    "avion": {"detector": "avion", "exposicion_us": 8000, "ganancia": 4.0, "color_rojo": 2.0, "color_azul": 1.5,
              "avion.polaridad": "oscuro", "avion.contraste_min": 12, "avion.dist_max": 0.18,
              "avion.tam_max_px": 91, "avion.area_min": 150, "avion.alargamiento_max": 8.0,
              "avion.frac_borde": 0.5, "avion.abrir_frac": 0.10, "avion.salto_activo": True,
              "lam": 0.80, "sigma_min_deg": 0.1},
    #? Objeto no identificado: se busca una figura circular (un disco inclinado se ve como
    #? elipse y también se acepta). Exposición de partida, igual a la del avión.
    "ovni": {"detector": "ovni", "exposicion_us": 6000, "ganancia": 2.0},
    #* Avión por COLOR (restituido el 19-11-2026). Cámara de las corridas de lazo cerrado del equipo
    #* con el detector por color (--exposicion 6000 --ganancia 2.0). Ganancias de color, λ y piso
    #* del gating en sus valores por defecto, para deshacer lo que deje el perfil del avión por forma.
    "avion_rojo": {"detector": "avion_rojo", "exposicion_us": 6000, "ganancia": 2.0, "color_rojo": 2.0,
                   "color_azul": 1.5, "avion_rojo.v_umbral": 170, "avion_rojo.min_pix": 3,
                   "avion_rojo.salto_activo": True, "lam": 0.80, "sigma_min_deg": 0.1},
}


@dataclass
class Parametros:
    optica: Optica = field(default_factory=Optica)
    captura: Captura = field(default_factory=Captura)
    deteccion: Deteccion = field(default_factory=Deteccion)
    deteccion_roja: DeteccionRoja = field(default_factory=DeteccionRoja)
    deteccion_luz: DeteccionLuz = field(default_factory=DeteccionLuz)
    prediccion: Prediccion = field(default_factory=Prediccion)
    supervisor: Supervisor = field(default_factory=Supervisor)
    lazo: LazoExterno = field(default_factory=LazoExterno)
    deteccion_avion: DeteccionAvion = field(default_factory=DeteccionAvion)   # avión por color (avion_rojo)
    actuador: Actuador = field(default_factory=Actuador)
    enlace: Enlace = field(default_factory=Enlace)
    #* Detectores por forma (18-11-2026). El círculo es una figura simple: se le exige más
    #* parecido (0,06) porque muchas manchas suaves se le acercan.
    #? Valor de partida: falta medirlo con el objeto real.
    #? Polaridad "ambos" para el objeto no identificado: emite luz, pero no se sabe si se verá
    #? como disco claro o como cuerpo oscuro con luces. Fijarla cuando se conozca el objeto.
    #? contraste_min del avión = 12: valor de PARTIDA (ver la nota del perfil «avion», más arriba).
    #? Si aparecen sombras o manchas del fondo, subirlo; si el avión se ve tenue, mejorar la luz antes de bajarlo.
    #* alargamiento_max del avión = 8. De lado, el avión real mide 4,7 a 5,7 de largo/ancho (fotos
    #* del 18-11, sin la sombra): con el límite general de 6 quedaba al borde. Con 8 no aparece
    #* ninguna detección en las 3 fotos de fondo (igual que con 6).
    #* dist_max = 0,18 (el general): la vista de lado da 0,11–0,18 en las fotos; con 0,15 se pierde una de cinco.
    forma_avion: DeteccionForma = field(default_factory=lambda: DeteccionForma(contraste_min=12, alargamiento_max=8.0))
    forma_ovni: DeteccionForma = field(default_factory=lambda: DeteccionForma(
        formas=("circulo",), polaridad="ambos", dist_max=0.06, alargamiento_max=3.0, abrir_frac=0.0))