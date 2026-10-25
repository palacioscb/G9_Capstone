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
    #* Parámetros del detector de avión rojo basado en plano V
    v_umbral: int = 155
    min_pix: int = 12
    roi_r: int = 40
    b_global: int = 4



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
    v_umbral: int = 170           # pico mínimo del plano V (128 = neutro, 255 = rojo puro)
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


# ---------------------------------------------------------------------------
# SE — Supervisor de estados (subconjunto para la prueba de concepto)
# ---------------------------------------------------------------------------
@dataclass
class Supervisor:
    n_adquisicion: int = 5        # cuadros válidos consecutivos para pasar a SEGUIMIENTO
    t_perdida_s: float = 0.3      # sin detección válida -> PÉRDIDA
    t_reintento_s: float = 2.0    # en PÉRDIDA sin reaparición -> BÚSQUEDA
    conf_min: float = 0.2         # confianza de EP-5 bajo este valor -> PÉRDIDA


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
    actuador: Actuador = field(default_factory=Actuador)
    enlace: Enlace = field(default_factory=Enlace)
    deteccion_avion: DeteccionAvion = field(default_factory=DeteccionAvion)