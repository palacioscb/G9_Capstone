"""
parametros.py — PI-7 (calibración y parámetros) y parámetros de todos los bloques.

Un solo lugar para todos los números con unidades. Los valores marcados
NOMINAL se reemplazan por valores medidos (benchmark de cámara, calibración).
"""
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Óptica — Waveshare Pi5-IMX219-77 (IMX219, 3280x2464, píxel 1,12 µm)
# ---------------------------------------------------------------------------
@dataclass
class Optica:
    pix_um: float = 1.12            # tamaño de píxel nativo [µm]
    f_mm: float = 2.85              # NOMINAL: focal del lente [mm] (reemplazar por calibración)
    sensor_px: tuple = (3280, 2464)  # arreglo activo [px nativos]


# ---------------------------------------------------------------------------
# PI-1 — Captura
# ---------------------------------------------------------------------------
@dataclass
class Captura:
    salida_px: tuple = (640, 480)        # tamaño de la matriz entregada (ancho, alto)
    modo_sensor_px: tuple = (1640, 1232)  # modo del sensor: binning 2x2, campo completo
    # recorte del sensor usado por ese modo [px nativos] (x, y, ancho, alto).
    # En la cámara real se lee de los metadatos (ScalerCrop); aquí es el valor esperado.
    recorte_sensor: tuple = (0, 0, 3280, 2464)
    fps: float = 60.0                    # PI-1. Medido (rpicam-hello --list-cameras, 29-09-2026): 1640x1232 llega a 81,07 fps
    exposicion_us: int = 0               # 0 = automática; >0 fija (láser: 1000–3000 µs)
    ganancia: float = 1.0                # sólo si exposición fija


# ---------------------------------------------------------------------------
# PI-3..PI-6 — Detector de láser
# ---------------------------------------------------------------------------
@dataclass
class Deteccion:
    umbral_abs: int = 180         # nivel mínimo de gris del pico para considerar candidato [0..255]
    frac_pico: float = 0.6        # píxeles >= frac_pico * pico forman la mancha
    medio_lado_mancha: int = 8    # ventana (2n+1)² alrededor del pico para el centroide [px]
    ventana_seguimiento: int = 128  # lado de la ventana de búsqueda en SEGUIMIENTO [px]
    area_min: int = 2             # área válida de la mancha [px]
    area_max: int = 400
    salto_max_px: float = 80.0    # salto máximo entre cuadros consecutivos válidos [px]


# ---------------------------------------------------------------------------
# EP — Estimación y predicción (RLS del equipo + gating + oclusiones)
# ---------------------------------------------------------------------------
@dataclass
class Prediccion:
    orden: int = 2                # 2 = posición + velocidad; 3 = + aceleración
    # Factor de olvido. La memoria efectiva es N_ef ≈ 1/(1-λ) MUESTRAS, así que depende de los fps.
    # Barrido sintético a 60 fps, h = 50 ms (herramientas/sintonia_prediccion.py --fps 60):
    #   λ=0,90 (N_ef 167 ms): 5,9 px RMS, PEOR que no predecir (5,6 px)
    #   λ=0,80 (N_ef  83 ms): 2,3 px RMS  <- valor por defecto
    # Re-sintonizar con el ruido de detección real medido.
    lam: float = 0.80
    horizonte_s: float = 0.050    # h = retardo total del lazo (reemplazar por el medido, ADV-2)
    k_gating: float = 6.0         # rechazo si |innovación| > k·σ
    sigma_min_px: float = 1.0     # piso de σ para el gating [px]
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
# Actuador (para convertir ω [°/s] a pasos/s del firmware PoC)
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
    puerto: str = "/dev/ttyUSB0"  # usar /dev/serial/by-id/... para que no cambie
    baudios: int = 115200         # firmware PoC actual; objetivo de diseño: 921 600
    periodo_min_s: float = 0.02   # no enviar consignas más rápido que esto (50 Hz)
    watchdog_s: float = 0.100     # sin respuesta -> enlace caído


@dataclass
class Parametros:
    optica: Optica = field(default_factory=Optica)
    captura: Captura = field(default_factory=Captura)
    deteccion: Deteccion = field(default_factory=Deteccion)
    prediccion: Prediccion = field(default_factory=Prediccion)
    supervisor: Supervisor = field(default_factory=Supervisor)
    actuador: Actuador = field(default_factory=Actuador)
    enlace: Enlace = field(default_factory=Enlace)
