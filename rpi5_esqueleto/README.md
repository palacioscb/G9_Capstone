# Esqueleto del software de la Raspberry Pi 5 (IEE2913, Grupo 9)

Prueba de concepto de los bloques que corren en la RPi:

- Procesamiento de imagen
- Estimación y predicción
- Control de seguimiento (lado RPi)
- Supervisor de estados

El flujo corre **completo en el PC**, con una cámara simulada, y en la RPi con la cámara real, sin cambiar el código.

## Estructura y trazabilidad a los diagramas de bajo nivel

| Archivo | Bloque | Qué hace hoy |
|---|---|---|
| `pt/parametros.py` | PI-7 y parámetros de todos los bloques | Todos los números con unidades en un solo lugar |
| `pt/camara.py` | PI-1 | `FuenteCamara` (Picamera2, plano Y) y `FuenteSintetica` (láser simulado con verdad conocida). Marca de tiempo = inicio de exposición |
| `pt/deteccion.py` | PI-2 a PI-6 | Detector de láser: ventana de 128 px en seguimiento, pico, centroide subpíxel, validación por área y salto |
| `pt/prediccion.py` | EP-1 a EP-5 | Gating, RLS del equipo (`pt/rls_predictor.py`, sin cambios), extrapolación a t+h, oclusiones, confianza |
| `pt/supervisor.py` | SE-2 / SE-3 | Estados BÚSQUEDA ↔ SEGUIMIENTO ↔ PÉRDIDA y MODO_SEGURO por enlace; rearme sólo manual |
| `pt/geometria.py` | CS-1 | Píxel → ángulo exacto (pan = atan x, tilt = −atan(y/√(1+x²))) y aproximación lineal por IFOV |
| `pt/control.py` | CS-2 (sólo interfaz) | `ControlNulo` por defecto; `ControlProporcionalDemo` sólo para probar el enlace. La ley real es de otro integrante |
| `pt/enlace.py` | CS-3 | `EnlaceCLI` para el firmware PoC (texto) y trama binaria con CRC-16 ya probada, lista para el firmware definitivo |
| `pt/registro.py` | MR-2 / CTL-7 (anticipo) | CSV por cuadro con marca de tiempo común |
| `main.py` | — | Une todo |
| `herramientas/` | — | `primera_captura.py`, `bench_camara.py`, `prueba_enlace.py`, `sintonia_prediccion.py` |
| `pruebas_unitarias/` | — | 12 pruebas sin hardware |

## 1. En el PC (hoy)

Requisitos: Python 3.10 o superior y NumPy. `pyserial` solo hace falta para el enlace.

```bash
pip install numpy pyserial
cd "pruebas/rpi5_esqueleto"
python pruebas_unitarias/test_esqueleto.py        # 12 × OK
python main.py                                    # 10 s de láser simulado
python herramientas/sintonia_prediccion.py        # barrido de λ y orden del RLS (~30 s)
```

El CSV de cada corrida queda en `registros/`.

## 2. Copiar a la Raspberry Pi

Desde PowerShell, parado en la carpeta `pruebas`:

```bash
scp -r rpi5_esqueleto g9-cap@g9-cap.local:~/
```

(O arrastra la carpeta al explorador de VS Code conectado por Remote-SSH.)

Dependencias en la RPi (una sola vez):

```bash
sudo apt install -y python3-serial python3-opencv
```

## 3. En la Raspberry Pi (universidad), en este orden

**Antes de medir nada**, verifica la alimentación con `vcgencmd get_throttled`. Debe dar `throttled=0x0`; si no, las mediciones no valen.

```bash
cd ~/rpi5_esqueleto
python3 pruebas_unitarias/test_esqueleto.py                     # 1. el código funciona en la RPi
python3 main.py                                                 # 2. flujo completo con fuente sintética
rpicam-hello --list-cameras                                     # 3. modos reales del sensor -> informe
python3 herramientas/primera_captura.py                         # 4. la imagen como matriz
python3 herramientas/bench_camara.py --salida bench_rgb         # 5. tabla modos/fps/latencia/FOV
python3 herramientas/bench_camara.py --formato YUV420 --salida bench_yuv
python3 main.py --fuente camara --duracion 20 --exposicion 2000 # 6. láser real, sin motores
```

Para el paso 6: apunta un puntero láser a una pared dentro del campo de visión y muévelo lento. El resumen muestra:

- tasa de detección;
- tiempo de detección;
- latencia desde la exposición hasta la consigna;
- transiciones del supervisor.

Con el ESP32 y el firmware PoC conectados por USB:

```bash
ls /dev/serial/by-id/
python3 herramientas/prueba_enlace.py --puerto /dev/ttyUSB0 --baudios 115200 -n 200
```

**Mover el motor con el láser** (`--control demo --habilitar`) solo con el banco armado, la fuente del motor encendida y alguien a cargo del corte.

## Resultados ya obtenidos en simulación (PC)

- **Centroide subpíxel:** error < 0,1 px sobre un punto gaussiano. Con la fuente sintética completa, RMS 0,05 px.
- **Detección:** ~0,15 ms por cuadro en el PC (a confirmar en la RPi).
- **Píxel → ángulo** (modo 1640×1232 escalado a 640×480, f nominal 2,85 mm):

  | u [px] | exacto [°] | lineal [°] | error lineal [°] |
  |---|---|---|---|
  | 400 | 9,21 | 9,29 | 0,08 |
  | 480 | 17,91 | 18,52 | 0,61 |
  | 560 | 25,84 | 27,75 | 1,91 |
  | 639 (borde) | 32,76 | 36,87 | 4,11 |

  **Conclusión: usar la fórmula exacta (atan), no Δθ = e·IFOV.**
- **Predictor RLS** a 60 fps con h = 50 ms: con λ = 0,90 (valor del informe del 5 %), el error en t+h es **peor que no predecir** (5,9 px contra 5,6 px). Con λ = 0,80 baja a 2,3 px, y ese queda por defecto. La memoria efectiva está en *muestras*: si cambia la tasa de cuadros, λ debe re-ajustarse. Re-sintonizar con el ruido de detección real.

## Modos reales del sensor (medido con `rpicam-hello --list-cameras`, 29-09-2026)

| Modo | Zona del sensor | fps máx | FOV H × V | °/px nativo | °/px a 640 px |
|---|---|---|---|---|---|
| 640×480 | recorte 1280×960 | 200,16 | 28,2° × 21,4° | 0,045 | 0,045 |
| **1640×1232** (binning 2×2) | **completa** | **81,07** | **65,6° × 51,7°** | 0,045 | **0,115** |
| 1920×1080 | recorte 1920×1080 | 47,57 | 41,3° × 24,0° | 0,0225 | 0,068 |
| 3280×2464 | completa | 21,19 | 65,6° × 51,7° | 0,0225 | 0,115 |

FOV y °/px calculados con f = 2,85 mm (nominal) y píxel de 1,12 µm. Los fps son iguales en 10 y 8 bits.
Configuración de trabajo: **modo 1640×1232 escalado a 640×480 a 60 fps**: campo completo y margen de 81 fps.

## Pendiente

- Confirmar con `bench_camara.py` que 60 fps se sostienen en Python (fps entregados, latencia).
- Calibrar f_px (tablero de ajedrez o calibración con motores, ADV-1) y reemplazar el valor nominal.
- Detectores de aeronave y de patrón desconocido; corrección de distorsión al punto.
- Firmware del ESP32 con la trama binaria y 921 600 baudios; entonces se agrega `EnlaceTrama`.
- HOMING, AS5600, marca de tiempo común con la telemetría del ESP32.
