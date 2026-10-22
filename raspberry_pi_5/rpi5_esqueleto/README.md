# Software de la Raspberry Pi 5 (IEE2913, Grupo 9) — lazo externo de seguimiento

Programa único que corre en la RPi. Hace lo siguiente:
- detecta el láser rojo;
- ubica el objetivo en el mundo con la pose real de la plataforma (AS5600, por telemetría);
- predice dónde estará el objetivo cuando el ESP32 aplique la consigna;
- le manda θ_ref (y ω_ff, opcional) al firmware `esp_32/esp32_pan_tilt`, que cierra el lazo de posición con el PI de `pid_t_2.pdf`.

El mismo código corre en el PC con una cámara simulada y un ESP32 simulado, en **lazo cerrado**.

## Estructura y trazabilidad a los diagramas de bajo nivel

| Archivo | Bloque | Qué hace |
|---|---|---|
| `pt/parametros.py` | PI-7 y todos | Todos los números con unidades. Ya actualizado a las decisiones del 25 % (80 fps, exposición 2 ms, RLS orden 3 con λ = 0,80) |
| `pt/camara.py` | PI-1 | `FuenteCamara`: planos Y y V, balance de blancos fijo. `FuenteSintetica`: láser rojo, lámpara blanca opcional y escena que responde a la pose simulada |
| `pt/deteccion.py` | PI-2 a PI-6 | `DetectorLaserRojo` (plano V con confirmación en Y, traído de `laser_op.py`) y `DetectorLaser` (brillo, el original) |
| `pt/geometria.py` | CS-1 | Píxel → ángulo, y **dirección del objetivo en el mundo** con la pose actual (composición exacta de rotaciones) |
| `pt/prediccion.py` | EP-1 a EP-5 | Gating, RLS del equipo (`pt/rls_predictor.py`, sin cambios), predicción y velocidad al instante de aplicación |
| `pt/control.py` | CS-2 | `ReferenciaVisual`: θ_ref y ω_ff a partir de la estimación; retención fuera de SEGUIMIENTO |
| `pt/enlace.py` | CS-3 | `EnlaceTrama` (ESP32 real, telemetría en un hilo), `EnlaceSimulado` (planta de pid_t_2 y el mismo PI) y `EnlaceNulo` |
| `pt/modelo_eje.py` | Modelo | Planta de pid_t_2, réplica exacta del PI del firmware, polos, Routh, métricas y ajuste de ζ |
| `pt/supervisor.py` | SE-2 / SE-3 | BÚSQUEDA ⇄ SEGUIMIENTO ⇄ PÉRDIDA; MODO_SEGURO por enlace caído o falla del ESP32; HOMING por AS5600 |
| `pt/registro.py` | MR-2 / CTL-7 | CSV por cuadro: pose, objetivo, consigna, telemetría y error de centrado |
| `pt/monitor_red.py` | MON-1 / MON-2 | Servidor web del tablero en un hilo aparte (sólo lectura): flujo en vivo y exportación CSV |
| `pt/monitor_web/index.html` | MON-1 | Tablero: imagen, detección, objeto, error, ángulos, referencia, ω_cmd, límites, fallas y modo |
| `main.py` | — | Une todo. `Ventana` (HDMI) y `MonitorRed` (red local) |
| `herramientas/respuesta_escalon.py` | CTL-7 | Escalón real contra el modelo de pid_t_2; estima ζ (es decir, b) |
| `pruebas_unitarias/` | — | 26 pruebas sin hardware |

Los comentarios usan colores: `#!` rojo (seguridad y riesgos), `#*` verde (lo importante), `#?` azul (supuestos por verificar), `#todo` naranjo (pendientes).

## Cómo se reparte el lazo

```
RPi (80 fps)                                                 ESP32 (1 kHz, pid_t_2)
cámara -> (u,w) -> objetivo en el mundo = f(u, w, θ_med(t_exp)) ─┐
                    -> RLS -> θ_ref = objetivo(t_aplicación) ─────┼──► e = θ_ref − θ_med -> PI -> ω_cmd -> STEP
                    -> ω_ff = velocidad del objetivo (opcional) ──┘                  ▲
                                         telemetría θ_med (200 Hz) ◄─────────────────┘
```

Se usa θ_med en el **instante de exposición** porque entre la foto y la llegada de la consigna (unos 18 ms) la plataforma ya se movió. Si se sumara Δθ a la posición del momento, ese movimiento se contaría dos veces.

## 1. En el PC

```bash
pip install numpy pyserial matplotlib
python pruebas_unitarias/test_esqueleto.py                 # 26 × OK
python main.py                                             # visión sola, cámara fija
python main.py --enlace simulado --lampara                 # lazo cerrado simulado, PI puro
python main.py --enlace simulado --lampara --feedforward   # ídem con ω_ff
python herramientas/respuesta_escalon.py --solo-modelo --eje pan --kp 5 --ki 6.25 --escalon 10
```

## 2. En la Raspberry Pi

Antes de medir: `vcgencmd get_throttled` debe dar `0x0`. Dependencias: `sudo apt install -y python3-serial python3-opencv python3-matplotlib`.

```bash
python3 pruebas_unitarias/test_esqueleto.py
python3 main.py --fuente camara --duracion 20 --ventana            # 1. láser real, sin motores: revisar detección
python3 main.py --fuente camara --enlace trama --puerto /dev/ttyUSB0 --duracion 20   # 2. con ESP32, motores deshabilitados
python3 herramientas/respuesta_escalon.py --puerto /dev/ttyUSB0 --eje pan --escalon 3 --habilitar   # 3. modelo
python3 main.py --fuente camara --enlace trama --puerto /dev/ttyUSB0 --duracion 30 --habilitar      # 4. LAZO CERRADO
```

- En el paso 1, si el láser no se detecta, revisar `pico` y `motivo` en el CSV. Si el pico de V queda bajo 170, subir `ganancias_color[0]` (rojo) o bajar `exposicion_us`.
- El paso 2 sirve para comprobar que la telemetría llega y que `pan_med`/`tilt_med` se mueven al girar los ejes a mano. **Sin `--habilitar` la RPi nunca mueve los motores:** manda "deshabilitar" en vez de consignas, aunque el ESP32 haya quedado energizado de una corrida anterior.
- La pose se lee en el instante de la **fila** donde está el objetivo (persiana rodante del IMX219; `t_lectura_ms` en `parametros.py` es una estimación por medir).
- El procedimiento de signos y cero del ESP32 está en `esp_32/esp32_pan_tilt/README.md`, y se hace **antes** del paso 3.

## 3. Monitoreo por la red, sin monitor HDMI (MON-1 / MON-2)

`--monitor` levanta un tablero web en la propia RPi, en hilos aparte del lazo. Se abre desde el navegador del PC o del celular, conectados a la misma red que la RPi.

```bash
# [RPi]
python3 main.py --fuente camara --enlace trama --puerto $PUERTO --duracion 0 --monitor
# [PC] navegador:  http://g9-cap.local:8000   (o http://<IP de la RPi>:8000)
```

- **Muestra (MON-1):**
  - estado de detección;
  - posición del objeto en la imagen y en el mundo;
  - error de centrado en pan y tilt;
  - θ_med por eje y referencia de centrado (centro de la imagen y θ_ref);
  - ω_cmd por actuador;
  - límites y finales de carrera;
  - las 8 fallas del ESP32, el enlace y el modo seguro;
  - modo de operación (supervisor y ESP32);
  - la imagen con el objeto y la consigna superpuestos.
- **Dinámica (MON-2):** cada refresco (20 por segundo) trae **todos** los cuadros nuevos, así que los gráficos tienen los 80 Hz completos. Tienen cursor sincronizado, ventana de 5 a 60 s y pausa.
- **Exportar (MON-2):**
  - `/historial.csv`: últimos 120 s, una fila por cuadro;
  - `/telemetria.csv`: θ_med, θ_ref y ω_cmd del ESP32 a 200 Hz;
  - `/registro.csv`: el CSV completo de la corrida.
- **Sólo lectura:** desde el navegador no se puede mover nada. Al terminar la corrida, el tablero queda abierto para exportar; se cierra con `Ctrl+C`.
- **Sin dependencias nuevas:** usa la biblioteca estándar, y OpenCV para la imagen. Opciones en `python3 main.py --help`, grupo *monitoreo por red*.
- **Prueba en el PC:** `python main.py --enlace simulado --lampara --monitor --duracion 30`, y abrir `http://localhost:8000`.

## Resultados en simulación (PC)

- **Detector rojo con una lámpara blanca en escena:** 160 de 160 cuadros, error de 0,41 px RMS (unos 0,05°, bajo el micropaso de 0,1125°). El detector por brillo se va a la lámpara en todos los cuadros (error de 254 px).
- **Modelo de pid_t_2:** reproduce los polos del PDF (pan −4,60 ± j141,8, −2,60, −2,41; tilt −58,3 ± j1365, −10,3, −9,75).
- **El PI del firmware compilado en C++ y su réplica en Python** dan la misma salida (|Δω| < 8·10⁻⁴ °/s en 3000 pasos, 1358 de ellos con límites activos).
- **Lazo cerrado simulado**, 10 s a 80 fps, objetivo de hasta 24,7°/s con dos oclusiones, ganancias de arranque y la ley final del firmware:

  | Configuración | Error de centrado RMS, todo SEGUIMIENTO | En régimen (0,6–3,2 s / 6,5–10 s) |
  |---|---|---|
  | PI puro, sin ω_ff | 31,6 px (3,6°) | — |
  | PI con ω_ff (`--feedforward`) | 11,1 px (1,3°) | 3,2 / 1,7 px (≈ 0,2–0,4°) |

  Con ω_ff, el error que queda está casi todo en la adquisición inicial y en la reaparición después de la oclusión de 0,5 s. Sin ω_ff, el PI sólo integra con |e| < 5°, así que se atrasa con un objetivo rápido.
- **Escalones (modelo con la ley final):** el peor sobrepaso es 2,3°, en escalones de 2 a 60°. Sin el tope de frenado y la banda integral llegaba a 42,7°.

## Modos reales del sensor (medido con `rpicam-hello --list-cameras`, 29-09-2026)

| Modo | Zona del sensor | fps máx | FOV H × V | °/px a 640 px |
|---|---|---|---|---|
| 640×480 | recorte 1280×960 | 200,16 | 28,2° × 21,4° | 0,045 |
| **1640×1232** (binning 2×2) | **completa** | **81,07** | **65,6° × 51,7°** | **0,115** |
| 1920×1080 | recorte 1920×1080 | 47,57 | 41,3° × 24,0° | 0,068 |
| 3280×2464 | completa | 21,19 | 65,6° × 51,7° | 0,115 |

`luz_op.py` y `laser_op.py` (en `mi_proyecto_camara`) no fijan el modo. Sus 120 fps y 5,9 ms son del modo recortado de 28°. Este programa fija 1640×1232.

## Pendiente

- Medir ζ (b) con `respuesta_escalon.py` y fijar las ganancias definitivas. El techo de estabilidad de pan depende de b: con ζ = 0,02 es Kp < 5,7.
- Calibrar f_px y las ganancias de color con el láser real. Medir el ruido de centroide y re-sintonizar λ.
- Patrón de búsqueda en BÚSQUEDA; finales de carrera; corrección de distorsión.
- La sección 5.1 del informe supone 30 fps; ahora son 80.
