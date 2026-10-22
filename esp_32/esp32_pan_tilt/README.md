# esp32_pan_tilt — lazo interno de la plataforma pan-tilt (IEE2913, Grupo 9)

Firmware del ESP32 que cierra el **lazo de posición de cada eje** con los AS5600, usando la ley de control de `pid_t_2.pdf`. La Raspberry Pi cierra el lazo de la cámara y le manda la referencia θ_ref. La prueba de concepto de un eje en lazo abierto queda intacta en `../esp32_drv8825`.

```
θ_ref (RPi o consola) ─►(+)─ e_θ ─► PI (pid_t_2) ─ ω_cmd ─► STEP/DIR ─► DRV8825 ─► NEMA 17
                        ▲(−)                                                       │
                        └──────────── θ_med ◄── AS5600 (I2C, 1 kHz) ◄──────────────┘
```

Compilado y enlazado contra Arduino-ESP32 2.0.17 (el núcleo de `platform = espressif32@^6`), sin advertencias con `-Wall -Wextra`. Revisado por un segundo revisor independiente, cuyos hallazgos están corregidos. Todavía no se probó en hardware.

## Archivos

| Archivo | Bloque | Qué hace |
|---|---|---|
| `include/config.h` | — | **Todos** los números: pines, signos, relación de transmisión, ceros, límites, ganancias, enlace |
| `src/generador_pasos.cpp` | CS-7 / CS-8 | Tren STEP/DIR de los dos ejes con un temporizador a 40 kHz (oscilador numérico) |
| `src/as5600.cpp` | Sensado | Lectura del AS5600, desenrollado de vueltas, cero y signo |
| `src/control_eje.cpp` | CS-10 | PI de pid_t_2 con saturación de ω, límite de α, anti-windup, tope de frenado del término P y banda integral |
| `src/protocolo.cpp` | CS-4 | Tramas binarias con CRC-16 (las mismas de `pt/enlace.py`) |
| `src/main.cpp` | CS-9 y resto | Estados, supervisión local, watchdog, telemetría, consola, captura de escalón |

## Pines: cableado real del laboratorio (22-10-2026)

Placa de expansión con S/V/G por GPIO. Todo está en `include/config.h`.

| Función | Pan (azimut) | Tilt (elevación) | Nota |
|---|---|---|---|
| nENABLE | GPIO13 (S) | GPIO27 (S) | sin pull-up externo: el driver queda energizado (sin pasos) mientras el ESP32 arranca |
| STEP | GPIO12 (S) | GPIO26 (S) | GPIO12 es pin de arranque: **no** ponerle pull-up |
| DIR | GPIO14 (S) | GPIO25 (S) | |
| AS5600 SDA / SCL | GPIO21 / GPIO22 (bus 0) | GPIO18 / GPIO19 (bus 1) | |
| AS5600 VCC | fila V del GPIO23 | fila V del GPIO5 | **debe ser 3,3 V**: medir V–G (puente de la placa) |
| Enlace con la RPi | RX0 (GPIO3) ← pin 8 RPi · TX0 (GPIO1) → pin 10 RPi · G → GND RPi | | también escucha en GPIO16 (RX2) / GPIO17 (TX2) |

- **Sin cablear todavía:** nFAULT, finales de carrera y M0/M1/M2. El firmware no los lee (`USAR_PIN_FAULT`, `USAR_FINES_CARRERA` y `USAR_PINES_MICROPASO` en `false`).
- **Micropasos:**
  - Los fija el hardware del driver. Con M0..M2 sueltos, el DRV8825 trabaja en paso completo (1,8°). Para 1/16, llevar M2 a 3V3.
  - El firmware parte en 1/1. Se cambia con `m <N>`, que queda guardado, y la prueba `t` los estima.
- **Enlace:**
  - El firmware escucha tramas en UART0 y UART2, y responde por el que recibió la última trama válida.
  - Con la RPi en RX0/TX0, no conectar el USB del ESP32 mientras corre el lazo, porque el chip USB-serie choca con el TX de la RPi.
  - Para cargar firmware, desconectar el cable de RX0, o mover los dos cables a GPIO16/17.
- **Ajustes guardados en el ESP32 (NVS):**
  - `m` (micropasos), `i <eje> motor|sensor` (signos) y `z` (cero) se guardan y sobreviven al reinicio.
  - `p` muestra los valores en uso; `b borrar` vuelve a los de `config.h`.

## Comentarios con color

Se usa la extensión de comentarios con color: `//!` rojo (seguridad y riesgos), `//*` verde (lo importante: qué hace y por qué), `//?` azul (supuestos y preguntas por verificar), `//todo` naranjo (pendientes). En Python, lo mismo con `#`.

## Puesta en marcha en el laboratorio (orden sugerido)

Consola a **921 600 baudios**: desde la RPi con `python3 -m serial.tools.miniterm /dev/ttyAMA0 921600`, o desde el monitor de PlatformIO si el ESP32 está por USB. `?` muestra la ayuda.

1. **Sin VMOT.** Cargar el firmware y revisar que ambos AS5600 digan `OK` y `status=0x20` (imán detectado). Con `p`, girar cada eje a mano: `med` debe cambiar suavemente.
2. **Topes.** Con `p`, presionar cada final de carrera: `fin=ACTIVO` en el eje correcto.
3. **Cero.** Centrar la plataforma a mano y escribir `z`. El cero queda guardado en el ESP32, así que el AS5600 funciona como *homing* absoluto. Copiar también los `ceroRaw` a `config.h`.
4. **Límites.** Medir el recorrido libre de cada eje y ajustar `limMinDeg` y `limMaxDeg` en `config.h` y en `pt/parametros.py` (`LazoExterno`).
5. **Con VMOT: micropasos y signos.** `e 1`, luego `t pan` y `t tilt`. Cada eje avanza al menos 3° (5,4° en 1/1) y vuelve.
   - Si dice "MICROPASOS: el driver parece estar en 1/N": `x`, `m N` y repetir.
   - Pan debe moverse a la **derecha** y tilt hacia **arriba**. Si no: `x`, `i pan motor` (o `i tilt motor`) y repetir.
   - Si dice "sensor con signo CONTRARIO": `x`, `i <eje> sensor` y repetir.
   - Todo queda guardado en el ESP32, sin recompilar.
6. **Primer lazo cerrado.** `r pan 5`, `r pan 0`, `r tilt 3`, `r tilt 0`. Cada eje debe ir y quedarse.
   - Si algo diverge, `x`: el firmware igual lo corta por pérdida de pasos si la diferencia pasa de 2°.
7. **Prueba del modelo de pid_t_2.** Desde la RPi:
   ```bash
   python3 herramientas/respuesta_escalon.py --puerto /dev/ttyAMA0 --eje pan --escalon 3 --habilitar
   python3 herramientas/respuesta_escalon.py --puerto /dev/ttyAMA0 --eje pan --escalon 2 --alfa 0   # modelo lineal
   ```
   La herramienta compara la medición con el modelo y estima ζ. Con eso queda b medido y el techo real Kp < b/M.
   - Para comparar con el modelo **lineal** del PDF, usar `--alfa 0` y escalones de 2 a 4°: así el tope de frenado y la banda integral no actúan, y la ley es idéntica a la de pid_t_2.
   - Recién entonces subir a las ganancias de pid_t_2 con `k pan 5 6.25` y `k tilt 20 100`.
8. **Lazo completo con la cámara** (desde la RPi):
   ```bash
   python3 main.py --fuente camara --enlace trama --duracion 30 --habilitar
   ```

## Consola de diagnóstico

| Comando | Qué hace |
|---|---|
| `e 1` / `e 0` | habilitar, y rearmar fallas si ya no están presentes / deshabilitar |
| `x` | parada inmediata y deshabilitar |
| `s` | retener la posición actual |
| `r <eje> <deg>` | referencia **absoluta** en lazo cerrado |
| `g <eje> <deg>` | **escalón** relativo y captura de 6 s (una muestra cada 2 ms) |
| `d` | descargar la captura en CSV (detiene los ejes mientras imprime) |
| `k <eje> <Kp> <Ki>` | ganancias en caliente [1/s, 1/s²] |
| `a <eje> <deg/s²>` | límite de aceleración del PI (0 = sin límite, sólo escalones chicos) |
| `v <eje> <deg/s>` | velocidad en lazo abierto (diagnóstico) |
| `t <eje>` | prueba de signos y estimación de micropasos |
| `m <N>` | micropasos del driver (1–32), guardado |
| `i <eje> motor\|sensor` | invertir un signo, guardado |
| `b borrar` | borrar los ajustes guardados |
| `z` | cero en la posición actual (con los drivers deshabilitados), guardado |
| `p` | estado |

Los ejes se escriben `0`/`pan`/`p` o `1`/`tilt`/`t`. Mientras el programa de la RPi está conectado (enlace binario activo), la consola del mismo UART se ignora a propósito. Si el enlace va por UART2, por el USB se aceptan `x`, `p`, `e` y `?`.

## Estados y supervisión local

`DESHABILITADO → (e 1) → RETENCION ⇄ LAZO`, más `ABIERTO` para diagnóstico y `FALLA`.

La falla se enclava. Se rearma con `e 1` o con `CMD_HABILITAR`, y sólo si la causa ya desapareció (rearme manual, D21). Causas y reacción, dentro del mismo milisegundo:

| Falla | Detección | Reacción |
|---|---|---|
| nFAULT del DRV8825 | pin en bajo | deshabilitar ambos drivers |
| Sensor | 5 lecturas I2C fallidas seguidas, o imán ausente en 3 revisiones seguidas (a 10 Hz) | deshabilitar |
| Límite | θ_med fuera de [lím − 5°, lím + 5°] | deshabilitar |
| Final de carrera | tope abierto durante 3 ms seguidos | deshabilitar; no se rearma mientras el tope siga activo |
| Pérdida de pasos (ACT-5) | \|θ_pasos − θ_med\| > 2° (no se evalúa los primeros 200 ms tras habilitar) | deshabilitar |
| Watchdog del enlace | 100 ms sin consigna de la RPi | RETENCIÓN (no se enclava); después de 5 s, DESHABILITADO |

## Protocolo con la RPi

`AA 55 | seq | tipo | largo | carga | CRC-16/CCITT-FALSE (LE)`, a 921 600 baudios.

- `0x02 CONSIGNA_THETA` (RPi → ESP32): θ_ref pan y tilt [°] y ω_ff pan y tilt [°/s], 4 × float32.
- `0x03 COMANDO`: 1 = habilitar, 2 = deshabilitar, 3 = retener, 4 = cero.
- `0x81 TELEMETRIA` (ESP32 → RPi, 200 Hz mientras llegan tramas): t_us, θ_med, θ_ref y ω_cmd por eje, estado, fallas y número de la última consigna.

Entre consignas (80 fps), el ESP32 extrapola la referencia con ω_ff durante a lo más 50 ms.

## Ley de control: lo que se agregó a pid_t_2

El PI es el del PDF (C(s) = Kp + Ki/s sobre e = θ_ref − θ_med, salida ω_cmd), con saturación de ω (180°/s), límite de α (300°/s²) y anti-windup por integración condicional. La revisión del código encontró que, con escalones grandes, el límite de α trunca el frenado: tilt se pasaba 27,8° en 60°. Por eso hay dos agregados que sólo actúan con errores grandes:

- **Tope de frenado del término P:** |P| ≤ √(2·α_max·|e|), la velocidad desde la que todavía se alcanza a frenar.
- **Banda integral:** el integrador sólo acumula con |e| < 5° (`BANDA_INTEGRAL_DEG`).

En simulación, el peor sobrepaso baja a 2,3° (escalones de 2 a 60°, ambos ejes, ambos juegos de ganancias). Con `a <eje> 0` y escalones menores que 5°, la ley es idéntica a la lineal.

## Diferencias con la prueba de concepto

- **Sin zona muerta.** El generador por oscilador numérico produce cualquier velocidad; en la PoC, bajo 20 pasos/s el tren se detenía.
- **La ISR no llama funciones del núcleo**, así que es segura en IRAM. La alarma del temporizador se programa una sola vez.
- **No hay "inversión pendiente".** El cambio de sentido lo resuelve la ISR con STEP en bajo. Con esto desaparece la falla de `pendingReverse`, que podía arrancar el motor en un sentido viejo.
- **Watchdog, límites y detección de pérdida de pasos**, que la PoC no tenía.
