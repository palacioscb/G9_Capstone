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

## Pines (ESP32 DevKit WROOM)

| Función | Pan (azimut) | Tilt (elevación) | Nota |
|---|---|---|---|
| STEP | GPIO25 | GPIO17 | |
| DIR | GPIO26 | GPIO16 | |
| nENABLE | GPIO27 | GPIO4 | **pull-up externo 4,7 kΩ a 3V3 obligatorio** |
| nFAULT | GPIO34 | GPIO35 | sólo entrada: **pull-up externo 10 kΩ a 3V3** |
| Final de carrera | GPIO13 | GPIO23 | los 2 topes del eje, **NC en serie** a GND; pull-up interno |
| AS5600 SDA | GPIO21 (bus 0) | GPIO18 (bus 1) | un bus I2C por sensor: no hace falta multiplexor |
| AS5600 SCL | GPIO22 (bus 0) | GPIO19 (bus 1) | |
| M0 / M1 / M2 | GPIO32 / 33 / 14 | (los mismos) | compartidos: ambos drivers en 1/16 |

- El lado izquierdo de la placa queda con el eje pan y los micropasos; es el mismo cableado de la prueba de concepto. El lado derecho queda con los sensores y el eje tilt.
- Libres y reservados: GPIO36 y GPIO39 (ADC de los shunt).
- **Topes:** con contactos NC en serie, el pin lee LOW en reposo. Si un tope se abre o se corta un cable, lee HIGH y el eje se detiene (a prueba de fallas). Si los topes son NO, usar `FIN_CONTACTO_NC = false`.
- El corte de nENABLE por hardware desde los topes (EA-5) queda para la PCB.
- Diagrama completo y tabla pin a pin: `docs/conexiones_laboratorio.pdf`.
- En cada DRV8825, SLEEP y RESET van a 3V3, igual que en la prueba de concepto.
- Los AS5600 se alimentan con 3V3. Si el bus falla con cables largos, agregar 4,7 kΩ de pull-up en SDA y SCL.
- GPIO4 tiene pull-down interno durante el reinicio. Sin el pull-up externo, el driver de tilt queda habilitado mientras el ESP32 arranca.

## Comentarios con color

Se usa la extensión de comentarios con color: `//!` rojo (seguridad y riesgos), `//*` verde (lo importante: qué hace y por qué), `//?` azul (supuestos y preguntas por verificar), `//todo` naranjo (pendientes). En Python, lo mismo con `#`.

## Puesta en marcha en el laboratorio (orden sugerido)

Monitor de PlatformIO a **921 600 baudios** (ya está en `platformio.ini`). `?` muestra la ayuda.

1. **Sin VMOT.** Cargar el firmware y revisar que ambos AS5600 digan `OK` y `status=0x20` (imán detectado). Con `p`, girar cada eje a mano: `med` debe cambiar suavemente.
2. **Topes.** Con `p`, presionar cada final de carrera: `fin=ACTIVO` en el eje correcto.
3. **Cero.** Centrar la plataforma a mano y escribir `z`. Copiar los `ceroRaw` que imprime a `config.h` para que el cero sea permanente: con eso el AS5600 funciona como *homing* absoluto.
4. **Límites.** Medir el recorrido libre de cada eje y ajustar `limMinDeg` y `limMaxDeg` en `config.h` y en `pt/parametros.py` (`LazoExterno`).
5. **Con VMOT: signos.** `e 1`, luego `t pan` y `t tilt`. Cada eje avanza +3° y vuelve.
   - Pan debe moverse a la **derecha** y tilt hacia **arriba**. Si no, invertir `signoMotor`.
   - Después, si el mensaje dice "sensor con signo CONTRARIO", invertir `signoSensor`.
   - Recompilar y repetir la prueba hasta que ambos queden coherentes.
6. **Primer lazo cerrado.** `r pan 5`, `r pan 0`, `r tilt 3`, `r tilt 0`. Cada eje debe ir y quedarse.
   - Si algo diverge, `x`: el firmware igual lo corta por pérdida de pasos si la diferencia pasa de 2°.
7. **Prueba del modelo de pid_t_2.** Desde la RPi:
   ```bash
   python3 herramientas/respuesta_escalon.py --puerto /dev/ttyUSB0 --eje pan --escalon 3 --habilitar
   python3 herramientas/respuesta_escalon.py --puerto /dev/ttyUSB0 --eje pan --escalon 2 --alfa 0   # modelo lineal
   ```
   La herramienta compara la medición con el modelo y estima ζ. Con eso queda b medido y el techo real Kp < b/M.
   - Para comparar con el modelo **lineal** del PDF, usar `--alfa 0` y escalones de 2 a 4°: así el tope de frenado y la banda integral no actúan, y la ley es idéntica a la de pid_t_2.
   - Recién entonces subir a las ganancias de pid_t_2 con `k pan 5 6.25` y `k tilt 20 100`.
8. **Lazo completo con la cámara** (desde la RPi):
   ```bash
   python3 main.py --fuente camara --enlace trama --puerto /dev/ttyUSB0 --duracion 30 --habilitar
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
| `t <eje>` | prueba de signos |
| `z` | cero en la posición actual (con los drivers deshabilitados) |
| `p` | estado |

Los ejes se escriben `0`/`pan`/`p` o `1`/`tilt`/`t`. Mientras el programa de la RPi está conectado (enlace binario activo), la consola se ignora a propósito.

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
