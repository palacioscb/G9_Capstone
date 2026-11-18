# Generador de pasos del ESP32: ¿conviene reemplazar el oscilador numérico?

Revisión del 17-11-2026 de `esp_32/esp32_pan_tilt/src/generador_pasos.cpp`, a raíz del comentario de los ayudantes: el oscilador sería "matar una hormiga con un cañón" y podría generar retardos.

## Conclusión

**No conviene reemplazarlo ni modificarlo.**

- **No es la fuente de retardo.** Su error de tiempo máximo es un tic, 25 µs. A la velocidad máxima (180 °/s) eso equivale a 0,0045°, 25 veces menos que un micropaso de 1/16.
- **No es un cañón.** La rutina de interrupción tiene unas 20 líneas. Las alternativas más "simples" tienen más retardo a baja velocidad, o pierden el conteo de pasos que usa la detección de pérdida de pasos.
- **El retardo que se ve en el seguimiento está en otra parte:** en la ganancia del PI, en el límite de aceleración y en la falta de feedforward (ver `docs/prediccion_rls.pdf`, sección 5).
- **Único ajuste razonable, opcional:** bajar `F_TICK_HZ` de 40 000 a 20 000 si alguna vez se mide que falta CPU. Hoy no hay evidencia de eso.

Nada de esto está medido en el ESP32: son cálculos sobre el código y una simulación. La sección "Cómo comprobarlo" dice qué medir para cerrar el punto con los ayudantes.

## Cómo funciona

El nombre correcto es **oscilador controlado numéricamente** (NCO, o acumulador de fase; en control numérico se le dice DDA). Es la misma idea de un DDS, sin la tabla de seno.

1. Un temporizador de hardware interrumpe 40 000 veces por segundo (cada 25 µs). Se programa una vez y no se vuelve a tocar.
2. Cada eje tiene un contador de 32 bits, la **fase**. En cada tic se le suma un **incremento** proporcional a la velocidad pedida: `incremento = |v| · 2³² / 40 000`, con `v` en micropasos por segundo.
3. Cuando la fase desborda (da la vuelta), corresponde un micropaso: se sube STEP durante un tic y se suma ±1 a `posicion`.
4. El lazo de control (1 kHz) sólo escribe `incremento` y el sentido. No hay secciones críticas: una escritura de 32 bits es atómica.

Lo que la rutina cuida, y está bien resuelto:

- **Ancho de pulso:** STEP queda en alto exactamente un tic (25 µs; el DRV8825 pide 1,9 µs).
- **Cambio de sentido:** primero escribe DIR y emite el paso en el tic siguiente (25 µs de anticipación; el DRV8825 pide 0,65 µs).
- **Sin velocidad mínima:** la resolución es 40 000 / 2³² ≈ 0,00001 pasos/s. La prueba de concepto se detenía bajo 20 pasos/s.
- **Tope:** `PASOS_S_MAX` = 10 000 pasos/s. Lo que se necesita a 180 °/s es 1 600 pasos/s en 1/16 y 3 200 en 1/32.
  Con paso completo (el valor de arranque del firmware si no se configuró `m 16`) son apenas 100 pasos/s.

## ¿Genera retardo?

Presupuesto de tiempos entre la foto y el movimiento, con los valores actuales del código:

| Etapa | Tiempo | Origen del dato |
|---|---|---|
| Exposición → consigna lista en la RPi | 12 a 19 ms | simulación (12,2 ms) y prueba de concepto (18,6 ms) |
| Trama por UART (23 bytes a 921 600 baudios) | 0,25 ms | cálculo |
| Espera al siguiente período del lazo PI | hasta 1 ms | `F_CONTROL_HZ` = 1 000 |
| **Generador de pasos** | **hasta 0,025 ms** | un tic de `F_TICK_HZ` |
| Constante de tiempo del lazo de posición, 1/Kp | 333 ms (pan), 100 ms (tilt) | ganancias de arranque de `config.h` |
| Rampa de aceleración hasta 30 °/s | 100 ms | `ALFA_MAX_DPS2` = 300 °/s² |

El generador aporta menos de una diezmilésima del tiempo de respuesta del lazo. Además, un cambio de velocidad rige desde el tic siguiente, porque la fase es continua: no hay que esperar a que termine el período en curso.

Lo que sí hay es **cuantización**: a 1 °/s en 1/16 sale un micropaso cada 112 ms. Eso no es retardo del generador; es el tamaño del paso, y lo tendría cualquier otro método.

## Costo real: CPU

El costo del método es que interrumpe 40 000 veces por segundo aunque los motores estén quietos. Estimación, sin medir: entre 2 y 4 µs por interrupción, casi todo en la entrada y salida de la interrupción, es decir entre 8 y 16 % de un núcleo. El lazo de 1 kHz ocupa el resto con holgura, y el otro núcleo del ESP32 queda libre.

Si se quisiera reducir, basta cambiar una constante: con `F_TICK_HZ` = 20 000 el costo baja a la mitad, el error de tiempo sube a 50 µs (0,009° a 180 °/s) y el tope queda en 5 000 pasos/s, todavía sobre los 3 200 de 1/32.

## Alternativas

| Método | Retardo ante un cambio de velocidad | Velocidad mínima | Cuenta los pasos | CPU | Observación |
|---|---|---|---|---|---|
| **NCO actual** | ≤ 25 µs | no tiene | sí, exacto | fija, 8–16 % (estimada) | un temporizador para los dos ejes |
| Temporizador por paso (lo que hacía la PoC) | hasta un período entero entre pasos, salvo que se reprograme la alarma desde el lazo | sí, o hay que tratar aparte el caso v ≈ 0 | sí | proporcional a la velocidad | reprogramar la alarma desde la interrupción fue el riesgo que se documentó en la PoC |
| PWM por hardware (LEDC o MCPWM) | depende de cómo cambie la frecuencia; habría que verificarlo con osciloscopio | la del periférico | no: habría que agregar un contador de pulsos (PCNT) | casi nula | invertir el sentido y detenerse exigen coordinación aparte |
| Librería (AccelStepper, FastAccelStepper) | el de su propio generador de rampas | depende | sí | variable | están pensadas para movimientos a una posición con rampa propia, que se sumaría a la del PI |

El conteo exacto de pasos no es un lujo: `main.cpp` lo compara con el AS5600 para detectar pérdida de pasos (ACT-5).

## Lo que se probó y se descartó

Se evaluó una variante "más exacta": acumulador **con signo**, donde invertir el sentido descuenta la fracción de paso acumulada. Se simuló la retención de una referencia fija con el PI del firmware, el AS5600 cuantizado a 12 bits con ruido, y el generador tic a tic (`herramientas/sim_generador_pasos.py`, micropasos 1/16, planta ideal).

| Eje (Kp; Ki) | Ruido del sensor | Firmware actual | Con signo |
|---|---|---|---|
| pan (3; 2,25) | 0,03° | 5,9 pasos/s | 278 pasos/s |
| pan (3; 2,25) | 0,06° | 6,3 pasos/s | 194 pasos/s |
| tilt (10; 25) | 0,03° | 7,4 pasos/s | 377 pasos/s |
| tilt (10; 25) | 0,06° | 8,5 pasos/s | 171 pasos/s |

La variante con signo emite entre 20 y 50 veces más pasos en reposo: queda justo sobre el borde de un paso y cada cambio de signo de ω_cmd lo cruza. El acumulador del firmware tiene, de hecho, una histéresis de un paso completo, y por eso es más tranquilo. **Es un argumento más para dejarlo como está.**

Dato lateral de esa simulación: en reposo el lazo igual emite unos 6 a 8 micropasos por segundo, porque el paso (0,1125°) y la resolución del sensor (0,088°) no calzan y el integrador busca el punto intermedio. Si en la plataforma se oye un tic-tic en reposo, la solución está en el control (una banda muerta de medio paso en el error), no en el generador.

## Cómo comprobarlo

Dos mediciones cortas bastan para responder con datos:

1. **Osciloscopio en STEP** (es la prueba A del protocolo de banco): a 10, 50 y 180 °/s, medir el período entre pulsos y su variación. Debe ser el período teórico ± 25 µs.
2. **Atrasos del lazo de 1 kHz:** contar cuántas veces el período real supera 1,5 ms y mostrarlo en el comando `p`. Son unas 6 líneas en `main.cpp`. Si el contador queda en cero con los dos ejes moviéndose, la interrupción de 40 kHz no le está quitando tiempo al control.

## Respuesta corta para los ayudantes

- El generador es un acumulador de fase de 32 bits por eje, atendido por una interrupción fija de 40 kHz. Su retardo máximo es 25 µs.
- El lazo de posición responde en 100 a 333 ms. El generador no es el cuello de botella.
- Se eligió frente al temporizador por paso de la prueba de concepto porque no tiene velocidad mínima, sirve a los dos ejes con un temporizador y nunca reprograma el temporizador desde la interrupción.
- Su costo es CPU (8 a 16 % de un núcleo, estimado), y se puede bajar a la mitad con una constante.
