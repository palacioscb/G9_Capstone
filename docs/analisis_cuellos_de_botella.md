# Cuellos de botella de la plataforma pan-tilt: rendimiento y latencia

Revisión crítica del 17-11-2026 de toda la cadena: cámara, software de la RPi, enlace, firmware del ESP32 y lazo de control.

Cada hallazgo indica de dónde sale el dato:

- **medido en la RPi**: benchmark de cámara del 29-09-2026;
- **medido en el PC**: mediciones de esta revisión sobre `main.py` simulado;
- **simulado**: modelo de planta de `pid_t_2` con ζ = 0,05 supuesto;
- **estimado**: cálculo sobre el código, sin medición.

Falta lo más importante: datos de una corrida real. Para eso quedó `herramientas/analizar_registro.py` (sección 4).

## 1. Conclusión

**El cuello de botella es el lazo de posición del ESP32, no el cómputo ni el generador de pasos.** El software de la RPi usa cerca del 3 % del período de cuadro. La plataforma, en cambio, sigue al objetivo con un atraso de cientos de milisegundos cuando no hay feedforward.

| # | Cuello de botella | Efecto | Origen del dato | Qué hacer |
|---|---|---|---|---|
| 1 | Lazo de posición sin feedforward | atraso de 244 ms en pan; a 20 °/s el eje queda 5,4° atrás | simulado | activar el feedforward (`docs/feedforward_velocidad.pdf`) |
| 2 | Ganancias de arranque y banda integral | Kp = 3 en pan; el integrador se congela con error > 5° | código y simulado | medir ζ con `respuesta_escalon.py` y subir Kp |
| 3 | Límite de aceleración de 300 °/s² | 67 ms para llegar a 20 °/s; cerca de 1° de error en cada cambio de velocidad | código y simulado | medir el margen de torque y subirlo si lo hay |
| 4 | Latencia de la cámara | 15,5 ms de lectura, el 86 % del retardo total | medido en la RPi | bajar la exposición; modo recortado como opción |
| 5 | Resolución mecánica | con paso completo, un paso son 1,8° (16 px) | código | confirmar los micropasos con `p` y `t` |
| 6 | Reinicios del predictor | un cuadro en PÉRDIDA y corte de ω_ff en cada salto de la detección | simulado | reinicio informado (propuesto) |
| 7 | Sin patrón de búsqueda | si el objetivo sale del cuadro, la plataforma se queda quieta | código | implementar el barrido de BÚSQUEDA |
| 8 | Sincronía cámara–AS5600 | 0,03° por cada ms de error a 30 °/s | estimado | medir `t_lectura_ms` |
| 9 | Pausas del intérprete de Python | hasta +6 ms en el p99 con descargas del tablero | medido en el PC | corregido en `main.py` |

Los puntos 1 a 3 son el mismo problema visto desde tres lados: la plataforma responde lento. El 1 se resuelve hoy desde el tablero; el 2 y el 3 necesitan una medición en el banco.

## 2. Detalle por etapa

### Cámara (PI-1)

- **Latencia de lectura: 15,5 ms**, medida. Es el período del modo del sensor más unos 3 ms, y no depende del tamaño de salida. De los 18 ms entre la foto y la consigna, la cámara pone 15,5.
- **Exposición:** con `--exposicion 6000` el centro de la exposición queda 3 ms antes del final (con 2000 µs era 1 ms), y a 30 °/s el objetivo se desenfoca 0,18°, es decir 1,6 px.
  - Probar en vivo desde el tablero: bajar la exposición a 3000 µs y subir la ganancia a 4,0. La imagen queda igual de clara, con la mitad del desenfoque y 1,5 ms menos de latencia. El costo es más ruido.
- **Modo recortado 640×480:** 5,5 ms de latencia y 200 cuadros/s, medido. Bajaría el retardo total de 18 a unos 8 ms.
  - Costo: el campo visual cae de 65,6° a 28,2°, así que el objetivo se sale del cuadro con más facilidad.
  - Sirve como modo de precisión para el láser. Exige volver a sintonizar λ, porque la memoria del predictor se mide en cuadros.
- **Persiana rodante:** `t_lectura_ms` = 12 es una estimación. Ver sincronía, más abajo.

### Software de la RPi

Costo por cuadro, medido en el PC con `main.py` simulado (mediana y percentil 99):

| Etapa | Mediana | p99 |
|---|---|---|
| Detección (rojo, luz o avión) | 0,17 ms | 0,45 ms |
| Geometría y predictor (dos ejes) | 0,12 ms | 0,27 ms |
| Registro CSV | 0,04 ms | 0,12 ms |
| Tablero (`publicar`) | 0,01 ms | 0,09 ms |
| Pose, supervisor y consigna | 0,01 ms | 0,05 ms |
| **Total** | **0,35 ms** | |

- El período de cuadro es 12,5 ms: el cómputo ocupa cerca del 3 %. En la RPi 5 será mayor, pero el benchmark del 29-09 ya mostró 0 cuadros perdidos a 81 cuadros/s.
- **No vale la pena optimizar el código de detección ni el del predictor.**
- **Cambios de hilo de Python (corregido).** El lazo comparte el intérprete con el lector de telemetría y con el servidor del tablero. Con descargas de CSV continuas desde el tablero, el p99 de la latencia total subía de 13,0 a 18,8 ms. Con `sys.setswitchinterval(0.0005)`, ya agregado a `main.py`, queda en 13,8 ms.
  - Con el tablero abierto sin descargar nada el efecto es menor: p99 de 13,5 ms.
- **Recolector de basura:** una pausa de 7 a 10 ms cada 40 s, aproximadamente, medida en el PC. En la RPi será más larga. No se corrigió: es un cuadro cada 3 000.

### Enlace RPi–ESP32

- **No es cuello de botella.** La trama de consigna (23 bytes a 921 600 baudios) tarda 0,25 ms.
- La telemetría llega a 200 Hz y la pose se interpola al instante de la foto. Con la plataforma acelerando a 300 °/s², interpolar entre muestras separadas 5 ms deja un error de 0,001°.

### Firmware del ESP32

- **Generador de pasos:** hasta 25 µs. Ver `docs/analisis_generador_pasos.md`.
- **Lectura de los dos AS5600:** en cada milisegundo se hacen dos transacciones I2C a 400 kHz. Estimado: entre 0,25 y 0,6 ms de cada período de 1 ms. Sumado a la interrupción de 40 kHz, el lazo de 1 kHz podría estar justo.
  - No hay evidencia de atrasos, pero tampoco medición. Vale la pena el contador de atrasos propuesto en el análisis del generador.
  - Si se midieran atrasos: I2C a 1 MHz (el AS5600 lo admite, según `config.h`).
- **Ganancias del PI:** Kp de 3 s⁻¹ en pan y 10 s⁻¹ en tilt son valores de arranque prudentes. El techo de estabilidad depende de la fricción, que sigue sin medirse.
- **Banda integral de 5°:** evita sobrepasos en movimientos largos, pero congela el integrador cuando el error supera 5°. En pan eso ocurre con objetivos sobre unos 15 °/s. Con feedforward el error ya no llega a ese valor.

### Mecánica y sensores

- **Micropasos:** el firmware parte en paso completo si nunca se configuró `m 16`. En ese caso un paso son 1,8°, 16 px de la imagen, y ningún ajuste de software lo compensa. Con 1/16 son 0,1125°, cerca de 1 px. El comando `p` muestra el valor vigente.
- **AS5600:** 0,088° por cuenta. Junto con el micropaso fija el piso de precisión en torno a 0,1°.
- **Cacería en reposo:** en simulación el lazo emite de 6 a 8 micropasos por segundo en torno a la referencia, porque el paso y la cuenta del sensor no calzan. Si molesta, se resuelve con una banda muerta de medio paso en el error.

### Predictor y supervisor

- **Reinicios:** descritos en `claude/revision-17-11-tablero-y-predictor.md`, con seis variantes simuladas.
- **Sin patrón de búsqueda:** en BÚSQUEDA la plataforma retiene. Es la limitación funcional más visible en una demostración, sobre todo si se usara el modo recortado.
- **Sincronía:** la dirección del objetivo se calcula con la pose del AS5600 en el instante de la foto. Un error de tiempo δ convierte movimiento propio en movimiento aparente del objetivo: 0,03° por milisegundo a 30 °/s. Se mide con un objetivo fijo y la plataforma en movimiento (`docs/prediccion_rls.pdf`, sección 6).

## 3. Lo que no es cuello de botella

| Sospechoso | Por qué no | Origen del dato |
|---|---|---|
| Generador de pasos (NCO) | 25 µs frente a un lazo de 100 a 333 ms | estimado |
| Procesamiento en Python | 0,35 ms de 12,5 ms | medido en el PC |
| Registro CSV por cuadro | 0,04 ms | medido en el PC |
| UART a 921 600 baudios | 0,25 ms por consigna | calculado |
| Resolución de 640×480 | 0,115 °/px, igual al micropaso de 1/16 | medido en la RPi |
| Valor de λ, sin feedforward | el error no cambia entre 0,60 y 0,95 | simulado |

## 4. Cómo medirlo en la plataforma

`herramientas/analizar_registro.py` lee el CSV de una corrida y entrega el presupuesto medido. Sirve para cualquier registro, incluidos los anteriores.

```bash
python3 herramientas/analizar_registro.py registros/camara_trama_*.csv
python3 herramientas/analizar_registro.py sin_ff.csv con_ff.csv        # compara dos corridas
```

Qué mirar y qué esperar:

| Salida | Valor esperado | Si no se cumple |
|---|---|---|
| Cuadros perdidos | 0 | falta CPU o hay limitación térmica: revisar `vcgencmd get_throttled` |
| Captura: exposición → matriz | 15,5 ms más la exposición | más que eso: la RPi no alcanza a retirar los cuadros |
| Detección | bajo 1 ms | revisar si el detector cae seguido a la búsqueda en el cuadro completo |
| Cuadros con objetivo válido | sobre 95 % | ajustar exposición y umbral desde el tablero |
| Mediciones rechazadas por el gating | bajo 1 a 2 % | subir el piso del gating para ese objetivo |
| **Atraso de la plataforma** | 100 a 330 ms sin feedforward; bajo 30 ms con feedforward | con feedforward y atraso alto: revisar escala (micropasos) |
| ω_cmd en saturación | 0 % | el objetivo es más rápido que 180 °/s, o la ganancia es excesiva |

En las dos corridas simuladas de prueba, el punto "atraso" dio 244 ms en pan sin feedforward y 14 ms con feedforward.

## 5. Orden sugerido

1. **Hoy, sin código:** confirmar micropasos (`p`), activar el feedforward, pasar una corrida real por `analizar_registro.py`.
2. **Banco, una sesión:** escalón con `respuesta_escalon.py` para medir ζ y fijar Kp; prueba de aceleración máxima sin pérdida de pasos.
3. **Código, bajo riesgo:** reinicio informado del predictor; contador de atrasos del lazo de 1 kHz en el firmware.
4. **Código, mayor alcance:** patrón de búsqueda; modo recortado de la cámara como perfil de precisión.

## 6. Documentos relacionados

- `docs/feedforward_velocidad.pdf`: qué es el feedforward, cómo está implementado en la RPi y en el ESP32, y cómo probarlo. Cifras del lazo interno simulado, eje pan: a 20 °/s el error baja de 5,4° a cerca de 0; con un objetivo senoidal de 15° y 0,25 Hz, de 5,67° a 0,07°. Un error de escala del 20 % en ω_ff deja 1,1 a 1,2°.
- `docs/prediccion_rls.pdf`: predictor RLS y efecto de λ.
- `docs/analisis_generador_pasos.md`: generador de pasos del ESP32.
