# Con y sin feedforward, por grupo de ganancias

Error de centrado: distancia angular del objetivo al centro de la imagen, RMS, sólo en SEGUIMIENTO con detección válida. Atraso: tiempo que la plataforma va detrás del objetivo (regresión del error contra la velocidad del objetivo).

- **Grupo A** (arranque (config.h)): pan Kp 3 · Ki 2,25; tilt Kp 10 · Ki 25
- **Grupo B** (pid_t_2 §8): pan Kp 5 · Ki 6,25; tilt Kp 20 · Ki 100

| Origen | Grupo | Feedforward | Error pan RMS | Error tilt RMS | Atraso pan | Atraso tilt | Vel. objetivo pan (RMS) |
|---|---|---|---|---|---|---|---|
| simulado | A | no | 3,64° (31,6 px) | 0,086° (0,7 px) | 264 ms | 12 ms | 12,4 °/s |
| simulado | A | sí | 0,19° (1,7 px) | 0,012° (0,1 px) | −3 ms | 0 ms | 12,4 °/s |
| simulado | B | no | 1,74° (15,1 px) | 0,044° (0,4 px) | 107 ms | 8 ms | 12,4 °/s |
| simulado | B | sí | 0,15° (1,3 px) | 0,015° (0,1 px) | −4 ms | −1 ms | 12,4 °/s |

| Origen | Grupo | Reducción del error en pan con feedforward | Reducción en tilt |
|---|---|---|---|
| simulado | A | 18,9 veces | 7,3 veces |
| simulado | B | 11,3 veces | 2,9 veces |

Simulado: planta de pid_t_2 con ζ = 0,05 supuesto y escena sintética de 40 s, en régimen (desde 8 s: sin el enganche inicial ni las dos oclusiones programadas). Sirve para comparar casos, no como predicción del error real. Entre simulado y medido se compara el atraso (ms), que no depende de la velocidad con que se movió el objetivo.
