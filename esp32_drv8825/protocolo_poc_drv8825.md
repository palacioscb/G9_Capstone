# Prueba de concepto ESP32 + DRV8825 — protocolo de banco

Proyecto III, plataforma 2-DOF de seguimiento visual (IEE2913, Grupo 9). Cubre las tareas de la carta Gantt "Protoboard DRV8825 + prueba de concepto" (ACT-3, ACT-6) y "Pruebas en banco NEMA 17" (ACT-1), semanas S4–S5. Objetivo: caracterizar el actuador en lazo abierto y obtener los números que acotan el lazo interno de posición (S6–S8).

Instrumentos: osciloscopio, multímetro, fuente bipolar, protoboard, ESP32, módulo DRV8825 (tipo Pololu). Firmware: `poc_drv8825.ino` (Arduino / PlatformIO).

## 0. Reglas de seguridad del banco

1. Condensador electrolítico ≥ 100 µF / 25 V entre VMOT y GND, a menos de 3 cm del módulo, con la polaridad correcta. Sin él, los picos LC al conectar la fuente pueden destruir el DRV8825 (máximo absoluto 45 V).
2. Nunca conectar ni desconectar el motor con VMOT energizado. Secuencia: cablear todo → ajustar Vref → conectar motor → energizar lógica (USB) → energizar VMOT. Para apagar: VMOT primero.
3. Fuente: un canal a 12.0 V, límite de corriente en 1.0 A al partir (subir a 1.5–2 A solo en la prueba D si el driver entra en UVLO por caída de tensión).
4. Tierra común: GND de la fuente, GND del módulo y GND del ESP32 unidos.
5. Disipador sobre el CI. Sin disipador, el DRV8825 sostiene ~1.5 A/fase; con disipador y flujo de aire hasta 2.2 A.
6. Vref siempre se ajusta con el motor desconectado y VMOT energizado (el módulo no tiene pin de lógica separado: se alimenta internamente desde VMOT).
7. Las puntas del osciloscopio tienen la referencia unida a tierra: nunca conectar la pinza de tierra a un terminal del devanado ni a VMOT.

## 1. Cableado

| ESP32 (GPIO) | DRV8825 | Nota |
|---|---|---|
| 25 | STEP | flanco de subida = un micropaso |
| 26 | DIR | se cambia solo con el tren detenido |
| 27 | ENABLE | activo en bajo; el firmware parte deshabilitado |
| 32 / 33 / 14 | M0 / M1 / M2 | microstepping; pull-down internos (por defecto paso completo) |
| 34 | FAULT | activo en bajo; agregar pull-up 10 kΩ a 3V3 |
| 35 | potenciómetro (cursor) | extremos a 3V3 y GND; opcional, modo perilla `k 1` |
| 3V3 | SLEEP y RESET (unidos) | ambos deben estar en alto; en el módulo Pololu FAULT está ligado a SLEEP |
| GND | GND | tierra común |
| — | VMOT / GND | 12 V de la fuente + electrolítico ≥ 100 µF |
| — | A1, A2, B1, B2 | devanado A entre A1–A2, devanado B entre B1–B2 |

Identificación de devanados: con el multímetro en ohmios, los pares con resistencia de pocos ohmios (1–5 Ω típico) son un mismo devanado; entre devanados hay circuito abierto. Anotar R medida.

Tabla de microstepping (M2 M1 M0): 000 = 1/1, 001 = 1/2, 010 = 1/4, 011 = 1/8, 100 = 1/16, 101 = 1/32.

## 2. Fase 1 — sin motor (se puede hacer ahora)

### Prueba A — señales lógicas ESP32 → DRV8825

Cargar el firmware, `e 0` (driver deshabilitado), `a 0`, `v 1000`. Osciloscopio en STEP (CH1) y DIR (CH2).

| Magnitud | Especificación DRV8825 | Medido | Cumple |
|---|---|---|---|
| Frecuencia STEP a `v 1000` | 1000 Hz | | |
| Ancho de pulso alto | ≥ 1.9 µs | | |
| Ancho de pulso bajo | ≥ 1.9 µs | | |
| Nivel alto | ≥ 2.2 V (V_IH) | ~3.3 V | |
| Tiempo de subida | — | | |
| Setup DIR → flanco STEP | ≥ 650 ns | (cambio con tren detenido) | |
| Jitter de período (persistencia) | — | | |

Repetir a `v 100`, `v 5000` y `v 20000`. Con `a 2000` y luego `v 3000`, capturar en modo roll la frecuencia creciente (rampa) — esa captura sirve para la lámina de "generador de rampas".

### Prueba B — ajuste de corriente (Vref)

Con VMOT energizado y motor desconectado, medir con el multímetro entre el cursor del potenciómetro y GND. En módulos con R_sense = 0.100 Ω (marcadas "R100"): I_límite = 2·Vref. Verificar la marca de la resistencia de sensado del módulo antes de usar la fórmula. En paso completo el driver aplica el 71 % de I_límite; en microstepping el valor pico es I_límite.

| Motor (modelo) | I nominal/fase | Vref objetivo (70 % de nominal, sin disipador) | Vref medido | I_límite resultante |
|---|---|---|---|---|
| | | | | |

Mientras no haya motor, dejar Vref ≈ 0.4 V (0.8 A) como valor seguro.

### Prueba A2 — secuencia de fases sin carga (opcional)

Con el driver habilitado y sin motor, medir A1 y B1 respecto a GND: cada salida conmuta entre 0 y VMOT siguiendo la secuencia de pasos. En microstepping la polaridad del devanado invierte en los cruces por cero de la senoide, de modo que la frecuencia visible en A1 es f_STEP / (4·M), con M el factor de microstepping (un ciclo eléctrico = 4 pasos completos). Verificarlo con `m 1` y `m 16` a la misma `v`. Sin corriente no hay chopper: la forma de onda con carga se ve en la prueba C.

## 3. Fase 2 — con motor (apenas llegue cualquier NEMA 17 bipolar)

### Prueba C — forma de onda de corriente de fase

Opción 1 (recomendada): shunt externo de 0.22–0.47 Ω / 2 W en serie con el devanado A, medición diferencial CH1 − CH2 (función MATH), escala de corriente = V_math / R_shunt.
Opción 2: punta única sobre el pad de la resistencia de sensado del módulo (nodo referido a tierra). Solo muestra la corriente durante la conducción del puente (en decaimiento lento la corriente recircula por los MOSFET bajos y no pasa por el shunt), pero la envolvente se ve igual.

| Modo | I_límite | I pico medido | Rizado del chopper (Δi) | f de chopper aprox. | Observación |
|---|---|---|---|---|---|
| 1/1, `v 200` | | | | | onda cuadrada ±0.71·I_límite |
| 1/16, `v 3200` | | | | | envolvente senoidal |
| 1/16, motor detenido | | | | | corriente de retención |

Interpretación para el informe: pendiente de subida di/dt ≈ (V_MOT − R·i)/L; con τ_e = L/R (medir R con multímetro; L del datasheet o estimar de la pendiente). Explica la caída de torque con la velocidad y respalda la elección de 12 V (o más) para VMOT.

### Prueba D — límite de velocidad sin pérdida de pasos

Marcar el eje (cinta o disco graduado). Comandar `g 3200` (una vuelta a 1/16) o `g 200` (a 1/1) y comprobar que la marca vuelve al origen. Aumentar `v` hasta que se pierdan pasos; repetir con `a 0` (escalón) y con `a 4000` (rampa).

| Modo | Vref | a (pasos/s²) | f_STEP máx. sin pérdida | ω máx. (°/s) | Zona de resonancia observada |
|---|---|---|---|---|---|
| 1/1 | | 0 | | | |
| 1/1 | | 4000 | | | |
| 1/16 | | 0 | | | |
| 1/16 | | 4000 | | | |

ω = f_STEP · 1.8° / M. Este valor es el límite de slew del lazo interno de posición y acota la velocidad angular de blanco que la plataforma puede seguir (CTL-5). La zona de resonancia (típicamente 100–300 pasos completos/s) justifica el uso de microstepping y las rampas (ROB-4, ROB-5).

### Prueba E — corriente de alimentación vs. carga mecánica (ACT-6)

Multímetro en serie con VMOT (o shunt de 0.1 Ω en el retorno de la fuente, referido a tierra, para verlo en el osciloscopio).

| Condición (1/16, `v 1600`) | I fuente (A) | I fase pico (A) | Comentario |
|---|---|---|---|
| Detenido, driver habilitado | | | corriente de retención |
| Giro libre | | | |
| Frenado a mano (carga parcial) | | | |
| Bloqueado (eje sujeto) | | | ¿sube o baja I fuente? |
| Driver deshabilitado | | | |

Resultado esperado: la corriente de fase se mantiene en I_límite en todas las condiciones (la regula el chopper), y la corriente de fuente no aumenta al bloquear el eje — puede incluso bajar, porque deja de entregarse potencia mecánica. Conclusión de diseño para el informe: la detección de bloqueo y pérdida de pasos se hace con e_θ = θ_ref − θ_med (AS5600); el shunt en la PCB propia y el pin FAULT del DRV8825 cubren sobrecorriente, cortocircuito, UVLO y térmico. Con esto quedan explícitas las "protecciones" que el equipo docente pidió aclarar.

Dato adicional para "Dimensionamiento de fuentes": I fuente máxima observada × 2 motores + margen → consumo del riel de 12 V para elegir batería y conversor DC-DC.

### Prueba F — pasos comandados vs. ángulo real (si hay AS5600)

Lectura I²C del AS5600 desde el ESP32; comparar `g 3200` con Δθ medido. Sin sensor, usar disco graduado y `g 800` (90° a 1/16). Registrar el error angular acumulado en 5 vueltas en cada sentido.

## 4. Demostración para la presentación del 5 % (11 de septiembre)

1. Motor girando con velocidad y sentido comandados desde el ESP32 (`k 1` con potenciómetro, o comandos por serial).
2. Osciloscopio en vivo: STEP (CH1) y corriente de fase (MATH) — cambiar `m 1` ↔ `m 16` para mostrar el microstepping.
3. Contraste: `a 0` + `v 4000` (pierde pasos, vibra) versus `a 4000` + `v 4000` (acelera limpio). Justifica el generador de rampas en el bloque de control.
4. Lámina del lazo interno con variables nombradas: controlada θ, medida θ_med (AS5600), manipulada tren STEP/DIR (ω_cmd), perturbación T_carga / empujón manual.

Numerar todas las figuras y capturas (observación del feedback del 28/08).

## 5. Registro de resultados

Fecha: ____  Participantes: ____  Motor: ____  Vref: ____  VMOT: ____  Disipador: sí / no
Capturas del osciloscopio guardadas en: `pruebas/poc_drv8825/capturas/` con nombre `pruebaX_descripcion.png`.
