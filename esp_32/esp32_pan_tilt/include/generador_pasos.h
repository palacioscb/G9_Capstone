#pragma once
/*
 * generador_pasos.h — CS-7: generador de pulsos STEP/DIR para los dos ejes.
 *
 * Un solo temporizador de hardware a F_TICK_HZ ejecuta una ISR corta. Para cada eje, un
 * acumulador de fase de 32 bits (oscilador numérico) suma un incremento proporcional a la
 * velocidad pedida; cada desborde es un micropaso. Ventajas frente a reprogramar el
 * temporizador en cada paso (lo que hacía la prueba de concepto):
 *   - la ISR no llama funciones del núcleo Arduino; es segura en IRAM;
 *   - no hay velocidad mínima: cualquier velocidad, por pequeña que sea, se genera
 *     (en la PoC, bajo 20 pasos/s el tren se detenía y quedaba una zona muerta);
 *   - un temporizador sirve para los dos ejes.
 * Costo: el flanco de cada paso cae en la grilla de 25 µs (jitter ≤ 1 tic).
 */
#include <Arduino.h>

namespace pasos {

//* Configura pines STEP/DIR/nENABLE, micropasos y arranca el temporizador.
void iniciar();

//* Velocidad con signo en micropasos/s (+ = DIR en alto). Se satura a ±PASOS_S_MAX.
void fijarVelocidad(uint8_t eje, float pasosPorSegundo);

//* Micropasos emitidos con signo desde el arranque (lo escribe la ISR).
int32_t posicion(uint8_t eje);

//* Habilita (nENABLE en bajo) o deshabilita el driver de un eje.
void habilitar(uint8_t eje, bool on);
bool habilitado(uint8_t eje);

}  // namespace pasos
