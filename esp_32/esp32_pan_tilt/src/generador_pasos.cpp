/*
 * generador_pasos.cpp — CS-7 / CS-8: tren de pulsos STEP/DIR por oscilador numérico.
 */
#include "generador_pasos.h"
#include "config.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

namespace pasos {

//* Estado de un eje compartido entre la ISR y el lazo de control.
//* Las escrituras de 32 bits alineadas son atómicas en el ESP32, así que el lazo puede
//* actualizar `incremento` y `dirDeseada` sin secciones críticas.
struct Generador {
  uint32_t maskStep = 0, maskDir = 0;
  volatile uint32_t incremento = 0;   // fase por tic: v · 2^32 / F_TICK
  volatile int8_t   dirDeseada = +1;  // +1 -> DIR en alto
  volatile int32_t  posicion   = 0;   // micropasos emitidos con signo
  uint32_t fase = 0;
  int8_t   dirActual = +1;
  bool     stepAlto  = false;
  bool     pendiente = false;         // hubo desborde y falta emitir el paso
};

static Generador gen[N_EJES];
static hw_timer_t* temporizador = nullptr;
static bool habil[N_EJES] = {false, false};

//! Los pines STEP y DIR se escriben con los registros W1TS/W1TC, que sólo cubren
//! GPIO 0–31. Si se cambia un pin a 32 o más, este chequeo impide compilar.
static_assert(EJES[0].pinStep < 32 && EJES[0].pinDir < 32 &&
              EJES[1].pinStep < 32 && EJES[1].pinDir < 32,
              "STEP/DIR deben ser GPIO < 32");

// ---------------------------------------------------------------------------
//* Un tic para un eje. Orden de operaciones:
//*   1. si STEP quedó en alto el tic anterior, se baja (pulso de exactamente 1 tic);
//*   2. se acumula la fase; un desborde deja un paso pendiente;
//*   3. si hay paso pendiente y STEP está en bajo: si el sentido cambió, primero se
//*      escribe DIR y el paso sale en el tic siguiente (≥ 25 µs de setup, el DRV8825
//*      pide 650 ns); si no, se sube STEP y se cuenta el paso.
// ---------------------------------------------------------------------------
static inline void IRAM_ATTR tickEje(Generador& g) {
  bool bajoEnEsteTic = false;
  if (g.stepAlto) {
    REG_WRITE(GPIO_OUT_W1TC_REG, g.maskStep);
    g.stepAlto = false;
    bajoEnEsteTic = true;
  }

  const uint32_t antes = g.fase;
  g.fase += g.incremento;
  if (g.fase < antes) g.pendiente = true;          // desborde = un micropaso

  if (!g.pendiente || bajoEnEsteTic) return;       // el pulso necesita un tic en bajo

  const int8_t d = g.dirDeseada;
  if (d != g.dirActual) {                          // cambio de sentido con STEP en bajo
    REG_WRITE(d > 0 ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, g.maskDir);
    g.dirActual = d;
    return;                                        // el paso sale en el próximo tic
  }
  REG_WRITE(GPIO_OUT_W1TS_REG, g.maskStep);
  g.stepAlto  = true;
  g.pendiente = false;
  g.posicion  = g.posicion + d;
}

static void IRAM_ATTR isrTick() {
  tickEje(gen[0]);
  tickEje(gen[1]);
}

// ---------------------------------------------------------------------------
void fijarMicropasos(uint8_t micropasos) {
  //* Sólo si M0..M2 están cableados al ESP32. Si no, los micropasos los fija el hardware
  //* del driver y el firmware sólo usa el valor para convertir pasos en grados.
  if (!USAR_PINES_MICROPASO) return;
  // DRV8825: M2 M1 M0 -> 000 1/1, 001 1/2, 010 1/4, 011 1/8, 100 1/16, 101 1/32
  uint8_t codigo;
  switch (micropasos) {
    case 1: codigo = 0; break;   case 2: codigo = 1; break;   case 4: codigo = 2; break;
    case 8: codigo = 3; break;   case 16: codigo = 4; break;  case 32: codigo = 5; break;
    default: codigo = 4; break;
  }
  pinMode(PIN_M0, OUTPUT); pinMode(PIN_M1, OUTPUT); pinMode(PIN_M2, OUTPUT);
  digitalWrite(PIN_M0, codigo & 1);
  digitalWrite(PIN_M1, (codigo >> 1) & 1);
  digitalWrite(PIN_M2, (codigo >> 2) & 1);
}

void iniciar(uint8_t micropasos) {
  fijarMicropasos(micropasos);

  for (uint8_t e = 0; e < N_EJES; e++) {
    const ConfigEje& c = EJES[e];
    pinMode(c.pinEn, OUTPUT);
    digitalWrite(c.pinEn, HIGH);                   //* parte DESHABILITADO
    pinMode(c.pinStep, OUTPUT); digitalWrite(c.pinStep, LOW);
    pinMode(c.pinDir, OUTPUT);  digitalWrite(c.pinDir, HIGH);
    gen[e].maskStep = 1UL << c.pinStep;
    gen[e].maskDir  = 1UL << c.pinDir;
    gen[e].dirActual = +1;
    gen[e].dirDeseada = +1;
    if (c.pinFault != PIN_NC) {
      if (c.pinFault < 34) pinMode(c.pinFault, INPUT_PULLUP);  // 34–39 no tienen pull-up interno
      else                 pinMode(c.pinFault, INPUT);
    }
    if (c.pinFin != PIN_NC) pinMode(c.pinFin, INPUT_PULLUP);   //* topes NC a GND: en reposo LOW
  }

  const uint32_t periodo_us = 1000000UL / F_TICK_HZ;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  temporizador = timerBegin(1000000);              // 1 MHz
  timerAttachInterrupt(temporizador, &isrTick);
  timerAlarm(temporizador, periodo_us, true, 0);
#else
  temporizador = timerBegin(0, 80, true);          // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(temporizador, &isrTick, true);
  timerAlarmWrite(temporizador, periodo_us, true);
  timerAlarmEnable(temporizador);
#endif
  //* La alarma se programa UNA vez y nunca se toca desde la ISR (riesgo de la PoC resuelto).
}

void fijarVelocidad(uint8_t eje, float v) {
  if (eje >= N_EJES) return;
  float mag = fabsf(v);
  if (mag > PASOS_S_MAX) mag = PASOS_S_MAX;
  if (!habil[eje]) mag = 0.0f;                     // deshabilitado: no se emiten pasos
  // incremento = mag · 2^32 / F_TICK  (double: el ESP32 lo emula, pero esto corre a 1 kHz)
  const uint32_t inc = (uint32_t)((double)mag * (4294967296.0 / (double)F_TICK_HZ));
  if (inc > 0) gen[eje].dirDeseada = (v >= 0.0f) ? +1 : -1;   // con v = 0 se conserva DIR
  gen[eje].incremento = inc;
}

int32_t posicion(uint8_t eje) {
  return (eje < N_EJES) ? gen[eje].posicion : 0;
}

void habilitar(uint8_t eje, bool on) {
  if (eje >= N_EJES) return;
  habil[eje] = on;
  if (!on) gen[eje].incremento = 0;
  digitalWrite(EJES[eje].pinEn, on ? LOW : HIGH);  // nENABLE activo en bajo
}

bool habilitado(uint8_t eje) {
  return (eje < N_EJES) ? habil[eje] : false;
}

}  // namespace pasos
