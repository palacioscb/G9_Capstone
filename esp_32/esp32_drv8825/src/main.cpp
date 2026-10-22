/*
 * poc_drv8825 — Prueba de concepto ESP32 + DRV8825 (IEE2913, Grupo 9)
 * -----------------------------------------------------------------------
 * Generador de tren de pulsos STEP/DIR con rampa de aceleración (perfil
 * trapezoidal), control por comandos seriales, modo perilla (potenciómetro)
 * y MODO OSCILACIÓN para demostración de banco.
 *
 * Compila en Arduino-ESP32 core 2.x (PlatformIO espressif32 6.x) y core 3.x
 * (Arduino IDE reciente). Las diferencias de API del temporizador se
 * resuelven con ESP_ARDUINO_VERSION_MAJOR.
 *
 * CABLEADO (ESP32 DevKit -> módulo DRV8825 tipo Pololu)
 *   GPIO25 -> STEP        GPIO32 -> M0        GPIO34 <- FAULT (pull-up 10k a 3V3)
 *   GPIO26 -> DIR         GPIO33 -> M1        GPIO35 <- potenciómetro (0-3V3), opcional
 *   GPIO27 -> ENABLE      GPIO14 -> M2        3V3    -> SLEEP + RESET (unidos)
 *   GND    -> GND (tierra común con la fuente de VMOT)
 *   Fuente 12 V -> VMOT/GND con electrolítico >= 100 uF junto al módulo.
 *   Resistencias: 4k7 pull-up en ENABLE, 10k pull-up en FAULT, 100R serie en STEP y DIR.
 *
 * COMANDOS (Monitor serie, 115200 baud, fin de línea NL):
 *   v <pasos/s>     velocidad continua (signo = sentido). Ej: v 800  / v -400
 *   a <pasos/s^2>   aceleración de la rampa (0 = escalón, sin rampa). Ej: a 2000
 *   g <pasos>       mover N pasos relativos con perfil trapezoidal. Ej: g 3200
 *   o <grados>      OSCILACIÓN continua +/-grados en torno a la posición actual.
 *                   'o' sin argumento usa 60 grados. 'o 0' la detiene.
 *   s               detener con rampa
 *   x               detener de inmediato y deshabilitar driver
 *   e <0|1>         ENABLE del driver (1 = habilitado)
 *   m <1|2|4|8|16|32>  microstepping (M0..M2)
 *   k <0|1>         modo perilla: velocidad desde el potenciómetro
 *   z               poner contador de posición en cero
 *   p               imprimir estado
 *   ?               ayuda
 *
 * Unidades: "pasos" = micropasos en el modo activo. Grados = pasos * 1.8 / M.
 * Velocidad mínima útil ~20 pasos/s (bajo eso el tren se detiene).
 *
 * AVISO: no hay finales de carrera cableados. El modo oscilación no sabe dónde
 * están los topes mecánicos. Verifica el recorrido libre a mano (con 'e 0')
 * antes de lanzar la oscilación, y centra el eje antes de arrancar.
 */

#include <Arduino.h>
#include "soc/gpio_struct.h"   // acceso directo a GPIO.out_w1ts / out_w1tc desde la ISR

// ---------------- Pines ----------------
const int PIN_STEP  = 25;
const int PIN_DIR   = 26;
const int PIN_EN    = 27;   // activo en bajo
const int PIN_M0    = 32;
const int PIN_M1    = 33;
const int PIN_M2    = 14;
const int PIN_FAULT = 34;   // entrada, activo en bajo, requiere pull-up externo
const int PIN_POT   = 35;   // ADC1_CH7

// ---------------- Configuración ----------------
const bool  USE_FAULT_PIN  = true;      // false si FAULT no está cableado (evita avisos falsos)
const float V_MIN_STEPS    = 40.0f;     // velocidad de arranque/frenado de la rampa (pasos/s)
const float V_MAX_STEPS    = 40000.0f;  // tope de seguridad (pasos/s)
const float KNOB_V_MAX     = 3200.0f;   // velocidad con la perilla al máximo (pasos/s)
const uint32_t RAMP_DT_US  = 1000;      // período de actualización de la rampa (1 ms)
const float FULL_STEP_DEG  = 1.8f;

// ---------------- Configuración del modo oscilación ----------------
const float    OSC_DEG_DEFAULT = 60.0f;   // amplitud por defecto ('o' sin argumento)
const float    OSC_DEG_MAX     = 180.0f;  // tope de seguridad de amplitud
const uint32_t OSC_DWELL_MS    = 300;     // pausa en cada extremo antes de invertir

// ---------------- Estado compartido con la ISR ----------------
hw_timer_t* stepTimer = nullptr;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t halfPeriodUs   = 1000;   // medio período del STEP (50 % duty)
volatile bool     stepRunning    = false;
volatile bool     stepLevel      = false;
volatile int8_t   dirSign        = +1;
volatile long     positionSteps  = 0;      // micropasos acumulados con signo
volatile long     remainingSteps = -1;     // -1 = modo velocidad continua
volatile bool     moveDone       = false;

// ---------------- Perfil de velocidad (lado loop) ----------------
float vCurrent      = 0.0f;     // pasos/s (>= 0; el sentido va en dirSign)
float vTarget       = 0.0f;
float accel         = 2000.0f;  // pasos/s^2 (0 = sin rampa)
float lastVCommand  = 800.0f;   // magnitud del último 'v', usada como v_max de 'g' y de 'o'
bool  pendingReverse = false;
float pendingV      = 0.0f;
int   microstep     = 16;
bool  knobMode      = false;
bool  driverEnabled = false;

// ---------------- Estado del modo oscilación ----------------
bool     oscActive     = false;
float    oscDeg        = OSC_DEG_DEFAULT;  // amplitud comandada, en grados
long     oscSteps      = 0;                // amplitud en micropasos
long     oscCenter     = 0;                // posición central de la oscilación
int8_t   oscSide       = +1;               // próximo extremo a visitar (+1 / -1)
uint32_t oscDwellUntil = 0;                // marca de tiempo de fin de la pausa
uint32_t oscLegs       = 0;                // tramos (medios ciclos) completados

// ============ Temporizador (abstracción de versión de core) ============
static inline void timerSetHalfPeriod(uint32_t us) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerAlarm(stepTimer, us, true, 0);
#else
  timerAlarmWrite(stepTimer, us, true);
#endif
}

static inline void timerRun(bool on) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (on) { timerWrite(stepTimer, 0); timerStart(stepTimer); }
  else    { timerStop(stepTimer); }
#else
  if (on) { timerWrite(stepTimer, 0); timerAlarmEnable(stepTimer); }
  else    { timerAlarmDisable(stepTimer); }
#endif
}

// ============ ISR: cada disparo conmuta STEP (50 % duty) ============
void IRAM_ATTR onStepTimer() {
  static uint32_t applied = 0;
  if (!stepRunning) {                     // tren detenido: garantizar STEP en bajo
    GPIO.out_w1tc = (1UL << PIN_STEP);
    stepLevel = false;
    return;
  }
  uint32_t hp = halfPeriodUs;
  if (hp != applied) {                    // el cambio se aplica justo tras recargar el contador
    applied = hp;
    timerSetHalfPeriod(hp);
  }
  stepLevel = !stepLevel;
  if (stepLevel) {
    GPIO.out_w1ts = (1UL << PIN_STEP);    // flanco de subida = un (micro)paso
    positionSteps += dirSign;
    if (remainingSteps > 0) {
      remainingSteps--;
      if (remainingSteps == 0) { moveDone = true; stepRunning = false; }
    }
  } else {
    GPIO.out_w1tc = (1UL << PIN_STEP);
  }
}

// ============ Driver ============
void setEnable(bool en) {
  driverEnabled = en;
  digitalWrite(PIN_EN, en ? LOW : HIGH);
}

void setMicrostep(int m) {
  // DRV8825: M2 M1 M0 -> 000 full, 001 1/2, 010 1/4, 011 1/8, 100 1/16, 101 1/32
  int code;
  switch (m) {
    case 1:  code = 0; break;
    case 2:  code = 1; break;
    case 4:  code = 2; break;
    case 8:  code = 3; break;
    case 16: code = 4; break;
    case 32: code = 5; break;
    default: Serial.println("m: valor no válido (1,2,4,8,16,32)"); return;
  }
  microstep = m;
  digitalWrite(PIN_M0, code & 1);
  digitalWrite(PIN_M1, (code >> 1) & 1);
  digitalWrite(PIN_M2, (code >> 2) & 1);
  Serial.printf("Microstepping 1/%d  (resolución de comando %.4f deg)\n", m, FULL_STEP_DEG / m);
}

void setDirection(int8_t sign) {
  // Solo se cambia DIR con el tren detenido (STEP en bajo) -> cumple t_setup de 650 ns
  portENTER_CRITICAL(&mux);
  dirSign = sign;
  portEXIT_CRITICAL(&mux);
  digitalWrite(PIN_DIR, sign > 0 ? HIGH : LOW);
  delayMicroseconds(5);
}

void stopNow() {
  timerRun(false);
  portENTER_CRITICAL(&mux);
  stepRunning = false;
  remainingSteps = -1;
  stepLevel = false;
  portEXIT_CRITICAL(&mux);
  GPIO.out_w1tc = (1UL << PIN_STEP);
  vCurrent = 0; vTarget = 0;
}

void startRunning(float v0) {
  halfPeriodUs = (uint32_t)(500000.0f / v0);
  portENTER_CRITICAL(&mux);
  stepRunning = true;
  stepLevel = false;
  portEXIT_CRITICAL(&mux);
  timerSetHalfPeriod(halfPeriodUs);
  timerRun(true);
}

// v con signo: sentido + magnitud. Con rampa arranca desde V_MIN.
void commandVelocity(float vSigned) {
  float mag = fabsf(vSigned);
  if (mag > V_MAX_STEPS) mag = V_MAX_STEPS;
  int8_t sign = (vSigned >= 0) ? +1 : -1;

  if (stepRunning && sign != dirSign) {
    // inversión: frenar a cero y luego partir en el otro sentido
    vTarget = 0;
    pendingReverse = true; pendingV = vSigned;
    return;
  }
  if (!stepRunning) setDirection(sign);
  portENTER_CRITICAL(&mux);
  remainingSteps = -1;
  portEXIT_CRITICAL(&mux);
  vTarget = mag;
  if (mag > 0 && !stepRunning) {
    vCurrent = (accel > 0) ? min(V_MIN_STEPS, mag) : mag;
    startRunning(vCurrent);
  }
}

void commandMove(long steps) {
  if (steps == 0) return;
  if (stepRunning) { Serial.println("Detén primero (s) antes de usar g"); return; }
  setDirection(steps > 0 ? +1 : -1);
  moveDone = false;
  portENTER_CRITICAL(&mux);
  remainingSteps = labs(steps);
  portEXIT_CRITICAL(&mux);
  float vmax = (lastVCommand > 0) ? lastVCommand : 800.0f;
  vTarget = vmax;
  vCurrent = (accel > 0) ? min(V_MIN_STEPS, vmax) : vmax;
  startRunning(vCurrent);
}

// ============ Modo oscilación ============
// Barrido continuo entre +amplitud y -amplitud en torno a la posición que
// tenía el eje al arrancar el modo. Se implementa encadenando movimientos
// posicionales ('g') mediante objetivos ABSOLUTOS: cada tramo se calcula como
// (objetivo - posición actual), de modo que el contador no acumula deriva de
// redondeo aunque se cambie la amplitud o se interrumpa un tramo.

void stopOscillation(const char* motivo) {
  if (!oscActive) return;
  oscActive = false;
  Serial.printf("OSC off (%s). Tramos completados = %lu | pos = %ld pasos (%.3f deg)\n",
                motivo, (unsigned long)oscLegs, positionSteps,
                positionSteps * FULL_STEP_DEG / microstep);
}

void startOscillation(float deg) {
  if (stepRunning) { Serial.println("o: detén primero (s) antes de iniciar la oscilación"); return; }
  if (knobMode)    { Serial.println("o: desactiva el modo perilla primero (k 0)"); return; }
  if (!driverEnabled) { Serial.println("o: habilita el driver primero (e 1)"); return; }
  if (deg <= 0.0f || deg > OSC_DEG_MAX) {
    Serial.printf("o: amplitud fuera de rango (0 < grados <= %.0f)\n", OSC_DEG_MAX);
    return;
  }

  oscDeg    = deg;
  oscSteps  = lroundf(deg * (float)microstep / FULL_STEP_DEG);
  if (oscSteps <= 0) { Serial.println("o: amplitud menor a un micropaso"); return; }

  oscCenter     = positionSteps;
  oscSide       = +1;
  oscLegs       = 0;
  oscDwellUntil = millis();
  oscActive     = true;

  float vmax = (lastVCommand > 0) ? lastVCommand : 800.0f;
  Serial.printf("OSC on: +/-%.3f deg = +/-%ld pasos (1/%d) | centro = %ld pasos | v = %.0f pasos/s | pausa %lu ms\n",
                oscSteps * FULL_STEP_DEG / microstep, oscSteps, microstep,
                oscCenter, vmax, (unsigned long)OSC_DWELL_MS);
  Serial.println("ATENCION: sin finales de carrera. Verifica el recorrido libre antes de continuar ('s' detiene).");
}

void updateOscillation() {
  if (!oscActive || stepRunning) return;
  if ((int32_t)(millis() - oscDwellUntil) < 0) return;   // aún en la pausa del extremo

  long target = oscCenter + (long)oscSide * oscSteps;
  long delta  = target - positionSteps;
  oscSide = -oscSide;                                     // el próximo tramo va al otro extremo

  if (delta == 0) { oscDwellUntil = millis() + OSC_DWELL_MS; return; }
  commandMove(delta);
}

// ============ Rampa (se ejecuta cada RAMP_DT_US desde loop) ============
void updateRamp() {
  static uint32_t tLast = micros();
  uint32_t now = micros();
  if (now - tLast < RAMP_DT_US) return;
  float dt = (now - tLast) * 1e-6f;
  tLast = now;

  if (moveDone) {                          // la ISR ya congeló el tren en el último paso
    moveDone = false;
    stopNow();
    if (oscActive) {                       // tramo de oscilación terminado: pausa y sigue
      oscLegs++;
      oscDwellUntil = millis() + OSC_DWELL_MS;
      Serial.printf("OSC tramo %lu -> pos = %ld pasos (%.3f deg)\n",
                    (unsigned long)oscLegs, positionSteps,
                    positionSteps * FULL_STEP_DEG / microstep);
    } else {
      Serial.printf("Movimiento completo. Posición = %ld pasos (%.3f deg)\n",
                    positionSteps, positionSteps * FULL_STEP_DEG / microstep);
    }
    return;
  }

  if (!stepRunning) {
    if (pendingReverse) { pendingReverse = false; commandVelocity(pendingV); }
    return;
  }

  // Movimiento posicional: frenar cuando lo restante iguala la distancia de desaceleración
  if (remainingSteps > 0 && accel > 0) {
    float dDecel = (vCurrent * vCurrent - V_MIN_STEPS * V_MIN_STEPS) / (2.0f * accel);
    if ((float)remainingSteps <= dDecel) vTarget = V_MIN_STEPS;
  }

  if (accel <= 0) {                        // sin rampa: escalón de frecuencia
    vCurrent = vTarget;
  } else {
    float dv = accel * dt;
    if (vCurrent < vTarget)      vCurrent = min(vCurrent + dv, vTarget);
    else if (vCurrent > vTarget) vCurrent = max(vCurrent - dv, vTarget);
  }

  if (vCurrent < V_MIN_STEPS * 0.5f) { stopNow(); return; }
  halfPeriodUs = (uint32_t)(500000.0f / vCurrent);
}

// ============ Perilla ============
void updateKnob() {
  static uint32_t tLast = 0;
  if (!knobMode || millis() - tLast < 50) return;
  tLast = millis();
  int raw = analogRead(PIN_POT);                  // 0..4095
  float v = (raw - 2048) / 2048.0f * KNOB_V_MAX;  // centro = detenido, extremos = ±máx
  if (fabsf(v) < KNOB_V_MAX * 0.05f) v = 0;       // zona muerta
  commandVelocity(v);
}

// ============ Estado ============
bool faultActive() {
  return USE_FAULT_PIN && (digitalRead(PIN_FAULT) == LOW);
}

void printStatus() {
  float fstep = stepRunning ? vCurrent : 0.0f;
  Serial.printf("pos=%ld pasos (%.3f deg) | f_STEP=%.1f Hz (%.2f deg/s) | a=%.0f | 1/%d | dir=%c | "
                "EN=%d | FAULT=%s | perilla=%d | osc=%s",
                positionSteps, positionSteps * FULL_STEP_DEG / microstep,
                fstep, fstep * FULL_STEP_DEG / microstep,
                accel, microstep, dirSign > 0 ? '+' : '-', driverEnabled,
                faultActive() ? "ACTIVA" : "ok", knobMode,
                oscActive ? "on" : "off");
  if (oscActive) Serial.printf(" (+/-%.1f deg, tramos=%lu)", oscDeg, (unsigned long)oscLegs);
  Serial.println();
}

void printHelp() {
  Serial.println(F("v <pasos/s> | a <pasos/s^2> (0=sin rampa) | g <pasos> | o <grados> (0=off) | "
                   "s | x | e 0/1 | m 1..32 | k 0/1 | z | p | ?"));
}

// ============ Setup / loop ============
void setup() {
  Serial.begin(115200);
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_M0, OUTPUT); pinMode(PIN_M1, OUTPUT); pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_FAULT, INPUT);
  analogReadResolution(12);
  digitalWrite(PIN_STEP, LOW);
  setEnable(false);
  setDirection(+1);
  setMicrostep(16);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(1000000);                 // 1 MHz -> resolución 1 us
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, halfPeriodUs, true, 0);
  timerStop(stepTimer);
#else
  stepTimer = timerBegin(0, 80, true);             // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, halfPeriodUs, true);  // alarma deshabilitada hasta el primer comando
#endif

  Serial.println("\nPoC ESP32 + DRV8825 listo. Driver DESHABILITADO: usa 'e 1' para habilitar.");
  printHelp();
}

void handleSerial() {
  static char buf[32]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0; n = 0;
      if (buf[0] == 0) continue;
      char cmd = buf[0]; float arg = atof(buf + 1);
      switch (cmd) {
        case 'v': stopOscillation("comando v"); lastVCommand = fabsf(arg); commandVelocity(arg); Serial.printf("v -> %.1f pasos/s\n", arg); break;
        case 'a': accel = max(0.0f, arg); Serial.printf("a -> %.0f pasos/s^2%s\n", accel, accel == 0 ? " (sin rampa)" : ""); break;
        case 'g': stopOscillation("comando g"); commandMove((long)arg); Serial.printf("g -> %ld pasos\n", (long)arg); break;
        case 'o': {
          bool hasArg = false;
          for (uint8_t i = 1; buf[i]; i++) if (buf[i] >= '0' && buf[i] <= '9') { hasArg = true; break; }
          if (!hasArg)        startOscillation(OSC_DEG_DEFAULT);   // 'o' solo = amplitud por defecto
          else if (arg <= 0)  stopOscillation("comando o 0");
          else                startOscillation(arg);
          break;
        }
        case 's': stopOscillation("comando s"); vTarget = 0; if (accel <= 0) stopNow(); Serial.println("deteniendo..."); break;
        case 'x': stopOscillation("comando x"); stopNow(); setEnable(false); Serial.println("STOP inmediato, driver deshabilitado"); break;
        case 'e': setEnable(arg >= 0.5f); if (!driverEnabled) stopOscillation("driver deshabilitado"); Serial.printf("ENABLE -> %d\n", driverEnabled); break;
        case 'm': if (stepRunning || oscActive) Serial.println("cambia microstepping con el motor detenido y la oscilación apagada");
                  else setMicrostep((int)arg); break;
        case 'k': stopOscillation("modo perilla"); knobMode = (arg >= 0.5f); if (!knobMode) vTarget = 0; Serial.printf("perilla -> %d\n", knobMode); break;
        case 'z': stopOscillation("comando z"); portENTER_CRITICAL(&mux); positionSteps = 0; portEXIT_CRITICAL(&mux); Serial.println("posición = 0"); break;
        case 'p': printStatus(); break;
        case '?': printHelp(); break;
        default:  Serial.println("comando desconocido; '?' para ayuda");
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void loop() {
  handleSerial();
  updateKnob();
  updateRamp();
  updateOscillation();          // encadena los tramos del modo oscilación

  // Aviso de FAULT por flanco
  static bool lastFault = false;
  bool fault = faultActive();
  if (fault && !lastFault) {
    Serial.println("!! FAULT del DRV8825 (sobrecorriente / térmico / UVLO)");
    if (oscActive) { stopOscillation("FAULT"); stopNow(); }   // no seguir barriendo en falla
  }
  lastFault = fault;

  // Estado periódico mientras se mueve
  static uint32_t tPrint = 0;
  if (stepRunning && millis() - tPrint > 500) { tPrint = millis(); printStatus(); }
}