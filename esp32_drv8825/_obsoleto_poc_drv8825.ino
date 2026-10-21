/*
 * poc_drv8825.ino — Prueba de concepto ESP32 + DRV8825 (IEE2913, Grupo 9)
 * -----------------------------------------------------------------------
 * Generador de tren de pulsos STEP/DIR con rampa de aceleración (perfil
 * trapezoidal), control por comandos seriales y modo perilla (potenciómetro).
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
 *
 * COMANDOS (Monitor serie, 115200 baud, fin de línea NL):
 *   v <pasos/s>     velocidad continua (signo = sentido). Ej: v 800  / v -400
 *   a <pasos/s^2>   aceleración de la rampa (0 = escalón, sin rampa). Ej: a 2000
 *   g <pasos>       mover N pasos relativos con perfil trapezoidal. Ej: g 3200
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
float lastVCommand  = 800.0f;   // magnitud del último 'v', usada como v_max de 'g'
bool  pendingReverse = false;
float pendingV      = 0.0f;
int   microstep     = 16;
bool  knobMode      = false;
bool  driverEnabled = false;

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
    Serial.printf("Movimiento completo. Posición = %ld pasos (%.3f deg)\n",
                  positionSteps, positionSteps * FULL_STEP_DEG / microstep);
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
                "EN=%d | FAULT=%s | perilla=%d\n",
                positionSteps, positionSteps * FULL_STEP_DEG / microstep,
                fstep, fstep * FULL_STEP_DEG / microstep,
                accel, microstep, dirSign > 0 ? '+' : '-', driverEnabled,
                faultActive() ? "ACTIVA" : "ok", knobMode);
}

void printHelp() {
  Serial.println(F("v <pasos/s> | a <pasos/s^2> (0=sin rampa) | g <pasos> | s | x | e 0/1 | "
                   "m 1..32 | k 0/1 | z | p | ?"));
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
        case 'v': lastVCommand = fabsf(arg); commandVelocity(arg); Serial.printf("v -> %.1f pasos/s\n", arg); break;
        case 'a': accel = max(0.0f, arg); Serial.printf("a -> %.0f pasos/s^2%s\n", accel, accel == 0 ? " (sin rampa)" : ""); break;
        case 'g': commandMove((long)arg); Serial.printf("g -> %ld pasos\n", (long)arg); break;
        case 's': vTarget = 0; if (accel <= 0) stopNow(); Serial.println("deteniendo..."); break;
        case 'x': stopNow(); setEnable(false); Serial.println("STOP inmediato, driver deshabilitado"); break;
        case 'e': setEnable(arg >= 0.5f); Serial.printf("ENABLE -> %d\n", driverEnabled); break;
        case 'm': if (stepRunning) Serial.println("cambia microstepping con el motor detenido"); else setMicrostep((int)arg); break;
        case 'k': knobMode = (arg >= 0.5f); if (!knobMode) vTarget = 0; Serial.printf("perilla -> %d\n", knobMode); break;
        case 'z': portENTER_CRITICAL(&mux); positionSteps = 0; portEXIT_CRITICAL(&mux); Serial.println("posición = 0"); break;
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

  // Aviso de FAULT por flanco
  static bool lastFault = false;
  bool fault = faultActive();
  if (fault && !lastFault) Serial.println("!! FAULT del DRV8825 (sobrecorriente / térmico / UVLO)");
  lastFault = fault;

  // Estado periódico mientras se mueve
  static uint32_t tPrint = 0;
  if (stepRunning && millis() - tPrint > 500) { tPrint = millis(); printStatus(); }
}
