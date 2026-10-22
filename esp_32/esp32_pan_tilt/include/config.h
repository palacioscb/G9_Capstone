#pragma once
/*
 * config.h — TODOS los números del firmware en un solo lugar: pines, ejes, control,
 * enlace y supervisión. Ningún otro archivo debería tener constantes "mágicas".
 *
 * Proyecto IEE2913, Grupo 9 — plataforma pan-tilt, lazo interno (CS-4 a CS-10).
 */
#include <Arduino.h>

//* ============================================================================
//* CONVENCIÓN DE SIGNOS (la misma de pt/geometria.py en la Raspberry Pi)
//*   θ_pan  > 0  -> la cámara gira hacia la DERECHA (mirando en la dirección de la cámara)
//*   θ_tilt > 0  -> la cámara mira hacia ARRIBA
//* Todos los ángulos del firmware están en GRADOS DEL EJE (después de la transmisión).
//* ============================================================================

// ============================================================================
//* PINES — ESP32 DevKit (WROOM-32), agrupados por lado de la placa
// ============================================================================
//  Lado IZQUIERDO (de arriba hacia abajo): fallas de ambos drivers, micropasos compartidos
//  y el eje PAN. Es el mismo cableado del eje de la prueba de concepto (poc_drv8825).
//
//    GPIO34  <- nFAULT pan        (sólo entrada: pull-up EXTERNO 10 kΩ a 3V3)
//    GPIO35  <- nFAULT tilt       (sólo entrada: pull-up EXTERNO 10 kΩ a 3V3)
//    GPIO32  -> M0  (ambos drivers)
//    GPIO33  -> M1  (ambos drivers)
//    GPIO25  -> STEP pan
//    GPIO26  -> DIR  pan
//    GPIO27  -> nENABLE pan       (pull-up EXTERNO 4,7 kΩ a 3V3)
//    GPIO14  -> M2  (ambos drivers)
//    GPIO13  <- final de carrera pan  (los 2 topes NC en serie a GND; pull-up interno)
//
//  Lado DERECHO (de arriba hacia abajo): fin de carrera, sensores y eje TILT.
//
//    GPIO23  <- final de carrera tilt (los 2 topes NC en serie a GND; pull-up interno)
//    GPIO22  -> SCL AS5600 pan    (bus I2C 0, "Wire")
//    GPIO21  <> SDA AS5600 pan
//    GPIO19  -> SCL AS5600 tilt   (bus I2C 1, "Wire1")
//    GPIO18  <> SDA AS5600 tilt
//    GPIO17  -> STEP tilt
//    GPIO16  -> DIR  tilt
//    GPIO4   -> nENABLE tilt      (pull-up EXTERNO 4,7 kΩ a 3V3)
//
//  Libres y reservados: GPIO36 y GPIO39 (lectura ADC de los shunt, futuro).
//
//! No usar GPIO 0, 2, 5, 12 ni 15 (pines de arranque) ni 6–11 (memoria flash).
//! GPIO4 tiene pull-DOWN interno durante el reinicio: sin el pull-up externo de 4,7 kΩ
//! el driver de tilt queda HABILITADO mientras el ESP32 arranca. Lo mismo vale para
//! GPIO27, que queda flotando. El pull-up es obligatorio en ambos nENABLE.
//! GPIO16/17 sirven en módulos WROOM. En un WROVER están ocupados por la PSRAM.
//* Cada AS5600 va en su propio bus I2C. Tienen dirección fija (0x36), así que con dos
//* buses no hace falta multiplexor y un sensor caído no bloquea al otro.
//? Los módulos AS5600 traen pull-ups de I2C propios. Si el bus falla con cables largos,
//? agregar 4,7 kΩ a 3V3 en SDA y SCL. Alimentar el AS5600 con 3V3, no con 5 V.
//* Finales de carrera: los DOS topes de cada eje van en SERIE, contactos NC (normalmente
//* cerrados), entre el GPIO y GND. En reposo el pin lee LOW. Si cualquiera de los dos topes
//* se abre, o si se corta un cable, lee HIGH = tope activado. Esto es a prueba de fallas:
//* un cable suelto detiene el eje en vez de ocultar un choque. Qué tope se tocó se sabe por θ_med.
//todo El corte de nENABLE por hardware desde los topes (EA-5) queda para la PCB. Por ahora,
//todo sólo lectura por software (reacción en ≤ FIN_ANTIRREBOTE_MS + 1 ms).

constexpr uint8_t PIN_M0 = 32;
constexpr uint8_t PIN_M1 = 33;
constexpr uint8_t PIN_M2 = 14;

constexpr bool    USAR_FINES_CARRERA = true;   //! false sólo si los topes no están cableados
constexpr bool    FIN_CONTACTO_NC    = true;   // NC en serie: activo = pin en ALTO
constexpr uint8_t FIN_ANTIRREBOTE_MS = 3;      // lecturas seguidas (a 1 kHz) para aceptar el tope

// ============================================================================
//* MECÁNICA Y ACCIONAMIENTO
// ============================================================================
constexpr float   PASO_COMPLETO_DEG = 1.8f;   // motor de 200 pasos/vuelta
constexpr uint8_t MICROPASOS        = 16;     // 1/16 -> 0,1125 °/micropaso en el motor
constexpr float   GRADOS_POR_MICROPASO = PASO_COMPLETO_DEG / MICROPASOS;

//* Generador de pulsos: un solo temporizador de hardware a F_TICK para ambos ejes.
//* Cada tic, un acumulador de fase por eje decide si corresponde emitir un micropaso
//* (oscilador numérico, DDA). El pulso STEP dura exactamente un tic.
constexpr uint32_t F_TICK_HZ        = 40000;                // período de 25 µs
constexpr float    PASOS_S_MAX      = F_TICK_HZ / 4.0f;     // 10 000 micropasos/s, techo del generador
//? El DRV8825 pide pulsos de 1,9 µs como mínimo, en alto y en bajo. Con 25 µs sobra margen.

// ============================================================================
//* LAZO INTERNO (CS-10) — ley de control de pid_t_2.pdf
// ============================================================================
//*   ω_cmd = ω_ff + Kp·e + Ki·∫e dt,   e = θ_ref − θ_med   (sensor con H(s) = 1)
//*   saturación |ω_cmd| ≤ ω_max  y  límite de aceleración |dω_cmd/dt| ≤ α_max
//*   f_STEP = ω_cmd · relación / Δθ_p
//* Estabilidad (Routh, pid_t_2 §5): Kp < b/M. Con ζ = 0,02 / 0,05 / 0,10 el techo de pan es
//* 5,7 / 14,2 / 28,4 1/s, y el de tilt 55 / 137 / 273 1/s. b todavía NO está medido.
constexpr uint32_t F_CONTROL_HZ   = 1000;
constexpr uint32_t PERIODO_CTL_US = 1000000UL / F_CONTROL_HZ;

constexpr float OMEGA_MAX_DPS  = 180.0f;   // ω_max = 3,14 rad/s (decisión de diseño)
constexpr float ALFA_MAX_DPS2  = 300.0f;   // α_max = 5,24 rad/s² (decisión de diseño)
//* Dos agregados a la ley lineal, que sólo actúan con errores grandes (revisión del 21-10):
//*   - tope de frenado del término P, √(2·α_max·|e|): sin él, el límite de α hacía pasarse
//*     27,8° a tilt en 60°;
//*   - separación del integrador: sólo integra con |e| < BANDA_INTEGRAL_DEG (sobrepaso de pan).
//* Simulado: el peor sobrepaso baja a 2,3° (escalones de 2 a 60°, ambos ejes y ambos juegos de
//* ganancias). La prueba del modelo LINEAL de pid_t_2: 'a <eje> 0' y escalones de 2–4°.
constexpr float BANDA_INTEGRAL_DEG = 5.0f;

//* Feedforward: entre dos consignas de la RPi (80 fps -> 12,5 ms) la referencia se
//* extrapola con ω_ff durante como máximo este tiempo. Después queda congelada.
constexpr float EXTRAPOLACION_MAX_S = 0.050f;

// ============================================================================
//* CONFIGURACIÓN POR EJE
// ============================================================================
struct ConfigEje {
  const char* nombre;
  uint8_t pinStep, pinDir, pinEn, pinFault, pinFin;
  uint8_t pinSda, pinScl;
  uint8_t busI2C;            // 0 = Wire, 1 = Wire1
  int8_t  signoMotor;        // +1 / −1: sentido de DIR que hace crecer θ
  int8_t  signoSensor;       // +1 / −1: sentido del AS5600 que hace crecer θ
  float   relacion;          // vueltas del motor por vuelta del eje (correa 1:1 -> 1,0)
  int16_t ceroRaw;           // cuenta cruda del AS5600 para θ = 0; −1 = tomarla al encender
  float   limMinDeg, limMaxDeg;   // límites por software de la referencia
  float   kp, ki;            // [1/s], [1/s²]
};

//? Ganancias de ARRANQUE en laboratorio, más bajas que las de pid_t_2.pdf (§8: pan Kp=5,
//? Ki=6,25; tilt Kp=20, Ki=100). Si ζ real ≈ 0,02, el techo de pan es 5,7 1/s y Kp = 5
//? queda casi en el límite. Subir con 'k' por consola recién cuando la prueba de escalón no
//? muestre oscilación a la frecuencia de resonancia (~22 Hz en pan).
//todo Medir el recorrido mecánico libre de cada eje y ajustar limMinDeg / limMaxDeg.
//todo Los límites por software deben quedar ANTES de los topes físicos (≥ MARGEN_LIMITE_DEG).
//todo Tras la prueba 't' de la consola, corregir signoMotor y signoSensor si hace falta.
//todo Tras centrar la plataforma, fijar ceroRaw con el valor que imprime el comando 'z'.
constexpr ConfigEje EJES[2] = {
  //   nombre  STEP DIR EN FAULT FIN  SDA SCL bus  sgnM sgnS  rel   cero   limMin  limMax   Kp    Ki
  {   "pan",    25, 26, 27,  34,  13,  21, 22,  0,   +1,  +1, 1.0f,   -1,  -60.0f, +60.0f, 3.0f,  2.25f },
  {   "tilt",   17, 16,  4,  35,  23,  18, 19,  1,   +1,  +1, 1.0f,   -1,  -30.0f, +30.0f, 10.0f, 25.0f },
};
constexpr uint8_t EJE_PAN  = 0;
constexpr uint8_t EJE_TILT = 1;
constexpr uint8_t N_EJES   = 2;

//? El AS5600 se supone montado en el EJE (después de la correa), como en el diagrama de
//? control. Si quedara en el eje del motor, θ_med debe dividirse por la relación.

// ============================================================================
//* SENSOR AS5600
// ============================================================================
constexpr uint32_t I2C_HZ            = 400000;  // modo rápido; el AS5600 admite hasta 1 MHz
constexpr uint16_t I2C_TIMEOUT_MS    = 2;       //! sin esto, un sensor desconectado congela 50 ms el lazo
//* Filtro lento interno del AS5600 (CONF.SF): 0=16x (2,2 ms), 1=8x (1,1 ms), 2=4x (0,55 ms),
//* 3=2x (0,286 ms). Con 4x el ruido es 0,029° RMS, menor que 1 LSB (0,088°), y el retardo
//* del filtro queda cerca de medio período de control.
constexpr uint8_t  AS5600_SF          = 2;
constexpr uint8_t  ERRORES_I2C_FALLA  = 5;       // lecturas fallidas seguidas -> falla de sensor
constexpr uint8_t  IMAN_AUSENTE_FALLA = 3;       // revisiones seguidas (a 10 Hz) con MD = 0 -> falla
constexpr uint32_t RETARDO_FILTRO_AS5600_US = 275;   // ≈ mitad del asentamiento con SF = 4x

// ============================================================================
//* SUPERVISIÓN LOCAL (CS-9)
// ============================================================================
constexpr bool  USAR_PIN_FAULT       = true;    //! poner false si nFAULT no está cableado (evita fallas falsas)
constexpr float MARGEN_LIMITE_DEG    = 5.0f;    // θ_med fuera de [lím − margen, lím + margen] -> falla
constexpr float UMBRAL_PERDIDA_DEG   = 2.0f;    // |θ_pasos − θ_med| > umbral -> pérdida de pasos (ACT-5)
constexpr uint32_t RESINCRONIA_HABILITAR_MS = 200;  // tras habilitar, el rotor se asienta: no evaluar pérdida
//? Con un signo mal configurado el PI realimenta positivo; el detector de pérdida de pasos
//? lo corta en cuanto la diferencia supera el umbral.

// ============================================================================
//* ENLACE CON LA RASPBERRY PI (CS-4)
// ============================================================================
constexpr uint32_t BAUDIOS              = 921600;
constexpr uint32_t WATCHDOG_ENLACE_MS   = 100;    // sin consigna -> RETENCIÓN (diagrama CS-3/CS-4)
constexpr uint32_t DESHABILITAR_SIN_ENLACE_MS = 5000;  // retención por watchdog más larga -> DESHABILITADO
constexpr uint32_t ENLACE_ACTIVO_MS     = 1000;   // tras la última trama válida se sigue en modo binario
constexpr uint32_t PERIODO_TELEMETRIA_US = 5000;  // 200 Hz

// ============================================================================
//* CAPTURA DE RESPUESTA AL ESCALÓN (CTL-7)
// ============================================================================
constexpr uint16_t CAPTURA_N = 3000;            // muestras (≈ 60 kB de RAM)
constexpr uint8_t  CAPTURA_DECIMACION = 2;      // una muestra cada 2 ms -> 6 s de captura
//* 6 s porque pan, con las ganancias de arranque, tarda más de 3 s en asentarse. A 500 Hz la
//* resonancia de pan (~23 Hz) sigue bien resuelta.
