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
//* PINES — ESP32 DevKit (WROOM-32) sobre la placa de expansión (cada GPIO con S, V, G)
// ============================================================================
//* Cableado REAL del laboratorio (22-10-2026). "S" = señal del GPIO, "V" y "G" = alimentación
//* y tierra de la misma fila de la placa de expansión.
//
//    Eje      Función             GPIO   Canal
//    -------  ------------------  -----  -----
//    pan      nENABLE  DRV8825     13     S
//    (azimut) STEP                 12     S     //! GPIO12 es pin de arranque (MTDI): ver abajo
//             DIR                  14     S
//    tilt     nENABLE  DRV8825     27     S
//    (elev.)  STEP                 26     S
//             DIR                  25     S
//    pan      AS5600 SDA           21     S     bus I2C 0 ("Wire")
//             AS5600 SCL           22     S
//             AS5600 VCC           (23)   V     sólo alimentación desde la fila V del GPIO23
//    tilt     AS5600 SDA           18     S     bus I2C 1 ("Wire1")
//             AS5600 SCL           19     S
//             AS5600 VCC           (5)    V     sólo alimentación desde la fila V del GPIO5
//    enlace   RX0 <- TX de la RPi  3      S     pin 8 del header de la RPi (GPIO14, TXD)
//             TX0 -> RX de la RPi  1      S     pin 10 del header de la RPi (GPIO15, RXD)
//             GND común            (23)   G     debe ir a un pin GND de la RPi (6, 9, 14...)
//
//* Sin cablear por ahora: nFAULT, finales de carrera y M0/M1/M2 (micropasos). Por eso
//* USAR_PIN_FAULT = false, USAR_FINES_CARRERA = false y USAR_PINES_MICROPASO = false.
//! Los micropasos los fija el HARDWARE del driver (pines M0..M2 o interruptores de la base).
//! El firmware tiene que saber cuál es: comando 'm' de la consola (queda guardado en la
//! memoria del ESP32) y la prueba 't' lo estima. Con M0..M2 sueltos el DRV8825 trabaja en
//! paso completo (1/1). Para 1/16 basta llevar M2 a 3V3 y dejar M0 y M1 a GND o sueltos.
//! GPIO12 (STEP pan): si está en ALTO al arrancar, el ESP32 elige flash a 1,8 V y no arranca.
//! El STEP del DRV8825 tiene pull-down interno, así que funciona; no agregar pull-up ahí.
//! Las filas V de la placa de expansión pueden ser 5 V o 3,3 V (puente en la placa). El
//! AS5600 debe ir a 3,3 V: con 5 V, los pull-ups del módulo llevan SDA/SCL a 5 V y el ESP32
//! no los tolera. Medir V–G con el multímetro antes de conectar.
//! nENABLE sin pull-up externo: mientras el ESP32 arranca, el pin flota y el DRV8825 (pull-down
//! interno en nENABLE) queda HABILITADO con corriente de retención, sin pasos. No se mueve,
//! pero calienta. Se deshabilita en cuanto corre setup().
//* Cada AS5600 va en su propio bus I2C (dirección fija 0x36): sin multiplexor.
//* Libres: GPIO16 (RX2) y GPIO17 (TX2). Si el enlace por RX0/TX0 da problemas (ver ENLACE más
//* abajo), mover los dos cables de la RPi a GPIO16/17: el firmware escucha en ambos.

constexpr uint8_t PIN_NC = 255;                // "no conectado"

constexpr bool    USAR_PINES_MICROPASO = false;  // M0..M2 no van al ESP32
constexpr uint8_t PIN_M0 = PIN_NC;
constexpr uint8_t PIN_M1 = PIN_NC;
constexpr uint8_t PIN_M2 = PIN_NC;

constexpr bool    USAR_FINES_CARRERA = false;  //! topes no cableados todavía
constexpr bool    FIN_CONTACTO_NC    = true;   // NC en serie: activo = pin en ALTO
constexpr uint8_t FIN_ANTIRREBOTE_MS = 3;      // lecturas seguidas (a 1 kHz) para aceptar el tope

// ============================================================================
//* MECÁNICA Y ACCIONAMIENTO
// ============================================================================
constexpr float   PASO_COMPLETO_DEG = 1.8f;   // motor de 200 pasos/vuelta
//* Micropasos que tiene configurados el DRV8825 (1, 2, 4, 8, 16 o 32). Es un valor de
//* ARRANQUE: el comando 'm <N>' lo cambia y lo guarda en la memoria no volátil del ESP32.
//! Por defecto 1 (paso completo), que es lo que hace el DRV8825 con M0..M2 sueltos. Además es
//! lo seguro si no se sabe: si el driver estuviera en 1/16, el eje se movería 16 veces MENOS
//! de lo pedido. Al revés (firmware en 16 y driver en 1/1) se movería 16 veces MÁS.
constexpr uint8_t MICROPASOS_POR_DEFECTO = 1;

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
//* signoMotor, signoSensor y ceroRaw son valores de ARRANQUE: los comandos 'i' y 'z' de la
//* consola los corrigen y los guardan en la memoria del ESP32 (no hace falta recompilar).
//todo Cuando estén medidos, copiarlos aquí también (los imprime 'p'), para no depender de la memoria.
constexpr ConfigEje EJES[2] = {
  //   nombre  STEP DIR EN FAULT FIN  SDA SCL bus  sgnM sgnS  rel   cero   limMin  limMax   Kp    Ki
  {   "pan",    12, 14, 13, PIN_NC, PIN_NC, 21, 22, 0, +1,  +1, 1.0f,   -1,  -60.0f, +60.0f, 3.0f,  2.25f },
  {   "tilt",   26, 25, 27, PIN_NC, PIN_NC, 18, 19, 1, +1,  +1, 1.0f,   -1,  -30.0f, +30.0f, 10.0f, 25.0f },
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
constexpr bool  USAR_PIN_FAULT       = false;   //! nFAULT no cableado: si se cablea, poner true y su GPIO en EJES
constexpr float MARGEN_LIMITE_DEG    = 5.0f;    // θ_med fuera de [lím − margen, lím + margen] -> falla
constexpr float UMBRAL_PERDIDA_DEG   = 2.0f;    // |θ_pasos − θ_med| > umbral -> pérdida de pasos (ACT-5)
//* Con pasos gruesos (1/1 = 1,8°) el umbral efectivo sube a 2,5 pasos para no dar fallas falsas.
constexpr float UMBRAL_PERDIDA_PASOS = 2.5f;
constexpr uint32_t RESINCRONIA_HABILITAR_MS = 200;  // tras habilitar, el rotor se asienta: no evaluar pérdida
//? Con un signo mal configurado el PI realimenta positivo; el detector de pérdida de pasos
//? lo corta en cuanto la diferencia supera el umbral.

// ============================================================================
//* ENLACE CON LA RASPBERRY PI (CS-4)
// ============================================================================
//* El firmware escucha tramas en DOS UART y responde por el que recibió la última trama válida:
//*   - UART0 (RX0 = GPIO3, TX0 = GPIO1): el cableado actual a la RPi. Es el MISMO UART del
//*     USB, así que la consola por USB y la RPi comparten la línea.
//*   - UART2 (RX2 = GPIO16, TX2 = GPIO17): alternativa recomendada. Deja el USB libre para la
//*     consola y para cargar firmware con la RPi conectada.
//! Con la RPi en RX0/TX0, NO conectar el USB del ESP32 a un PC o a la RPi mientras corre el
//! lazo: el chip USB-serie también maneja RX0 y choca con el TX de la RPi. Alimentar el ESP32
//! por la placa de expansión (DC o 5V). Para cargar firmware por USB, desconectar el cable RX0.
constexpr bool    ESCUCHAR_UART2 = true;
constexpr uint8_t PIN_RX2 = 16;
constexpr uint8_t PIN_TX2 = 17;
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
