/*
 * esp32_pan_tilt — controlador de movimiento de la plataforma pan-tilt (IEE2913, Grupo 9)
 * --------------------------------------------------------------------------------------
 * Lazo INTERNO del diagrama de control (CS-4 a CS-10), uno por eje:
 *
 *   θ_ref (RPi o consola) ─►(+)─ e_θ ─► PI pid_t_2 ─ ω_cmd ─► generador STEP/DIR ─► DRV8825 ─► NEMA 17
 *                           ▲(−)                                                              │
 *                           └──────────────── θ_med ◄── AS5600 (I2C) ◄────────────────────────┘
 *
 * La Raspberry Pi cierra el lazo EXTERNO (cámara) y manda θ_ref absoluto por eje, más un
 * feedforward de velocidad ω_ff, por trama binaria a 921 600 baudios. Este firmware:
 *   - lee ambos AS5600 y corre el PI a 1 kHz;
 *   - genera los trenes de pulsos de los dos ejes con un temporizador de hardware;
 *   - supervisa localmente (CS-9): nFAULT, sensor, límites, pérdida de pasos y watchdog
 *     del enlace, y reacciona en el mismo período de control (< 1 ms);
 *   - manda telemetría a 200 Hz mientras el enlace está activo;
 *   - atiende una consola de texto para puesta en marcha (signos, cero, escalones).
 *
 * Archivos: config.h (todos los números), generador_pasos, as5600, control_eje, protocolo.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "generador_pasos.h"
#include "as5600.h"
#include "control_eje.h"
#include "protocolo.h"

// ============================================================================
//* ESTADOS, FALLAS Y DATOS POR EJE
// ============================================================================
//* El número de cada estado viaja en la telemetría (pt/enlace.py usa los mismos valores).
enum Estado : uint8_t { DESHABILITADO = 0, RETENCION = 1, LAZO = 2, ABIERTO = 3, FALLA = 4 };
static const char* const NOMBRE_ESTADO[] = {"DESHABILITADO", "RETENCION", "LAZO", "ABIERTO", "FALLA"};

enum : uint8_t {
  F_DRIVER_PAN = 0x01, F_DRIVER_TILT = 0x02,
  F_SENSOR_PAN = 0x04, F_SENSOR_TILT = 0x08,
  F_LIMITE_PAN = 0x10, F_LIMITE_TILT = 0x20,
  F_PASOS      = 0x40,
  F_WATCHDOG   = 0x80,   // informativa: no se enclava, se limpia con la próxima consigna
};
constexpr uint8_t FALLAS_ENCLAVADAS = 0x7F;
static inline uint8_t bitDriver(uint8_t e) { return e == EJE_PAN ? F_DRIVER_PAN : F_DRIVER_TILT; }
static inline uint8_t bitSensor(uint8_t e) { return e == EJE_PAN ? F_SENSOR_PAN : F_SENSOR_TILT; }
static inline uint8_t bitLimite(uint8_t e) { return e == EJE_PAN ? F_LIMITE_PAN : F_LIMITE_TILT; }

enum Fuente : uint8_t { CONSOLA = 0, ENLACE = 1 };

struct EstadoEje {
  float    thetaMed     = 0.0f;  // θ_med [°]
  float    ref          = 0.0f;  // referencia recibida [°]
  float    omegaFF      = 0.0f;  // feedforward recibido [°/s]
  uint32_t tRefUs       = 0;     // micros() al recibir la referencia
  float    refEff       = 0.0f;  // referencia aplicada: extrapolada y limitada [°]
  float    omegaCmd     = 0.0f;  // salida aplicada al generador [°/s]
  float    omegaAbierto = 0.0f;  // consigna de lazo abierto (diagnóstico) [°/s]
  float    offsetPasos  = 0.0f;  // θ_pasos = offset + k·posición
  float    thetaPasos   = 0.0f;  // ángulo que "creen" los pasos emitidos [°]
};

static EstadoEje    ej[N_EJES];
static SensorAS5600 sensor[N_EJES];
static ControlEje   ctl[N_EJES];

static Estado   estado = DESHABILITADO;
static Fuente   fuente = CONSOLA;
static uint8_t  fallas = 0;
static char     motivoFalla[72] = "";

// Enlace
static proto::Receptor rx;
static uint32_t tUltTramaMs = 0, tUltConsignaMs = 0;
static bool     huboTrama = false;
static uint8_t  seqConsigna = 0, seqTx = 0;

// Tiempos del lazo
static uint32_t tUltCtlUs = 0, tUltTelemetriaUs = 0, tUltImanMs = 0;
static uint32_t tMedUs = 0;                 // instante de la última lectura de sensores (telemetría)
static uint32_t tHabilitadoMs = 0;          // cuándo se energizaron los drivers
static uint8_t  sinIman[N_EJES] = {0, 0};   // revisiones seguidas con MD = 0
static uint8_t  finCuenta[N_EJES] = {0, 0}; // ms seguidos con el final de carrera activo

//* Final de carrera activo (NC en serie: pin en ALTO = algún tope abierto o cable cortado).
static inline bool finActivo(uint8_t e) {
  return USAR_FINES_CARRERA && (digitalRead(EJES[e].pinFin) == (FIN_CONTACTO_NC ? HIGH : LOW));
}

// Captura de escalón (CTL-7)
struct Muestra { uint32_t t_us; float ref, med, omega, pasos; };
static Muestra  captura[CAPTURA_N];
static uint16_t capturaN = 0;
static bool     capturando = false;
static uint8_t  capturaEje = 0;
static float    capturaKp = 0, capturaKi = 0, capturaEscalon = 0, capturaAlfa = 0;
static uint32_t capturaT0 = 0;

static TwoWire& bus(uint8_t e) { return EJES[e].busI2C == 0 ? Wire : Wire1; }

// ============================================================================
//* UTILIDADES
// ============================================================================
static inline float limitar(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

//* Factor micropasos -> grados del eje (con signo del motor y relación de transmisión).
static inline float gradosPorPasoEje(uint8_t e) {
  return EJES[e].signoMotor * GRADOS_POR_MICROPASO / EJES[e].relacion;
}

//* ω del eje [°/s] -> micropasos/s con signo, para el generador.
static inline float omegaAPasos(uint8_t e, float omega) {
  return EJES[e].signoMotor * omega * EJES[e].relacion / GRADOS_POR_MICROPASO;
}

static bool enlaceActivo() {
  return huboTrama && (millis() - tUltTramaMs) < ENLACE_ACTIVO_MS;
}

//* Mensajes de texto: empiezan con '#' para que la RPi los separe de las tramas.
//! Sólo ASCII: un byte 0xAA dentro de un texto UTF-8 podría parecer inicio de trama.
static void aviso(const char* fmt, ...) {
  char buf[160];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print("# "); Serial.println(buf);
}

static void sincronizarPasos(uint8_t e) {
  ej[e].offsetPasos = ej[e].thetaMed - gradosPorPasoEje(e) * (float)pasos::posicion(e);
  ej[e].thetaPasos  = ej[e].thetaMed;
}

//* Mantener la posición actual sin salto de velocidad: la referencia pasa a θ_med y el
//* PI parte desde la ω actual, así que frena con la rampa α_max en vez de en seco.
static void retenerEje(uint8_t e) {
  ej[e].ref = ej[e].thetaMed;
  ej[e].omegaFF = 0.0f;
  ej[e].tRefUs = micros();
  ctl[e].reiniciar(ej[e].omegaCmd);
}

// ============================================================================
//* TRANSICIONES DE ESTADO
// ============================================================================
static void irA(Estado nuevo, const char* motivo) {
  if (nuevo == estado) return;
  const Estado previo = estado;
  estado = nuevo;

  for (uint8_t e = 0; e < N_EJES; e++) {
    switch (nuevo) {
      case DESHABILITADO:
      case FALLA:
        pasos::habilitar(e, false);
        ej[e].omegaCmd = 0.0f;
        ej[e].omegaAbierto = 0.0f;
        ctl[e].reiniciar(0.0f);
        break;
      case RETENCION:
      case LAZO:
      case ABIERTO:
        if (!pasos::habilitado(e)) {               // venía deshabilitado: partir de reposo
          pasos::habilitar(e, true);
          tHabilitadoMs = millis();                //* ver RESINCRONIA_HABILITAR_MS en pasoControl
          sincronizarPasos(e);
          ej[e].omegaCmd = 0.0f;
          ctl[e].reiniciar(0.0f);
          retenerEje(e);
        } else if (nuevo == RETENCION) {
          retenerEje(e);
        } else if (nuevo == LAZO && previo == ABIERTO) {
          ctl[e].reiniciar(ej[e].omegaCmd);        // sin salto al cerrar el lazo
        }
        if (nuevo == ABIERTO) ej[e].omegaAbierto = 0.0f;
        break;
    }
  }
  aviso("estado %s -> %s (%s)", NOMBRE_ESTADO[previo], NOMBRE_ESTADO[nuevo], motivo);
}

static void declararFalla(uint8_t bits, const char* fmt, ...) {
  const uint8_t nuevas = bits & ~fallas;
  fallas |= bits;
  if (nuevas) {
    va_list ap; va_start(ap, fmt); vsnprintf(motivoFalla, sizeof(motivoFalla), fmt, ap); va_end(ap);
    aviso("FALLA 0x%02X: %s", fallas, motivoFalla);
  }
  if (estado != FALLA) irA(FALLA, motivoFalla);
}

//* Rearme manual (D21): sólo si las causas ya no están presentes.
static bool rearmarYHabilitar() {
  for (uint8_t e = 0; e < N_EJES; e++) {
    if (!sensor[e].leer() || sensor[e].revisarIman() != 1 || !sensor[e].referenciado()) {
      aviso("no se habilita: sensor %s sin lectura, sin iman o sin cero (status 0x%02X)", EJES[e].nombre, sensor[e].estado());
      return false;
    }
    ej[e].thetaMed = sensor[e].gradosEje();
    if (USAR_PIN_FAULT && digitalRead(EJES[e].pinFault) == LOW) {
      aviso("no se habilita: nFAULT activo en %s", EJES[e].nombre);
      return false;
    }
    if (finActivo(e)) {
      aviso("no se habilita: final de carrera de %s activo (o cable cortado). Alejar el eje a mano.", EJES[e].nombre);
      return false;
    }
    const ConfigEje& c = EJES[e];
    if (ej[e].thetaMed < c.limMinDeg - MARGEN_LIMITE_DEG || ej[e].thetaMed > c.limMaxDeg + MARGEN_LIMITE_DEG) {
      aviso("no se habilita: %s fuera de limites (%.2f deg). Moverlo a mano.", c.nombre, ej[e].thetaMed);
      return false;
    }
  }
  fallas = 0;
  motivoFalla[0] = 0;
  sinIman[0] = sinIman[1] = 0;
  finCuenta[0] = finCuenta[1] = 0;
  if (estado == FALLA) estado = DESHABILITADO;     // así irA() habilita desde reposo
  irA(RETENCION, "habilitado");
  return true;
}

// ============================================================================
//* LAZO DE CONTROL Y SUPERVISIÓN — un período (1 ms)
// ============================================================================
static void pasoControl(float dt, uint32_t ahoraUs) {
  const bool energizado = (estado == RETENCION || estado == LAZO || estado == ABIERTO);

  // ---- Sensado y supervisión local (CS-9) ----------------------------------------------
  //* Marca de tiempo de la medición para la telemetría: ahora, menos el retardo del filtro
  //* interno del AS5600 (con SF = 4x se estabiliza en 0,55 ms; se toma la mitad como retardo).
  tMedUs = ahoraUs - RETARDO_FILTRO_AS5600_US;
  for (uint8_t e = 0; e < N_EJES; e++) {
    if (!sensor[e].leer() && sensor[e].erroresSeguidos() >= ERRORES_I2C_FALLA) {
      declararFalla(bitSensor(e), "sensor %s sin respuesta I2C", EJES[e].nombre);
    }
    ej[e].thetaMed = sensor[e].gradosEje();

    if (USAR_PIN_FAULT && digitalRead(EJES[e].pinFault) == LOW) {
      declararFalla(bitDriver(e), "nFAULT del DRV8825 de %s (sobrecorriente/termico/UVLO)", EJES[e].nombre);
    }
  }
  if (millis() - tUltImanMs >= 100) {              // el imán se revisa a 10 Hz
    tUltImanMs = millis();
    for (uint8_t e = 0; e < N_EJES; e++) {
      //* Un error de lectura del STATUS NO cuenta como "sin imán" (lo cubren los errores I2C).
      //* Hacen falta IMAN_AUSENTE_FALLA revisiones seguidas con MD = 0 para declarar la falla.
      const int8_t r = sensor[e].revisarIman();
      if (r == 0) {
        if (++sinIman[e] >= IMAN_AUSENTE_FALLA) {
          declararFalla(bitSensor(e), "iman no detectado en %s (status 0x%02X)", EJES[e].nombre, sensor[e].estado());
        }
      } else if (r == 1) {
        sinIman[e] = 0;
      }
    }
  }

  for (uint8_t e = 0; e < N_EJES; e++) {
    const ConfigEje& c = EJES[e];
    ej[e].thetaPasos = ej[e].offsetPasos + gradosPorPasoEje(e) * (float)pasos::posicion(e);
    //* Deshabilitado, el eje se puede mover a mano: el conteo de pasos sigue al sensor.
    //* Recién habilitado, el rotor salta a la posición de micropaso que retiene el DRV8825
    //* (hasta ±1,8° si se movió a mano): se resincroniza durante RESINCRONIA_HABILITAR_MS
    //* para que ese salto no se lea como pérdida de pasos.
    if (!energizado || (millis() - tHabilitadoMs) < RESINCRONIA_HABILITAR_MS) {
      sincronizarPasos(e);
      if (!energizado) continue;
    }
    if (ej[e].thetaMed < c.limMinDeg - MARGEN_LIMITE_DEG || ej[e].thetaMed > c.limMaxDeg + MARGEN_LIMITE_DEG) {
      declararFalla(bitLimite(e), "%s fuera de limites: %.2f deg", c.nombre, ej[e].thetaMed);
    }
    //* Final de carrera físico: misma falla de límite (bit compartido), con su propio mensaje.
    finCuenta[e] = finActivo(e) ? (uint8_t)(finCuenta[e] < 255 ? finCuenta[e] + 1 : 255) : 0;
    if (finCuenta[e] >= FIN_ANTIRREBOTE_MS) {
      declararFalla(bitLimite(e), "FINAL DE CARRERA %s (theta %.2f deg: tope %s)", c.nombre,
                    ej[e].thetaMed, ej[e].thetaMed >= 0.0f ? "positivo" : "negativo");
    }
    //* Pérdida de pasos (ACT-5): lo que dicen los pasos emitidos contra lo que mide el sensor.
    if (fabsf(ej[e].thetaPasos - ej[e].thetaMed) > UMBRAL_PERDIDA_DEG) {
      declararFalla(F_PASOS, "perdida de pasos en %s: pasos %.2f vs sensor %.2f deg",
                    c.nombre, ej[e].thetaPasos, ej[e].thetaMed);
    }
  }

  // ---- Watchdog del enlace -------------------------------------------------------------
  if (estado == LAZO && fuente == ENLACE && (millis() - tUltConsignaMs) > WATCHDOG_ENLACE_MS) {
    fallas |= F_WATCHDOG;
    irA(RETENCION, "watchdog: sin consigna de la RPi");
  }
  //! Retención por watchdog prolongada (RPi caída): se deshabilitan los drivers.
  if (estado == RETENCION && (fallas & F_WATCHDOG) && (millis() - tUltConsignaMs) > DESHABILITAR_SIN_ENLACE_MS) {
    irA(DESHABILITADO, "sin enlace con la RPi");
  }

  // ---- Ley de control por eje ----------------------------------------------------------
  for (uint8_t e = 0; e < N_EJES; e++) {
    const ConfigEje& c = EJES[e];
    float omega = 0.0f;

    if (estado == RETENCION || estado == LAZO) {
      //* Entre consignas, la referencia avanza con ω_ff durante a lo más EXTRAPOLACION_MAX_S.
      const float edad = (float)(ahoraUs - ej[e].tRefUs) * 1e-6f;
      const bool extrapola = edad < EXTRAPOLACION_MAX_S;
      float ref = ej[e].ref + (extrapola ? ej[e].omegaFF * edad : ej[e].omegaFF * EXTRAPOLACION_MAX_S);
      float ff  = extrapola ? ej[e].omegaFF : 0.0f;
      const float refLim = limitar(ref, c.limMinDeg, c.limMaxDeg);
      if (refLim != ref) ff = 0.0f;                //! en el límite no se empuja con feedforward
      ej[e].refEff = refLim;
      omega = ctl[e].paso(refLim, ej[e].thetaMed, ff, dt);
    } else if (estado == ABIERTO) {
      const float dMax = ALFA_MAX_DPS2 * dt;       // lazo abierto también respeta α_max
      omega = ej[e].omegaCmd + limitar(ej[e].omegaAbierto - ej[e].omegaCmd, -dMax, dMax);
      ej[e].refEff = ej[e].thetaMed;
    } else {
      ej[e].refEff = ej[e].thetaMed;
    }

    ej[e].omegaCmd = omega;
    pasos::fijarVelocidad(e, omegaAPasos(e, omega));
  }

  // ---- Captura de la respuesta al escalón ---------------------------------------------
  static uint8_t decim = 0;
  if (capturando && ++decim >= CAPTURA_DECIMACION) {
    decim = 0;
    const uint8_t e = capturaEje;
    captura[capturaN++] = {ahoraUs - capturaT0, ej[e].refEff, ej[e].thetaMed, ej[e].omegaCmd, ej[e].thetaPasos};
    if (capturaN >= CAPTURA_N) {
      capturando = false;
      aviso("captura lista (%u muestras). Escribe 'd' para descargarla.", capturaN);
    }
  }
}

// ============================================================================
//* TELEMETRÍA (ESP32 -> RPi)
// ============================================================================
static void enviarTelemetria(uint32_t tUs) {
  uint8_t c[proto::LARGO_TELEMETRIA];
  proto::ponerU32(c + 0, tUs);
  proto::ponerF32(c + 4,  ej[EJE_PAN].thetaMed);
  proto::ponerF32(c + 8,  ej[EJE_TILT].thetaMed);
  proto::ponerF32(c + 12, ej[EJE_PAN].refEff);
  proto::ponerF32(c + 16, ej[EJE_TILT].refEff);
  proto::ponerF32(c + 20, ej[EJE_PAN].omegaCmd);
  proto::ponerF32(c + 24, ej[EJE_TILT].omegaCmd);
  c[28] = (uint8_t)estado;
  c[29] = fallas;
  c[30] = seqConsigna;
  uint8_t trama[7 + proto::LARGO_TELEMETRIA];
  const size_t n = proto::armar(trama, seqTx++, proto::TELEMETRIA, c, sizeof(c));
  Serial.write(trama, n);
}

// ============================================================================
//* TRAMAS RECIBIDAS (RPi -> ESP32)
// ============================================================================
static void procesarTrama() {
  tUltTramaMs = millis();
  huboTrama = true;

  if (rx.tipo == proto::CONSIGNA_THETA && rx.largo == proto::LARGO_CONSIGNA_THETA) {
    float v[4];
    for (uint8_t i = 0; i < 4; i++) v[i] = proto::leerF32(rx.carga + 4 * i);
    for (uint8_t i = 0; i < 4; i++) if (!isfinite(v[i])) return;     //! NaN/inf: se ignora
    tUltConsignaMs = millis();
    seqConsigna = rx.seq;
    if (estado == RETENCION || estado == LAZO || estado == ABIERTO) {
      const uint32_t ahora = micros();
      for (uint8_t e = 0; e < N_EJES; e++) {
        ej[e].ref = v[e];
        ej[e].omegaFF = limitar(v[2 + e], -OMEGA_MAX_DPS, OMEGA_MAX_DPS);
        ej[e].tRefUs = ahora;
      }
      fuente = ENLACE;
      fallas &= ~F_WATCHDOG;
      if (estado != LAZO) irA(LAZO, "consigna del enlace");
    }
    //* En DESHABILITADO o FALLA las consignas se ignoran: hace falta CMD_HABILITAR.
  } else if (rx.tipo == proto::COMANDO && rx.largo == 1) {
    switch (rx.carga[0]) {
      case proto::CMD_HABILITAR:    rearmarYHabilitar(); break;
      case proto::CMD_DESHABILITAR: irA(DESHABILITADO, "comando del enlace"); break;
      case proto::CMD_RETENER:      if (estado == LAZO || estado == ABIERTO) irA(RETENCION, "comando del enlace"); break;
      case proto::CMD_CERO:
        if (estado == DESHABILITADO) for (uint8_t e = 0; e < N_EJES; e++) sensor[e].ponerCero();
        break;
      default: break;
    }
  }
}

// ============================================================================
//* CONSOLA DE DIAGNÓSTICO (texto, puesta en marcha)
// ============================================================================
static void imprimirAyuda() {
  Serial.println(F(
    "# Consola esp32_pan_tilt. Ejes: 0|pan, 1|tilt. Angulos en grados del eje.\n"
    "#   e 1 | e 0        habilitar (y rearmar fallas) | deshabilitar\n"
    "#   x                parada inmediata y deshabilitar\n"
    "#   s                retener la posicion actual\n"
    "#   r <eje> <deg>    referencia ABSOLUTA en lazo cerrado (PI)\n"
    "#   g <eje> <deg>    ESCALON relativo + captura de 6 s (1 muestra cada 2 ms)\n"
    "#   d                descargar la captura (CSV)\n"
    "#   k <eje> <Kp> <Ki> cambiar ganancias en caliente\n"
    "#   a <eje> <deg/s2> limite de aceleracion del PI (0 = sin limite, solo escalones chicos)\n"
    "#   v <eje> <deg/s>  velocidad en LAZO ABIERTO (diagnostico)\n"
    "#   t <eje>          prueba de signos: +3 deg y vuelta (debe estar en RETENCION)\n"
    "#   z                cero en la posicion actual (deshabilitado); imprime ceroRaw\n"
    "#   p                estado        ?  esta ayuda"));
}

static void imprimirEstado() {
  Serial.printf("# estado=%s fallas=0x%02X (%s) fuente=%s enlace=%s tramas=%lu err_crc=%lu\n",
                NOMBRE_ESTADO[estado], fallas, motivoFalla[0] ? motivoFalla : "-",
                fuente == ENLACE ? "enlace" : "consola", enlaceActivo() ? "activo" : "no",
                (unsigned long)rx.tramasValidas, (unsigned long)rx.erroresCrc);
  for (uint8_t e = 0; e < N_EJES; e++) {
    Serial.printf("#  %-4s med=%+8.3f ref=%+8.3f w=%+7.2f pasos=%+8.3f raw=%4d cero=%4d "
                  "Kp=%.2f Ki=%.2f st=0x%02X i2c_err=%u EN=%d fin=%s\n",
                  EJES[e].nombre, ej[e].thetaMed, ej[e].refEff, ej[e].omegaCmd, ej[e].thetaPasos,
                  sensor[e].raw(), sensor[e].cero(), ctl[e].kp(), ctl[e].ki(),
                  sensor[e].estado(), sensor[e].erroresSeguidos(), pasos::habilitado(e),
                  !USAR_FINES_CARRERA ? "no_usado" : (finActivo(e) ? "ACTIVO" : "ok"));
  }
}

static int ejeDeTexto(const char* s) {
  if (!s) return -1;
  if (!strcmp(s, "0") || !strcmp(s, "pan") || !strcmp(s, "p")) return EJE_PAN;
  if (!strcmp(s, "1") || !strcmp(s, "tilt") || !strcmp(s, "t")) return EJE_TILT;
  return -1;
}

static void descargarCaptura() {
  if (capturando) { aviso("captura en curso, espera a que termine"); return; }
  if (capturaN == 0) { aviso("no hay captura; usa 'g <eje> <deg>'"); return; }
  //* alfa_max = 0 en la cabecera significa "sin límite".
  Serial.printf("#CAPTURA eje=%s kp=%.4f ki=%.4f escalon=%.4f n=%u dt_us=%lu omega_max=%.1f alfa_max=%.1f\n",
                EJES[capturaEje].nombre, capturaKp, capturaKi, capturaEscalon, capturaN,
                (unsigned long)(PERIODO_CTL_US * CAPTURA_DECIMACION), OMEGA_MAX_DPS, capturaAlfa >= 1e8f ? 0.0f : capturaAlfa);
  Serial.println("t_s,ref_deg,med_deg,omega_dps,pasos_deg");
  for (uint16_t i = 0; i < capturaN; i++) {
    const Muestra& m = captura[i];
    Serial.printf("%.6f,%.4f,%.4f,%.3f,%.4f\n", m.t_us * 1e-6f, m.ref, m.med, m.omega, m.pasos);
  }
  Serial.println("#FIN");
}

//* Prueba de signos: +3° y vuelta en lazo abierto, comparando pasos emitidos con el sensor.
//! Bloquea el lazo ~1 s. El otro eje queda detenido (sin pulsos, con torque de retención).
static void pruebaSignos(uint8_t e) {
  if (estado != RETENCION) { aviso("t: primero 'e 1' (estado RETENCION)"); return; }
  const float AMPL = 3.0f, VEL = 20.0f;
  const uint32_t tMov = (uint32_t)(1000.0f * AMPL / VEL);
  for (uint8_t i = 0; i < N_EJES; i++) pasos::fijarVelocidad(i, 0.0f);
  delay(50);

  sensor[e].leer();
  const float th0 = sensor[e].gradosEje();
  const int32_t p0 = pasos::posicion(e);
  pasos::fijarVelocidad(e, omegaAPasos(e, +VEL)); delay(tMov);
  pasos::fijarVelocidad(e, 0.0f);                 delay(300);
  sensor[e].leer();
  const float th1 = sensor[e].gradosEje();
  const float dPasos = gradosPorPasoEje(e) * (float)(pasos::posicion(e) - p0);

  pasos::fijarVelocidad(e, omegaAPasos(e, -VEL)); delay(tMov);
  pasos::fijarVelocidad(e, 0.0f);                 delay(300);
  sensor[e].leer();

  const float dMed = th1 - th0;
  aviso("prueba de signos %s: ordenado %+.2f deg | sensor %+.2f deg", EJES[e].nombre, dPasos, dMed);
  aviso("  1) La camara debio moverse hacia %s. Si fue al reves: invertir signoMotor.",
        e == EJE_PAN ? "la DERECHA" : "ARRIBA");
  if (dMed * dPasos < 0)                 aviso("  2) sensor con signo CONTRARIO: invertir signoSensor.");
  else if (fabsf(dMed) < 0.5f * AMPL)    aviso("  2) el sensor casi no se movio: revisar iman/relacion/acople.");
  else                                   aviso("  2) signo del sensor coherente con los pasos.");
  aviso("  (si se corrige signoMotor, repetir la prueba: el paso 2 depende de el)");

  for (uint8_t i = 0; i < N_EJES; i++) {          // volver a RETENCION sin salto
    sensor[i].leer();
    ej[i].thetaMed = sensor[i].gradosEje();
    ej[i].omegaCmd = 0.0f;
    sincronizarPasos(i);
    retenerEje(i);
  }
}

static void ejecutarLinea(char* linea) {
  for (char* p = linea; *p; p++) if (*p < 0x20 || *p > 0x7E) return;   // basura binaria: ignorar
  char* cmd = strtok(linea, " \t");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  char* a3 = strtok(nullptr, " \t");
  const int eje = ejeDeTexto(a1);

  switch (cmd[0]) {
    case '?': imprimirAyuda(); break;
    case 'p': imprimirEstado(); break;
    case 'e':
      if (a1 && atoi(a1) == 1) rearmarYHabilitar();
      else irA(DESHABILITADO, "consola");
      break;
    case 'x': irA(DESHABILITADO, "parada inmediata"); break;
    case 's':
      if (estado == LAZO || estado == ABIERTO) { fuente = CONSOLA; irA(RETENCION, "consola"); }
      break;
    case 'r':
    case 'g': {
      if (eje < 0 || !a2) { aviso("uso: %c <eje> <deg>", cmd[0]); break; }
      if (estado != RETENCION && estado != LAZO && estado != ABIERTO) { aviso("primero 'e 1'"); break; }
      if (capturando) { aviso("captura en curso"); break; }
      const float x = atof(a2);
      const ConfigEje& c = EJES[eje];
      fuente = CONSOLA;
      if (estado == ABIERTO) irA(RETENCION, "consola");
      const float destino = (cmd[0] == 'g') ? ej[eje].thetaMed + x : x;
      ej[eje].ref = limitar(destino, c.limMinDeg, c.limMaxDeg);
      ej[eje].omegaFF = 0.0f;
      ej[eje].tRefUs = micros();
      irA(LAZO, "referencia por consola");
      if (cmd[0] == 'g') {
        capturaEje = eje; capturaN = 0; capturaT0 = micros();
        capturaKp = ctl[eje].kp(); capturaKi = ctl[eje].ki(); capturaAlfa = ctl[eje].aceleracionMax();
        capturaEscalon = ej[eje].ref - ej[eje].thetaMed;
        capturando = true;
        aviso("escalon %s de %+.3f deg (ref %.3f), capturando 6 s...", c.nombre, capturaEscalon, ej[eje].ref);
      } else {
        aviso("ref %s = %.3f deg", c.nombre, ej[eje].ref);
      }
      break;
    }
    case 'd':
      //! Imprimir 3000 líneas bloquea ~1,3 s: antes se detienen los ejes (sin pulsos, con
      //! torque de retención) y al final se retiene la posición actual.
      for (uint8_t e = 0; e < N_EJES; e++) pasos::fijarVelocidad(e, 0.0f);
      descargarCaptura();
      if (estado == LAZO || estado == ABIERTO || estado == RETENCION) {
        for (uint8_t e = 0; e < N_EJES; e++) {
          sensor[e].leer(); ej[e].thetaMed = sensor[e].gradosEje(); ej[e].omegaCmd = 0.0f;
          sincronizarPasos(e); retenerEje(e);
        }
        fuente = CONSOLA;
        if (estado != RETENCION) irA(RETENCION, "tras descargar la captura");
      }
      break;
    case 'k':
      if (eje < 0 || !a2 || !a3) { aviso("uso: k <eje> <Kp> <Ki>"); break; }
      ctl[eje].ganancias(atof(a2), atof(a3));
      aviso("%s: Kp=%.3f 1/s  Ki=%.3f 1/s^2", EJES[eje].nombre, ctl[eje].kp(), ctl[eje].ki());
      break;
    case 'a':
      if (eje < 0 || !a2) { aviso("uso: a <eje> <deg/s2>  (0 = sin limite)"); break; }
      ctl[eje].aceleracionMax(atof(a2));
      if (atof(a2) <= 0.0f) aviso("%s: SIN limite de aceleracion. Usar solo con escalones chicos.", EJES[eje].nombre);
      else                  aviso("%s: alfa_max = %.1f deg/s2", EJES[eje].nombre, ctl[eje].aceleracionMax());
      break;
    case 'v':
      if (eje < 0 || !a2) { aviso("uso: v <eje> <deg/s>"); break; }
      if (estado == DESHABILITADO || estado == FALLA) { aviso("primero 'e 1'"); break; }
      fuente = CONSOLA;
      irA(ABIERTO, "velocidad por consola");
      ej[eje].omegaAbierto = limitar(atof(a2), -OMEGA_MAX_DPS, OMEGA_MAX_DPS);
      aviso("lazo abierto %s: %.2f deg/s ('s' para retener)", EJES[eje].nombre, ej[eje].omegaAbierto);
      break;
    case 't':
      if (eje < 0) { aviso("uso: t <eje>"); break; }
      pruebaSignos(eje);
      break;
    case 'z':
      if (estado != DESHABILITADO) { aviso("z: primero 'e 0' (deshabilitado)"); break; }
      for (uint8_t e = 0; e < N_EJES; e++) { sensor[e].leer(); sensor[e].ponerCero(); }
      aviso("cero fijado. Para que sea permanente, en config.h: ceroRaw pan = %d, tilt = %d",
            sensor[EJE_PAN].cero(), sensor[EJE_TILT].cero());
      break;
    default: aviso("comando desconocido; '?' para ayuda");
  }
}

static void atenderSerie() {
  static char linea[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    const uint8_t b = (uint8_t)Serial.read();
    //* Con el receptor en reposo, 0xAA inicia trama; cualquier otro byte es texto.
    if (!rx.enReposo() || b == proto::CAB0) {
      if (rx.alimentar(b)) procesarTrama();
      continue;
    }
    if (b == '\n' || b == '\r') {
      linea[n] = 0;
      //! Con el enlace binario activo la consola se ignora: restos de una trama corrupta
      //! podrían parecer un comando. Para usar la consola, cerrar el programa de la RPi.
      if (n && !enlaceActivo()) ejecutarLinea(linea);
      n = 0;
    } else if (n < sizeof(linea) - 1) {
      linea[n++] = (char)b;
    }
  }
}

// ============================================================================
//* SETUP / LOOP
// ============================================================================
void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(BAUDIOS);
  delay(50);
  pasos::iniciar();                                //* drivers deshabilitados desde el primer instante

  for (uint8_t e = 0; e < N_EJES; e++) {
    const ConfigEje& c = EJES[e];
    ctl[e].configurar(c.kp, c.ki, OMEGA_MAX_DPS, ALFA_MAX_DPS2, BANDA_INTEGRAL_DEG);
    const bool ok = sensor[e].iniciar(bus(e), c.pinSda, c.pinScl, c.signoSensor, c.ceroRaw);
    ej[e].thetaMed = sensor[e].gradosEje();
    sincronizarPasos(e);
    Serial.printf("# AS5600 %s: %s raw=%d theta=%.2f deg status=0x%02X\n", c.nombre,
                  ok ? "OK" : "SIN RESPUESTA O SIN IMAN", sensor[e].raw(), ej[e].thetaMed, sensor[e].estado());
    if (!ok) declararFalla(bitSensor(e), "sensor %s no inicio", c.nombre);
  }
  Serial.printf("# esp32_pan_tilt listo: %lu baudios, control %lu Hz, 1/%u micropasos. Drivers DESHABILITADOS.\n",
                (unsigned long)BAUDIOS, (unsigned long)F_CONTROL_HZ, MICROPASOS);
  imprimirAyuda();
  tUltCtlUs = micros();
}

void loop() {
  atenderSerie();

  const uint32_t ahora = micros();
  if (ahora - tUltCtlUs >= PERIODO_CTL_US) {
    float dt = (ahora - tUltCtlUs) * 1e-6f;
    if (dt > 0.005f) dt = 0.005f;                  //! tras un bloqueo largo (prueba 't'), no integrar de golpe
    tUltCtlUs = ahora;
    pasoControl(dt, ahora);
  }

  if (enlaceActivo() && (ahora - tUltTelemetriaUs) >= PERIODO_TELEMETRIA_US) {
    tUltTelemetriaUs = ahora;
    enviarTelemetria(tMedUs);                     //* marca = instante de la medición, no del envío
  }
}
