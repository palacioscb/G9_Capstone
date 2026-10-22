#pragma once
/*
 * protocolo.h — CS-4: tramas binarias RPi <-> ESP32 (las mismas de pt/enlace.py).
 *
 *   AA 55 | seq (1 B) | tipo (1 B) | largo (1 B) | carga (largo B) | CRC-16 (2 B, LE)
 *   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) calculado sobre seq..carga.
 *   Todos los números, little-endian. Ángulos en grados del eje, velocidades en °/s.
 *
 *   tipo 0x02  CONSIGNA_THETA (RPi -> ESP32, 16 B):
 *              f32 θref_pan, f32 θref_tilt, f32 ωff_pan, f32 ωff_tilt
 *   tipo 0x03  COMANDO (RPi -> ESP32, 1 B): u8 código (ver Comando)
 *   tipo 0x81  TELEMETRÍA (ESP32 -> RPi, 31 B):
 *              u32 t_us, f32 θmed_pan, f32 θmed_tilt, f32 θref_pan, f32 θref_tilt,
 *              f32 ωcmd_pan, f32 ωcmd_tilt, u8 estado, u8 fallas, u8 seq_última_consigna
 *
 * El mismo puerto atiende la consola de texto de diagnóstico: un byte 0xAA con el
 * receptor en reposo inicia una trama; cualquier otro byte va a la línea de texto.
 */
#include <Arduino.h>

namespace proto {

constexpr uint8_t CAB0 = 0xAA, CAB1 = 0x55;
constexpr uint8_t CARGA_MAX = 48;

enum Tipo : uint8_t {
  CONSIGNA_OMEGA = 0x01,     // heredado del esqueleto, no se usa en este firmware
  CONSIGNA_THETA = 0x02,
  COMANDO        = 0x03,
  TELEMETRIA     = 0x81,
};

enum Comando : uint8_t {
  CMD_HABILITAR    = 1,      // habilita ambos drivers y rearma fallas (rearme manual, D21)
  CMD_DESHABILITAR = 2,
  CMD_RETENER      = 3,      // mantener la posición actual
  CMD_CERO         = 4,      // θ = 0 en la posición actual (sólo deshabilitado)
};

constexpr uint8_t LARGO_CONSIGNA_THETA = 16;
constexpr uint8_t LARGO_TELEMETRIA     = 31;

uint16_t crc16(const uint8_t* datos, size_t n, uint16_t crc = 0xFFFF);

//* Arma una trama completa en `buf` (tamaño ≥ 7 + n). Devuelve el largo total.
size_t armar(uint8_t* buf, uint8_t seq, uint8_t tipo, const uint8_t* carga, uint8_t n);

//* Receptor byte a byte con máquina de estados. Una trama con CRC inválido se descarta.
class Receptor {
 public:
  //* Devuelve true cuando acaba de completarse una trama válida (leer seq/tipo/carga).
  bool alimentar(uint8_t b);
  bool enReposo() const { return estado_ == E_CAB0; }
  void reiniciar() { estado_ = E_CAB0; }

  uint8_t seq = 0, tipo = 0, largo = 0;
  uint8_t carga[CARGA_MAX];
  uint32_t tramasValidas = 0, erroresCrc = 0;

 private:
  enum Estado : uint8_t { E_CAB0, E_CAB1, E_SEQ, E_TIPO, E_LARGO, E_CARGA, E_CRC_L, E_CRC_H };
  Estado estado_ = E_CAB0;
  uint8_t idx_ = 0;
  uint16_t crcRx_ = 0;
};

//* Lectura y escritura de números little-endian en una carga.
inline float leerF32(const uint8_t* p) { float f; memcpy(&f, p, 4); return f; }
inline void  ponerF32(uint8_t* p, float f) { memcpy(p, &f, 4); }
inline void  ponerU32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }

}  // namespace proto
