/*
 * protocolo.cpp — CRC-16/CCITT-FALSE y receptor de tramas.
 */
#include "protocolo.h"

namespace proto {

uint16_t crc16(const uint8_t* datos, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)datos[i] << 8;
    for (uint8_t k = 0; k < 8; k++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

size_t armar(uint8_t* buf, uint8_t seq, uint8_t tipo, const uint8_t* carga, uint8_t n) {
  buf[0] = CAB0; buf[1] = CAB1;
  buf[2] = seq;  buf[3] = tipo; buf[4] = n;
  if (n) memcpy(buf + 5, carga, n);
  const uint16_t c = crc16(buf + 2, 3 + n);
  buf[5 + n] = (uint8_t)(c & 0xFF);
  buf[6 + n] = (uint8_t)(c >> 8);
  return 7 + n;
}

bool Receptor::alimentar(uint8_t b) {
  switch (estado_) {
    case E_CAB0:  if (b == CAB0) estado_ = E_CAB1; break;
    case E_CAB1:  estado_ = (b == CAB1) ? E_SEQ : (b == CAB0 ? E_CAB1 : E_CAB0); break;
    case E_SEQ:   seq = b;  estado_ = E_TIPO; break;
    case E_TIPO:  tipo = b; estado_ = E_LARGO; break;
    case E_LARGO:
      largo = b; idx_ = 0;
      if (largo > CARGA_MAX) { estado_ = E_CAB0; erroresCrc++; }   //! largo imposible
      else estado_ = (largo == 0) ? E_CRC_L : E_CARGA;
      break;
    case E_CARGA:
      carga[idx_++] = b;
      if (idx_ >= largo) estado_ = E_CRC_L;
      break;
    case E_CRC_L: crcRx_ = b; estado_ = E_CRC_H; break;
    case E_CRC_H: {
      crcRx_ |= (uint16_t)b << 8;
      estado_ = E_CAB0;
      uint8_t cab[3] = {seq, tipo, largo};
      uint16_t c = crc16(cab, 3);
      c = crc16(carga, largo, c);
      if (c == crcRx_) { tramasValidas++; return true; }
      erroresCrc++;
      break;
    }
  }
  return false;
}

}  // namespace proto
