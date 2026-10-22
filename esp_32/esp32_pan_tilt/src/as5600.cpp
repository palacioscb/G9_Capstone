/*
 * as5600.cpp — lectura del AS5600 (ams OSRAM), registros según su hoja de datos.
 */
#include "as5600.h"
#include "config.h"

namespace {
constexpr uint8_t DIR_AS5600   = 0x36;   // dirección I2C fija
constexpr uint8_t REG_CONF_H   = 0x07;   // WD | FTH[2:0] | SF[1:0]
constexpr uint8_t REG_STATUS   = 0x0B;   // MD bit 5, ML bit 4, MH bit 3
constexpr uint8_t REG_RAW_ANG  = 0x0C;   // RAW ANGLE, 12 bits (0x0C alto, 0x0D bajo)
constexpr float   CUENTAS_VUELTA = 4096.0f;
}

bool SensorAS5600::iniciar(TwoWire& bus, uint8_t pinSda, uint8_t pinScl, int8_t signo, int16_t ceroRaw) {
  bus_ = &bus;
  signo_ = (signo >= 0) ? +1 : -1;
  bus_->begin(pinSda, pinScl, I2C_HZ);
  bus_->setTimeOut(I2C_TIMEOUT_MS);

  //* Filtro lento SF (bits 1:0 de CONF alto). Escritura volátil: NO se graba (sin BURN),
  //* se repite en cada arranque.
  uint8_t conf;
  if (leerRegistro8(REG_CONF_H, conf)) {
    conf = (conf & ~0x03) | (AS5600_SF & 0x03);
    escribirRegistro8(REG_CONF_H, conf);
  }

  ceroConfig_ = ceroRaw;
  primera_ = true;                                  //* el cero se aplica en la PRIMERA lectura buena
  if (!leer()) return false;
  return revisarIman() == 1;
}

bool SensorAS5600::leer() {
  uint16_t v;
  if (!leerRegistro16(REG_RAW_ANG, v)) {
    if (errores_ < 255) errores_++;
    return false;
  }
  errores_ = 0;
  const int16_t nuevo = (int16_t)(v & 0x0FFF);
  if (primera_) {
    //! El cero se fija aquí y no en iniciar(): si la primera lectura del arranque falla,
    //! la siguiente buena igual deja θ referido al cero correcto (revisión del 21-10).
    if (ceroConfig_ >= 0 && ceroConfig_ < 4096) {
      //* Cero configurado: θ_med absoluto desde el arranque (homing por encoder absoluto).
      //* Se elige el acumulado más cercano al cero para que θ quede en ±180°.
      cero_ = ceroConfig_;
      int32_t d = (int32_t)nuevo - cero_;
      if (d >  2048) d -= 4096;
      if (d < -2048) d += 4096;
      acumulado_ = cero_ + d;
    } else {
      //? Sin cero configurado, θ = 0 donde esté el eje al encender: centrar la plataforma
      //? ANTES de energizar, o fijar ceroRaw en config.h con el valor que imprime 'z'.
      cero_ = nuevo;
      acumulado_ = nuevo;
    }
    referenciado_ = true;
    primera_ = false;
  } else {
    int32_t d = (int32_t)nuevo - raw_;          // desenrollado: el salto más corto
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    acumulado_ += d;
  }
  raw_ = nuevo;
  recalcular();
  return true;
}

int8_t SensorAS5600::revisarIman() {
  uint8_t s;
  if (!leerRegistro8(REG_STATUS, s)) return -1;    // error de lectura: NO es "sin imán"
  status_ = s;
  return (s & 0x20) ? 1 : 0;                       // MD: imán detectado
}

void SensorAS5600::ponerCero() {
  cero_ = raw_;
  acumulado_ = raw_;
  recalcular();
}

void SensorAS5600::recalcular() {
  grados_ = signo_ * (float)(acumulado_ - cero_) * (360.0f / CUENTAS_VUELTA);
}

bool SensorAS5600::leerRegistro16(uint8_t reg, uint16_t& valor) {
  bus_->beginTransmission(DIR_AS5600);
  bus_->write(reg);
  if (bus_->endTransmission(false) != 0) return false;           // inicio repetido
  if (bus_->requestFrom(DIR_AS5600, (uint8_t)2) != 2) return false;
  const uint8_t hi = bus_->read();
  const uint8_t lo = bus_->read();
  valor = ((uint16_t)hi << 8) | lo;
  return true;
}

bool SensorAS5600::leerRegistro8(uint8_t reg, uint8_t& valor) {
  bus_->beginTransmission(DIR_AS5600);
  bus_->write(reg);
  if (bus_->endTransmission(false) != 0) return false;
  if (bus_->requestFrom(DIR_AS5600, (uint8_t)1) != 1) return false;
  valor = bus_->read();
  return true;
}

bool SensorAS5600::escribirRegistro8(uint8_t reg, uint8_t valor) {
  bus_->beginTransmission(DIR_AS5600);
  bus_->write(reg);
  bus_->write(valor);
  return bus_->endTransmission() == 0;
}
