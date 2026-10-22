#pragma once
/*
 * as5600.h — sensado angular (AS5600 por I2C), uno por eje y uno por bus.
 *
 * Entrega θ_med en grados del eje, con signo y cero configurables, y "desenrollado":
 * el AS5600 da 0..4095 por vuelta, y aquí se acumulan los cruces por 0/360° para que
 * θ_med sea continuo (a 1 kHz el eje gira mucho menos de media vuelta entre lecturas).
 */
#include <Arduino.h>
#include <Wire.h>

class SensorAS5600 {
 public:
  //* bus: Wire o Wire1. Devuelve false si el sensor no responde.
  bool iniciar(TwoWire& bus, uint8_t pinSda, uint8_t pinScl, int8_t signo, int16_t ceroRaw);

  //* Una lectura del ángulo crudo (≈ 0,1–0,2 ms a 400 kHz). Devuelve false si falló.
  bool leer();

  //* Revisa el imán (registro STATUS): 1 = presente, 0 = ausente, −1 = error de lectura I2C.
  //* Llamarlo a ~10 Hz, no a 1 kHz.
  int8_t revisarIman();

  //* true cuando ya hubo una lectura buena y el cero quedó aplicado.
  bool referenciado() const { return referenciado_; }

  //* Fija el cero en la posición actual.
  void ponerCero();

  //* Invierte el sentido (signoSensor) sin perder la referencia del cero.
  void invertirSigno() { signo_ = -signo_; recalcular(); }
  int8_t signo() const { return signo_; }

  float   gradosEje() const { return grados_; }    // θ_med [°]
  int16_t raw() const { return raw_; }             // 0..4095, último valor válido
  int16_t cero() const { return cero_; }
  uint8_t erroresSeguidos() const { return errores_; }
  uint8_t estado() const { return status_; }       // MD (bit 5), ML (bit 4), MH (bit 3)

 private:
  bool leerRegistro16(uint8_t reg, uint16_t& valor);
  bool leerRegistro8(uint8_t reg, uint8_t& valor);
  bool escribirRegistro8(uint8_t reg, uint8_t valor);
  void recalcular();

  TwoWire* bus_ = nullptr;
  int8_t  signo_ = +1;
  int16_t cero_ = 0;
  int16_t ceroConfig_ = -1;
  bool    referenciado_ = false;
  int16_t raw_ = 0;
  int32_t acumulado_ = 0;     // cuentas desenrolladas
  bool    primera_ = true;
  uint8_t errores_ = 0;
  uint8_t status_ = 0;
  float   grados_ = 0.0f;
};
