#pragma once
/*
 * control_eje.h — CS-10: lazo de posición por eje (CTL-1), ley de pid_t_2.pdf.
 *
 *   e      = θ_ref − θ_med
 *   P      = sign(e)·min(Kp·|e|, √(2·α_max·|e|))     (tope de frenado del término P)
 *   u      = ω_ff + P + Ki·∫e dt                     (C(s) = Kp + Ki/s, más feedforward;
 *                                                     el integrador sólo acumula con |e| < banda)
 *   ω_sat  = sat(u, ±ω_max)
 *   ω_cmd  = ω_cmd_anterior + sat(ω_sat − ω_cmd_anterior, ±α_max·dt)
 *
 * La salida es la velocidad de comando ω_cmd [°/s], que es la variable manipulada de la
 * planta G(s) = Kt / (s (M s² + b s + Kt)).
 *
 * Anti-windup por integración condicional: si la salida quedó limitada (por saturación o
 * por el límite de aceleración) y el error empuja en el mismo sentido del exceso, el
 * integrador no se actualiza en ese paso.
 *
 *! pt/modelo_eje.py (RPi) replica este algoritmo línea por línea para simular la prueba
 *! de escalón. Si se cambia aquí, cambiar también allá.
 */

class ControlEje {
 public:
  void configurar(float kp, float ki, float omegaMax, float alfaMax, float bandaIntegral) {
    kp_ = kp; ki_ = ki; omegaMax_ = omegaMax; alfaMax_ = alfaMax; bandaIntegral_ = bandaIntegral;
  }
  void ganancias(float kp, float ki) { kp_ = kp; ki_ = ki; }
  //* α_max en °/s². Un valor ≤ 0 quita el límite (sólo para validar el modelo LINEAL con
  //* escalones chicos: un salto grande de ω sin rampa puede hacer perder pasos).
  void aceleracionMax(float alfa) { alfaMax_ = (alfa > 0.0f) ? alfa : 1e9f; }
  float aceleracionMax() const { return alfaMax_; }

  //* Reinicio sin salto: integrador en cero y ω_cmd en el valor dado (normalmente 0).
  void reiniciar(float omegaActual = 0.0f) { integ_ = 0.0f; omega_ = omegaActual; error_ = 0.0f; }

  //* Un período de control. Devuelve ω_cmd [°/s].
  float paso(float ref, float med, float omegaFF, float dt);

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float omega() const { return omega_; }
  float error() const { return error_; }
  float integral() const { return integ_; }
  bool  limitado() const { return limitado_; }

 private:
  float kp_ = 0, ki_ = 0, omegaMax_ = 0, alfaMax_ = 0, bandaIntegral_ = 1e9f;
  float integ_ = 0, omega_ = 0, error_ = 0;
  bool  limitado_ = false;
};
