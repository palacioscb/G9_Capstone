/*
 * control_eje.cpp — PI de posición con salida en velocidad (pid_t_2.pdf).
 */
#include "control_eje.h"
#include <math.h>

static inline float saturar(float x, float lim) {
  return (x > lim) ? lim : ((x < -lim) ? -lim : x);
}

float ControlEje::paso(float ref, float med, float omegaFF, float dt) {
  error_ = ref - med;

  //* Separación del integrador: sólo se integra con |e| < banda. En un movimiento largo el
  //* integrador no acumula durante la aproximación (eso causaba el sobrepaso en pan).
  const float integCandidata = (fabsf(error_) < bandaIntegral_) ? integ_ + ki_ * error_ * dt : integ_;

  //* Tope de frenado sobre el término PROPORCIONAL: |P| ≤ √(2·α_max·|e|), la velocidad desde
  //* la que todavía se alcanza a frenar con α_max en la distancia que queda. Sin esto, el
  //* límite de α trunca el frenado y el eje se pasa (simulado: tilt 60° -> +27,8°).
  //* El integrador no se toca: cerca de e = 0 sigue aportando lo que hace falta para seguir
  //* una rampa. Con α sin límite ('a <eje> 0') el tope desaparece y la ley es la lineal.
  float p = kp_ * error_;
  const float vFren = sqrtf(2.0f * alfaMax_ * fabsf(error_));
  if (fabsf(p) > vFren) p = (error_ > 0.0f) ? vFren : -vFren;
  const float u = omegaFF + p + integCandidata;                     // salida antes de los límites

  const float uSat = saturar(u, omegaMax_);                     // |ω| ≤ ω_max
  const float dMax = alfaMax_ * dt;                                 // |Δω| ≤ α_max·dt
  const float nuevo = omega_ + saturar(uSat - omega_, dMax);

  //* Integración condicional: no integrar si la salida está limitada y el error
  //* empuja hacia afuera del límite (evita el "windup" y el sobrepaso que trae).
  const float exceso = u - nuevo;
  limitado_ = fabsf(exceso) > 1e-6f;
  if (!(limitado_ && exceso * error_ > 0.0f)) integ_ = integCandidata;

  omega_ = nuevo;
  return omega_;
}
