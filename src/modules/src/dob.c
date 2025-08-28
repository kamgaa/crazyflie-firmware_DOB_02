#include "dob.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void dob1d_configure(Dob1D* d, float J, float Ts, float fc, float tau_min, float tau_max) {
  d->J = J; d->Ts = Ts; d->fc = fc;
  d->tau_min = tau_min; d->tau_max = tau_max;

  // 1st-order Butterworth (bilinear transform)
  const float c = tanf((float)M_PI * fc * Ts);
  const float denom = (1.0f + c);
  d->b0 = c / denom;
  d->b1 = d->b0;
  d->a1 = (1.0f - c) / denom;

  d->prev_x = 0.0f; d->prev_y = 0.0f;
  d->prev_omega = 0.0f;
  d->prev_omega_dot = 0.0f;
  d->prev_omega_dot_naive = 0.0f;
  d->d_hat = 0.0f;
}

float dob1d_update(Dob1D* d, float tau_cmd_in, float omega_meas) {
  // (1) clamp
  float tau_cmd = clampf(tau_cmd_in, d->tau_min, d->tau_max);

  // (2) band-limited differentiator: omega_dot
  const float Ts = d->Ts;
  const float omega_dot_naive = (omega_meas - d->prev_omega) / Ts;
  const float omega_dot = d->a1 * d->prev_omega_dot
                        + d->b0 * omega_dot_naive
                        + d->b1 * d->prev_omega_dot_naive;

  d->prev_omega_dot_naive = omega_dot_naive;
  d->prev_omega_dot = omega_dot;
  d->prev_omega = omega_meas;

  // (3) raw disturbance
  const float x = d->J * omega_dot - tau_cmd;

  // (4) Q-filter (same 1st-order LPF)
  d->d_hat = d->a1 * d->prev_y + d->b0 * x + d->b1 * d->prev_x;
  d->prev_x = x; d->prev_y = d->d_hat;

  // (5) compensated torque
  float tau_out = tau_cmd - d->d_hat;
  return clampf(tau_out, d->tau_min, d->tau_max);
}
