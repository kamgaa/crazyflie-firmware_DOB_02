#pragma once
#include <stdint.h>

typedef struct {
  // params
  float J;     // inertia or effective inertia gain (스케일 게인)
  float Ts;    // sample time [s]
  float fc;    // cutoff [Hz]
  float tau_min, tau_max;

  // Q filter coeffs/state (1st-order Butterworth via bilinear)
  float a1, b0, b1;
  float prev_x, prev_y;

  // differentiator state (band-limited differentiator)
  float prev_omega;
  float prev_omega_dot;
  float prev_omega_dot_naive;

  // disturbance estimate
  float d_hat;
} Dob1D;

void  dob1d_configure(Dob1D* d, float J, float Ts, float fc, float tau_min, float tau_max);
float dob1d_update   (Dob1D* d, float tau_cmd, float omega_meas);
