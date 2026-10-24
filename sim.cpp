// sim.cpp (PROVIDED): data stream + simulated motor. Don't edit.
#include "foc.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

int main() {
  const float PI = 3.14159265f, FLUX = 0.005f, dt = 1e-4f;   // 10 kHz
  float ia = 0, ib = 0, ic = 0, err2 = 0, iq_target = 0;
  int n = 0;
  FILE* f = fopen("output.csv", "w");
  fprintf(f, "t,iq_target,iq_actual\n");

  for (int k = 0; k < 40000; k++) {                 // 4 seconds
    float t = k * dt;

    // ---- Data stream: shaft angle and target current ----
    float theta = 0.3f, omega = 0;
    if (t > 1 && t < 3) { theta = 0.3f + sinf(2*PI*(t-1)); omega = 2*PI*cosf(2*PI*(t-1)); }
    if (k % 10 == 0)                                // outer loop updates at 1 kHz
      iq_target = t < 0.2f ? 0 : t < 0.5f ? 0.5f : t < 0.8f ? 1.0f : t < 1 ? 0
                : t < 3 ? std::clamp(-0.8f*(theta-0.3f), -1.f, 1.f) : t < 3.8f ? t-3 : 0;
    int raw = ((int)lroundf(theta * 16384 / (2*PI)) + ENC_OFFSET) % 16384;

    // ---- Your FOC ----
    float duty[3] = {0.5f, 0.5f, 0.5f};
    foc_step(ia, ib, ic, (uint16_t)raw, iq_target, dt, duty);

    // ---- Simulated motor (phase frame only) ----
    float v[3], i[3] = {ia, ib, ic}, e[3], th = POLE_PAIRS * theta;
    for (int p = 0; p < 3; p++) v[p] = std::clamp(duty[p], 0.f, 1.f) * VBUS;
    float vn = (v[0] + v[1] + v[2]) / 3;
    for (int s = 0; s < 10; s++) {
      float tq = 0;
      for (int p = 0; p < 3; p++) {
        e[p] = -sinf(th - p * 2*PI/3);
        i[p] += dt/10 * (v[p] - vn - R*i[p] - POLE_PAIRS*omega*FLUX*e[p]) / L;
      }
      float m = (i[0] + i[1] + i[2]) / 3;
      for (int p = 0; p < 3; p++) { i[p] -= m; tq += e[p] * i[p]; }
      if (s == 9 && k % 10 == 0) {
        float iq_actual = tq / 1.5f;                 // torque current the motor actually makes
        err2 += (iq_target - iq_actual) * (iq_target - iq_actual); n++;
        fprintf(f, "%.4f,%.4f,%.4f\n", t, iq_target, iq_actual);
      }
    }
    ia = i[0]; ib = i[1]; ic = i[2];
  }
  fclose(f);
  float rms = sqrtf(err2 / n);
  printf("Iq error: %.3f A  ->  %s\n", rms, rms < 0.05f ? "PASS" : "NOT YET");
}