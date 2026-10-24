#pragma once
#include <cstdint>

const float VBUS = 12.0f, R = 2.0f, L = 0.0005f;
const int POLE_PAIRS = 7, ENC_OFFSET = 1234;   // encoder count where electrical angle = 0

// Called every inner cycle (10 kHz). Return three duty cycles (0..1).
void foc_step(float ia, float ib, float ic, uint16_t encoder_raw,
              float iq_target, float dt, float duty[3]);