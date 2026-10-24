// foc.cpp: implement FOC here.
#include "foc.h"
#include <cmath>

// TODO 1: encoder_raw -> electrical angle (subtract ENC_OFFSET, counts -> radians, x POLE_PAIRS)
// TODO 2: Clarke: ia, ib, ic -> I_alpha, I_beta
// TODO 3: Park:   I_alpha, I_beta -> Id, Iq
// TODO 4: PI controllers: Iq -> iq_target, Id -> 0   (hint: Kp = L*wc, Ki = R*wc, wc = 2*pi*1000)
// TODO 5: Inverse Park:   Vd, Vq -> V_alpha, V_beta
// TODO 6: Inverse Clarke: V_alpha, V_beta -> Va, Vb, Vc
// TODO 7: duty = 0.5 + V / VBUS for each phase

void foc_step(float ia, float ib, float ic, uint16_t encoder_raw,
              float iq_target, float dt, float duty[3]) {
  // your code here (duties stay at 0.5 = zero voltage until you fill this in)
}