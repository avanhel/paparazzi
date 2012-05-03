#ifndef TRANSVEH_MULTIGAIN_H
#define TRANSVEH_MULTIGAIN_H

#include "std.h"

#define multiGain_SetKiPhi(_val) { \
    stabilization_gains.i.x = _val;             \
    stabilization_att_sum_err.phi = 0;          \
  }


extern struct Int32AttitudeGains stabilization_gainsA;
extern struct Int32AttitudeGains stabilization_gainsB;
extern struct Int32AttitudeGains stabilization_gainsC;


extern uint8_t activeGainSet;
extern void transveh_multigain_init(void);
extern void transveh_multigain_periodic(void);
extern void SetGainSetA(void);
extern void SetGainSetB(void);
extern void SetGainSetC(void);
extern void multiGain_SetGainSetHandler(uint8_t v);

#endif  /* TRANSVEH_MULTIGAIN_H */