#ifndef MPPT_H
#define MPPT_H

#include "stm32f1xx_hal.h"

extern uint8_t flag_adc_done;
extern uint8_t flag_enter_charge;

extern uint8_t flag_charging_Bulk;
extern uint8_t flag_charging_CV;
extern uint8_t flag_charging_FLOAT;

void MPPT_Hybrid(void);
void MPPT_Hybrid_Reset(void);
void check_initial_state(void);
void charging_flow(void);

#endif
