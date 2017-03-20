
#ifndef __MOTOR_H
#define __MOTOR_H 0x0200

// Motor Control Functions

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_it.h"
#include <stdio.h>

/* Motor Control PWM Output Functions */
void MT_PWM_init(uint32_t pwm_freq);

void MT_PWM_setDuty(uint8_t ch, int16_t duty);

uint32_t MT_PWM_getDuty(uint8_t ch);

/* Motor Encoder Functions */
void MT_Enc_init(void);

uint32_t MT_Enc_getValue(void);



#ifdef __cplusplus
}
#endif

#endif