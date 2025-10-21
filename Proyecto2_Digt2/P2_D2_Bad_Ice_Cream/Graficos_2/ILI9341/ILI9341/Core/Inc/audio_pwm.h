/*
 * audio_pwm.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#ifndef INC_AUDIO_PWM_H_
#define INC_AUDIO_PWM_H_

#pragma once
#include "main.h"
#include <stdint.h>

void AudioPWM_Init(TIM_HandleTypeDef *htim_pwm);
void AudioPWM_SetFreq(uint32_t hz);   // 0 = silencio
void AudioPWM_Silence(void);




#endif /* INC_AUDIO_PWM_H_ */
