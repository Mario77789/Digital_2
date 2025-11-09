/*
 * audio_pwm.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#include "audio_pwm.h"

static TIM_HandleTypeDef *s_pwm;
static uint32_t s_timclk;

void AudioPWM_Init(TIM_HandleTypeDef *htim_pwm)
{
  s_pwm = htim_pwm;

  // Reloj efectivo (timers en APB2 duplican si prescaler != 1)
  s_timclk = HAL_RCC_GetPCLK2Freq();
  uint32_t ppre2 = ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos) & 0x7;
  if (ppre2 >= 4) s_timclk *= 2U;

  // Base 1 MHz
  uint32_t target = 1000000UL;
  uint32_t psc = (s_timclk + target/2U) / target;
  if (psc) psc--;
  if (psc > 65535U) psc = 65535U;
  __HAL_TIM_SET_PRESCALER(s_pwm, (uint16_t)psc);

  HAL_TIM_PWM_Start(s_pwm, TIM_CHANNEL_1);
  AudioPWM_Silence();
}

void AudioPWM_SetFreq(uint32_t hz)
{
  if (!s_pwm) return;
  if (hz == 0) { AudioPWM_Silence(); return; }

  uint32_t arr = (1000000UL / hz);
  if (arr) arr--;
  if (arr > 65535U) arr = 65535U;
  __HAL_TIM_SET_AUTORELOAD(s_pwm, (uint16_t)arr);
  __HAL_TIM_SET_COMPARE(s_pwm, TIM_CHANNEL_1, (uint16_t)(arr >> 1)); // 50%
}

void AudioPWM_Silence(void)
{
  if (!s_pwm) return;
  __HAL_TIM_SET_COMPARE(s_pwm, TIM_CHANNEL_1, 0);
}

