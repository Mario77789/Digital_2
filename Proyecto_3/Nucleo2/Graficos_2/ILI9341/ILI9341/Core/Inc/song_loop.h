/*
 * song_loop.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#ifndef INC_SONG_LOOP_H_
#define INC_SONG_LOOP_H_

#pragma once
#include "main.h"
#include <stdbool.h>

void  SongLoop_Init(TIM_HandleTypeDef *htim_sched, uint32_t speed_x);
bool  SongLoop_Start(const char *path);   // carga a RAM y arranca
// ISR real del timer (define el símbolo débil de HAL)
void  TIM1_UP_TIM10_IRQHandler(void);
/* Reproduce desde un arreglo en Flash (no SD).
 * - notes: array de {midi, ticks}
 * - count: cantidad de elementos
 * - tick_ms: duración de 1 tick en ms (del archivo)
 * - loop: 1 para repetir automáticamente
 */
void SongLoop_StartFromArray(const void *notes, uint32_t count, uint32_t tick_ms, int loop);

/* Detener la reproducción actual (ya existe si la tienes; si no, añádela) */
void SongLoop_Stop(void);



#endif /* INC_SONG_LOOP_H_ */
