/*
 * uart_gamepad.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#ifndef INC_UART_GAMEPAD_H_
#define INC_UART_GAMEPAD_H_

#pragma once
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum { DIR_NONE=0, DIR_UP=1, DIR_DOWN=2, DIR_LEFT=4, DIR_RIGHT=8 } DirMask;
typedef struct {
  uint16_t X, Y;
  uint8_t  A, B;
  DirMask  dir;
} PlayerInput;

void Gamepad_Init(UART_HandleTypeDef *huart);   // usa USART3
void Gamepad_GetP1(PlayerInput *out);
void Gamepad_GetP2(PlayerInput *out);


#endif /* INC_UART_GAMEPAD_H_ */
