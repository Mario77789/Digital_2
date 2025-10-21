/*
 * uart_gamepad.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */


#include "uart_gamepad.h"
#include <string.h>
#include <stdio.h>

#define INPUT_RX_BUFSZ 256
#define TH_LOW   1500
#define TH_HIGH  2000
#define MAX_SAMPLE 4095

static UART_HandleTypeDef *s_uart;
static volatile PlayerInput s_p1 = {0}, s_p2 = {0};
static char     segbuf[INPUT_RX_BUFSZ];
static uint16_t seglen = 0;
static uint8_t  rx_byte;

static inline DirMask calc_dir(uint16_t X, uint16_t Y)
{
  DirMask d = DIR_NONE;
  if (X >= TH_HIGH || X >= MAX_SAMPLE) d = (DirMask)(d | DIR_UP);
  else if (X < TH_LOW)                 d = (DirMask)(d | DIR_DOWN);
  if (Y >= TH_HIGH)                    d = (DirMask)(d | DIR_RIGHT);
  else if (Y < TH_LOW)                 d = (DirMask)(d | DIR_LEFT);
  return d;
}

static void apply(uint8_t id, uint16_t X, uint16_t Y, uint8_t A, uint8_t B)
{
  PlayerInput *p = (id==1)? (PlayerInput*)&s_p1 : (PlayerInput*)&s_p2;
  p->X=X; p->Y=Y; p->A=A?1:0; p->B=B?1:0; p->dir=calc_dir(X,Y);
}

static void parse_segment(const char *s, uint16_t n)
{
  int id=0, x=0, y=0, sw=0, a=0, b=0;
  char tmp[INPUT_RX_BUFSZ];
  if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
  memcpy(tmp, s, n); tmp[n]='\0';
  char *pc = strchr(tmp, ';'); if (pc) *pc = '\0';

  int matched = sscanf(tmp, "M%d:X%dY%dSW%dA%dB%d", &id, &x, &y, &sw, &a, &b);
  if (matched < 6) {
    matched = sscanf(tmp, " M%d : X%d Y%d SW%d A%d B%d", &id, &x, &y, &sw, &a, &b);
    if (matched < 6) return;
  }
  if (id!=1 && id!=2) return;

  if (x<0) x=0; if (x>4095) x=4095;
  if (y<0) y=0; if (y>4095) y=4095;
  apply((uint8_t)id, (uint16_t)x, (uint16_t)y, (uint8_t)a, (uint8_t)b);
}

static void on_byte(uint8_t b)
{
  if (seglen >= INPUT_RX_BUFSZ-1) seglen = 0;
  segbuf[seglen++] = (char)b;

  if (b == ';') { parse_segment(segbuf, seglen); seglen = 0; }
  else if (b == '\n' || b == '\r') { if (seglen>0) { parse_segment(segbuf, seglen); seglen=0; } }
}

void Gamepad_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  seglen = 0;
  HAL_UART_Receive_IT(s_uart, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (s_uart && huart->Instance == s_uart->Instance) {
    on_byte(rx_byte);
    HAL_UART_Receive_IT(s_uart, &rx_byte, 1);
  }
}

void Gamepad_GetP1(PlayerInput *out) { if (out) *out = s_p1; }
void Gamepad_GetP2(PlayerInput *out) { if (out) *out = s_p2; }
