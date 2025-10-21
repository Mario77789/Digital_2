#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====== Umbrales para interpretar ejes ======
#define TH_LOW      1500
#define TH_HIGH     2000
#define MAX_SAMPLE  4095

typedef enum {
    DIR_NONE  = 0,
    DIR_UP    = 1 << 0,
    DIR_DOWN  = 1 << 1,
    DIR_LEFT  = 1 << 2,
    DIR_RIGHT = 1 << 3
} DirMask;

typedef struct {
    uint8_t  player;   // 1 ó 2
    uint16_t X;        // 0..4095
    uint16_t Y;        // 0..4095
    uint8_t  A;        // 0/1 (Patada)
    uint8_t  B;        // 0/1 (Hielo)
    DirMask  dir;      // máscara de dirección
} PlayerEvent;

// ====== API ======
void Input_Init(UART_HandleTypeDef *huart2);      // Arranca recepción IT
void Input_RxIRQ_Handler(void);                   // Llamar desde HAL_UART_RxCpltCallback
bool Input_TryDequeue(PlayerEvent *out_evt);      // Lectura no bloqueante de eventos

#ifdef __cplusplus
}
#endif
