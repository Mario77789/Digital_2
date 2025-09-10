/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- J2 (lado derecho) ---
// Pista J2 en D12..D5 (en ese orden visual) y WIN en D4
#define J2_LED1_Pin         GPIO_PIN_6   // D12 = PA6
#define J2_LED1_GPIO_Port   GPIOA
#define J2_LED2_Pin         GPIO_PIN_7   // D11 = PA7
#define J2_LED2_GPIO_Port   GPIOA
#define J2_LED3_Pin         GPIO_PIN_6   // D10 = PB6
#define J2_LED3_GPIO_Port   GPIOB
#define J2_LED4_Pin         GPIO_PIN_7   // D9  = PC7
#define J2_LED4_GPIO_Port   GPIOC
#define J2_LED5_Pin         GPIO_PIN_9   // D8  = PA9
#define J2_LED5_GPIO_Port   GPIOA
#define J2_LED6_Pin         GPIO_PIN_8   // D7  = PA8
#define J2_LED6_GPIO_Port   GPIOA
#define J2_LED7_Pin         GPIO_PIN_10  // D6  = PB10
#define J2_LED7_GPIO_Port   GPIOB
#define J2_LED8_Pin         GPIO_PIN_4   // D5  = PB4
#define J2_LED8_GPIO_Port   GPIOB

#define J2_WIN_Pin          GPIO_PIN_5   // D4  = PB5
#define J2_WIN_GPIO_Port    GPIOB

// vueltas necesarias para ganar
#define LAPS_TO_WIN 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// ------- estado de carrera -------
static volatile uint8_t race_enabled = 0;     // 0: no corre / 1: botones habilitados
static volatile uint8_t starting_seq = 0;     // 1 mientras corre el semáforo
static volatile uint8_t start_req = 0;        // flag solicitado desde EXTI
static volatile uint8_t race_finished = 0;    // 1 cuando hay ganador

// contadores octales (máscara 1,2,4,...,128)
static volatile uint8_t j1_mask = 0x01;
static volatile uint8_t j2_mask = 0x01;

// vueltas por jugador
static volatile uint8_t j1_laps = 0;
static volatile uint8_t j2_laps = 0;

// antirrebote en ms
#define DEBOUNCE_MS 150
static volatile uint32_t t_last_start = 0;
static volatile uint32_t t_last_j1    = 0;
static volatile uint32_t t_last_j2    = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void Track_Show_J1(uint8_t mask);
static void Track_Show_J2(uint8_t mask);
static void Lights_AllOff(void);
static void Lights_Red(void);
static void Lights_Yellow(void);
static void Lights_Green(void);

static inline void Reset_Game(void);
static inline void Winner_J1(void);
static inline void Winner_J2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint8_t octal_step(uint8_t m) {
  return (m == 0x80) ? 0x01 : (m << 1);
}

// secuencia de arranque: bloquea entradas mientras corre
static void Run_Starting_Lights(void)
{
  starting_seq = 1;
  race_enabled = 0;
  race_finished = 0;

  // limpia indicadores de ganador
  HAL_GPIO_WritePin(J1_WIN_GPIO_Port, J1_WIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_WIN_GPIO_Port, J2_WIN_Pin, GPIO_PIN_RESET);

  Lights_AllOff();

  // rojo (1 s)
  Lights_Red();
  HAL_Delay(1000);

  // amarillo (1 s)
  Lights_Yellow();
  HAL_Delay(1000);

  // verde  -> a partir de aquí se habilita la carrera
  Lights_Green();
  race_enabled = 1;

  // opcional: dejar el verde encendido o apagar semáforo
  // Lights_AllOff();

  starting_seq = 0;
}

// callback de interrupciones EXTI (HAL la invoca)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == StartRace_Pin) {
    if ((now - t_last_start) > DEBOUNCE_MS && !starting_seq) {
      t_last_start = now;
      start_req = 1; // se atiende en el while
    }
    return;
  }

  if (!race_enabled || starting_seq || race_finished) return;

  if (GPIO_Pin == PJ1_Pin) {
    if ((now - t_last_j1) > DEBOUNCE_MS) {
      uint8_t prev = j1_mask;
      t_last_j1 = now;
      j1_mask = octal_step(j1_mask);
      Track_Show_J1(j1_mask);
      if (prev == 0x80 && j1_mask == 0x01) {
        if (j1_laps < LAPS_TO_WIN) j1_laps++;
        if (j1_laps >= LAPS_TO_WIN) Winner_J1();
      }
    }
  }
  else if (GPIO_Pin == PJ2_Pin) {
    if ((now - t_last_j2) > DEBOUNCE_MS) {
      uint8_t prev = j2_mask;
      t_last_j2 = now;
      j2_mask = octal_step(j2_mask);
      Track_Show_J2(j2_mask);
      if (prev == 0x80 && j2_mask == 0x01) {
        if (j2_laps < LAPS_TO_WIN) j2_laps++;
        if (j2_laps >= LAPS_TO_WIN) Winner_J2();
      }
    }
  }
}

/* ---- helpers de LEDs de pistas ---- */
static void Track_Show_J1(uint8_t m)
{
  // apaga todos
  HAL_GPIO_WritePin(J1_LED1_GPIO_Port, J1_LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED2_GPIO_Port, J1_LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED3_GPIO_Port, J1_LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED4_GPIO_Port, J1_LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED5_GPIO_Port, J1_LED5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED6_GPIO_Port, J1_LED6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED7_GPIO_Port, J1_LED7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J1_LED8_GPIO_Port, J1_LED8_Pin, GPIO_PIN_RESET);

  if (m & 0x01) HAL_GPIO_WritePin(J1_LED1_GPIO_Port, J1_LED1_Pin, GPIO_PIN_SET);
  if (m & 0x02) HAL_GPIO_WritePin(J1_LED2_GPIO_Port, J1_LED2_Pin, GPIO_PIN_SET);
  if (m & 0x04) HAL_GPIO_WritePin(J1_LED3_GPIO_Port, J1_LED3_Pin, GPIO_PIN_SET);
  if (m & 0x08) HAL_GPIO_WritePin(J1_LED4_GPIO_Port, J1_LED4_Pin, GPIO_PIN_SET);
  if (m & 0x10) HAL_GPIO_WritePin(J1_LED5_GPIO_Port, J1_LED5_Pin, GPIO_PIN_SET);
  if (m & 0x20) HAL_GPIO_WritePin(J1_LED6_GPIO_Port, J1_LED6_Pin, GPIO_PIN_SET);
  if (m & 0x40) HAL_GPIO_WritePin(J1_LED7_GPIO_Port, J1_LED7_Pin, GPIO_PIN_SET);
  if (m & 0x80) HAL_GPIO_WritePin(J1_LED8_GPIO_Port, J1_LED8_Pin, GPIO_PIN_SET);
}

static void Track_Show_J2(uint8_t m)
{
  // apaga todos
  HAL_GPIO_WritePin(J2_LED1_GPIO_Port, J2_LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED2_GPIO_Port, J2_LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED3_GPIO_Port, J2_LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED4_GPIO_Port, J2_LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED5_GPIO_Port, J2_LED5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED6_GPIO_Port, J2_LED6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED7_GPIO_Port, J2_LED7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_LED8_GPIO_Port, J2_LED8_Pin, GPIO_PIN_RESET);

  if (m & 0x01) HAL_GPIO_WritePin(J2_LED1_GPIO_Port, J2_LED1_Pin, GPIO_PIN_SET);
  if (m & 0x02) HAL_GPIO_WritePin(J2_LED2_GPIO_Port, J2_LED2_Pin, GPIO_PIN_SET);
  if (m & 0x04) HAL_GPIO_WritePin(J2_LED3_GPIO_Port, J2_LED3_Pin, GPIO_PIN_SET);
  if (m & 0x08) HAL_GPIO_WritePin(J2_LED4_GPIO_Port, J2_LED4_Pin, GPIO_PIN_SET);
  if (m & 0x10) HAL_GPIO_WritePin(J2_LED5_GPIO_Port, J2_LED5_Pin, GPIO_PIN_SET);
  if (m & 0x20) HAL_GPIO_WritePin(J2_LED6_GPIO_Port, J2_LED6_Pin, GPIO_PIN_SET);
  if (m & 0x40) HAL_GPIO_WritePin(J2_LED7_GPIO_Port, J2_LED7_Pin, GPIO_PIN_SET);
  if (m & 0x80) HAL_GPIO_WritePin(J2_LED8_GPIO_Port, J2_LED8_Pin, GPIO_PIN_SET);
}

/* ---- semáforo ---- */
static void Lights_AllOff(void)
{
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port,     LED_ROJO_Pin,     GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port,    LED_VERDE_Pin,    GPIO_PIN_RESET);
}

static void Lights_Red(void)
{
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port,     LED_ROJO_Pin,     GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port,    LED_VERDE_Pin,    GPIO_PIN_RESET);
}

static void Lights_Yellow(void)
{
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port,     LED_ROJO_Pin,     GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port,    LED_VERDE_Pin,    GPIO_PIN_RESET);
}

static void Lights_Green(void)
{
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port,     LED_ROJO_Pin,     GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port,    LED_VERDE_Pin,    GPIO_PIN_SET);
}

/* ---- ganador y reset ---- */
static inline void Winner_J1(void)
{
  race_finished = 1;
  race_enabled = 0;
  HAL_GPIO_WritePin(J1_WIN_GPIO_Port, J1_WIN_Pin, GPIO_PIN_SET);
}

static inline void Winner_J2(void)
{
  race_finished = 1;
  race_enabled = 0;
  HAL_GPIO_WritePin(J2_WIN_GPIO_Port, J2_WIN_Pin, GPIO_PIN_SET);
}

static inline void Reset_Game(void)
{
  race_enabled = 0;
  starting_seq = 0;
  race_finished = 0;
  start_req = 0;

  j1_mask = 0x01; j2_mask = 0x01;
  j1_laps = 0;    j2_laps = 0;

  Lights_AllOff();
  Track_Show_J1(j1_mask);
  Track_Show_J2(j2_mask);

  HAL_GPIO_WritePin(J1_WIN_GPIO_Port, J1_WIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(J2_WIN_GPIO_Port, J2_WIN_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  Reset_Game();         // estado inicial limpio
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // si se solicitó inicio (o reinicio post victoria), ejecutar secuencia
    if (start_req) {
      start_req = 0;

      if (race_finished) {
        Reset_Game();   // reinicia antes de la nueva cuenta regresiva
      }
      Run_Starting_Lights();   // habilita la carrera al quedar en verde
    }
    // el avance y conteo de vueltas ocurren en HAL_GPIO_EXTI_Callback
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Niveles iniciales (apagar LEDs y ganadores) */
  HAL_GPIO_WritePin(GPIOC, J1_LED2_Pin|J1_WIN_Pin|J1_LED3_Pin|J1_LED4_Pin
                          |LED_AMARILLO_Pin|J2_LED4_Pin|J1_LED1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, J1_LED5_Pin|J1_LED6_Pin|J1_LED7_Pin|J2_LED1_Pin
                          |J2_LED2_Pin|J2_LED6_Pin|J2_LED5_Pin|LED_ROJO_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, J1_LED8_Pin|J2_LED7_Pin|LED_VERDE_Pin|J2_LED8_Pin
                          |J2_WIN_Pin|J2_LED3_Pin, GPIO_PIN_RESET);

  /* Salidas en puerto C (incluye amarillo y J2_LED4 PC7) */
  GPIO_InitStruct.Pin = J1_LED2_Pin|J1_WIN_Pin|J1_LED3_Pin|J1_LED4_Pin
                      |LED_AMARILLO_Pin|J2_LED4_Pin|J1_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Entradas con interrupción (si tus botones van a GND, cambia a IT_FALLING+PULLUP) */
  GPIO_InitStruct.Pin = StartRace_Pin|PJ2_Pin|PJ1_Pin;   // PC0, PC8 y (tu PJ1 en C)
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Salidas en puerto A (rojo y LEDs J2 en A) */
  GPIO_InitStruct.Pin = J1_LED5_Pin|J1_LED6_Pin|J1_LED7_Pin|J2_LED1_Pin
                      |J2_LED2_Pin|J2_LED6_Pin|J2_LED5_Pin|LED_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Salidas en puerto B (verde y LEDs J2 en B + WIN) */
  GPIO_InitStruct.Pin = J1_LED8_Pin|J2_LED7_Pin|LED_VERDE_Pin|J2_LED8_Pin
                      |J2_WIN_Pin|J2_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
