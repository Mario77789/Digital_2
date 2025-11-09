/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Nucleo1)
  ******************************************************************************
  */
 /* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ===== Debug opcional por UART2 ===== */
#define ENABLE_UART_DEBUG 0

/* ===== I2C registros con ESP32@0x30 ===== */
#define ESP32_ADDR_7B   0x30
#define REG_UP          0x01
#define REG_DN          0x02
#define REG_ALL         0x03
#define REG_FREE        0x04
#define I2C_TIMEOUT_MS  20

/* ===== IR locales ===== */
#define IR_ACTIVE_LOW   1
#define DEBOUNCE_MS     12

/* ===== DRDY en PA5 ===== */
#ifndef DRDY_Pin
#define DRDY_Pin       GPIO_PIN_5
#define DRDY_GPIO_Port GPIOA
#endif
#define DRDY_PULSE_MS  2

#define BIT(v,i)       (((v) >> (i)) & 0x1)

/* ===== WS2812 por SPI1 MOSI (PA7) ===== */
#define N_LEDS           16
#define BYTES_PER_LED     9     /* codificación 3:1 => 24*3 bits */
#define RESET_BYTES      64
#define MIDDLE_WHITE      1     /* 1 = separadores en blanco tenue */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
#if ENABLE_UART_DEBUG
  #include "usart.h"
  #define uprintf(...) do{ char __b[160]; int __n = snprintf(__b,sizeof(__b), __VA_ARGS__); \
      if(__n>0){ HAL_UART_Transmit(&huart2,(uint8_t*)__b,(uint16_t)strnlen(__b,sizeof(__b)),50);} }while(0)
#else
  #define uprintf(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
#if ENABLE_UART_DEBUG
extern UART_HandleTypeDef huart2;
#endif

static volatile uint8_t pk_up_bits = 0x00;  /* P1..P4 locales */
static volatile uint8_t pk_dn_bits = 0x00;  /* P5..P8 remotos */
static uint32_t last_scan_ms = 0;
static uint8_t  prev_disp    = 0xFF;

static uint8_t spi_buf[N_LEDS*BYTES_PER_LED + RESET_BYTES];

/* ---- Mapeo físico del 4x4 (serpentina típica) ----
   Fila 0 (arriba):  0  1  2  3
   Fila 1:           7  6  5  4
   Fila 2:           8  9 10 11
   Fila 3 (abajo):  15 14 13 12

   Ajusta estos arreglos si tu panel está invertido.
*/
static const uint8_t COL1_IDX[4] = { 0, 7,  8, 15 };  /* columna izquierda = P1..P4 */
static const uint8_t COL4_IDX[4] = { 3, 4, 11, 12 };  /* columna derecha  = P5..P8 */
static const uint8_t COL2_IDX[4] = { 1, 6,  9, 14 };  /* separador */
static const uint8_t COL3_IDX[4] = { 2, 5, 10, 13 };  /* separador */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* ===== 7 segmentos ===== */
static inline void seg_set_raw(uint8_t a_on, uint8_t b_on, uint8_t c_on,
                               uint8_t d_on, uint8_t e_on, uint8_t f_on, uint8_t g_on);
static void seg_show_digit(uint8_t d);
static void refresh_display_if_needed(uint8_t up_bits, uint8_t dn_bits);

/* ===== IR ===== */
static inline uint8_t read_ir_pin(GPIO_TypeDef* port, uint16_t pin);
static uint8_t read_P1_P4(void);

/* ===== I2C ===== */
static bool I2C_WriteReg(uint8_t reg, uint8_t val);
static bool I2C_ReadReg(uint8_t reg, uint8_t *val);

/* ===== DRDY ===== */
static void drdy_pulse(void);

/* ===== WS2812 por SPI1 ===== */
static inline uint32_t expand8_to24_3b(uint8_t v);
static inline void write24(uint8_t* dst, uint32_t v24);
static void ws2812_encode_led(int led_idx, uint8_t r, uint8_t g, uint8_t b);
static void ws2812_clear(void);
static void ws2812_show(void);
static void neopixel_update(uint8_t up_bits, uint8_t dn_bits);

/* ===== ciclo ===== */
static void do_i2c_exchange(void);

/* ================== Código ================== */
static void seg_set_raw(uint8_t a_on, uint8_t b_on, uint8_t c_on,
                        uint8_t d_on, uint8_t e_on, uint8_t f_on, uint8_t g_on)
{
  HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, a_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, b_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, c_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, d_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, e_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, f_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, g_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void seg_show_digit(uint8_t d){
  switch (d){
    case 0: seg_set_raw(1,1,1,1,1,1,0); break;
    case 1: seg_set_raw(0,1,1,0,0,0,0); break;
    case 2: seg_set_raw(1,1,0,1,1,0,1); break;
    case 3: seg_set_raw(1,1,1,1,0,0,1); break;
    case 4: seg_set_raw(0,1,1,0,0,1,1); break;
    case 5: seg_set_raw(1,0,1,1,0,1,1); break;
    case 6: seg_set_raw(1,0,1,1,1,1,1); break;
    case 7: seg_set_raw(1,1,1,0,0,0,0); break;
    case 8: seg_set_raw(1,1,1,1,1,1,1); break;
    default: seg_set_raw(1,1,1,1,0,1,1); break; /* 9 */
  }
}
static void refresh_display_if_needed(uint8_t up_bits, uint8_t dn_bits){
  uint8_t occ_up = BIT(up_bits,0)+BIT(up_bits,1)+BIT(up_bits,2)+BIT(up_bits,3);
  uint8_t occ_dn = BIT(dn_bits,0)+BIT(dn_bits,1)+BIT(dn_bits,2)+BIT(dn_bits,3);
  int disp = 8 - (occ_up + occ_dn);
  if (disp < 0) { disp = 0; }
  if (disp > 9) { disp = 9; }
  if ((uint8_t)disp != prev_disp){
    seg_show_digit((uint8_t)disp);
    prev_disp = (uint8_t)disp;
    uprintf("[N1][7SEG] disponibles=%d (occ_up=%u, occ_dn=%u)\r\n", disp, occ_up, occ_dn);
  }
}

static inline uint8_t read_ir_pin(GPIO_TypeDef* port, uint16_t pin){
#if IR_ACTIVE_LOW
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
#else
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)   ? 1u : 0u;
#endif
}
static uint8_t read_P1_P4(void){
  uint32_t now = HAL_GetTick();
  if ((now - last_scan_ms) < DEBOUNCE_MS) return (pk_up_bits & 0x0F);
  last_scan_ms = now;

  uint8_t p1 = read_ir_pin(Lugar1_GPIO_Port, Lugar1_Pin);
  uint8_t p2 = read_ir_pin(Lugar2_GPIO_Port, Lugar2_Pin);
  uint8_t p3 = read_ir_pin(Lugar3_GPIO_Port, Lugar3_Pin);
  uint8_t p4 = read_ir_pin(Lugar4_GPIO_Port, Lugar4_Pin);
  return (uint8_t)((p1<<0)|(p2<<1)|(p3<<2)|(p4<<3));
}

static bool I2C_WriteReg(uint8_t reg, uint8_t val){
  return (HAL_I2C_Mem_Write(&hi2c1, (ESP32_ADDR_7B<<1), reg,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK);
}
static bool I2C_ReadReg(uint8_t reg, uint8_t *val){
  return (HAL_I2C_Mem_Read(&hi2c1, (ESP32_ADDR_7B<<1), reg,
                           I2C_MEMADD_SIZE_8BIT, val, 1, I2C_TIMEOUT_MS) == HAL_OK);
}

static void drdy_pulse(void){
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_SET);
  HAL_Delay(DRDY_PULSE_MS);
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);
}

/* ---- Codificación 3:1 @ ~2.6 Mbps (1->110, 0->100) ---- */
static inline uint32_t expand8_to24_3b(uint8_t v) {
  uint32_t out = 0;
  for (int i=7; i>=0; --i){
    out <<= 3;
    out |= (v & (1u<<i)) ? 0b110 : 0b100;
  }
  return out & 0xFFFFFFu;
}
static inline void write24(uint8_t* dst, uint32_t v24){
  dst[0] = (uint8_t)((v24 >> 16) & 0xFF);
  dst[1] = (uint8_t)((v24 >>  8) & 0xFF);
  dst[2] = (uint8_t)( v24        & 0xFF);
}
static void ws2812_encode_led(int led_idx, uint8_t r, uint8_t g, uint8_t b){
  uint32_t G = expand8_to24_3b(g);
  uint32_t R = expand8_to24_3b(r);
  uint32_t B = expand8_to24_3b(b);
  int base = led_idx * BYTES_PER_LED;
  write24(&spi_buf[base+0], G);
  write24(&spi_buf[base+3], R);
  write24(&spi_buf[base+6], B);
}
static void ws2812_clear(void){
  for (int i=0;i<N_LEDS;i++) ws2812_encode_led(i, 0,0,0);
}
static void ws2812_show(void){
  for (int i=0;i<RESET_BYTES;i++) spi_buf[N_LEDS*BYTES_PER_LED + i] = 0x00;
  HAL_SPI_Transmit(&hspi1, spi_buf, sizeof(spi_buf), 100);
  HAL_Delay(1);
}

/* ---- Mapeo pedido: Columna 1 = P1..P4 ; Columna 4 = P5..P8 ; columnas 2 y 3 = separadores ---- */
static inline uint8_t B4(uint8_t v, int i){ return (v>>i)&1u; }
static void neopixel_update(uint8_t up_bits, uint8_t dn_bits){
  const uint8_t Rr=32, Rg=0,  Rb=0;   /* rojo tenue ocupado */
  const uint8_t Gr=0,  Gg=22, Gb=0;   /* verde tenue libre  */
  const uint8_t Wr=8,  Wg=8,  Wb=8;   /* blanco separador   */

  ws2812_clear();

  /* columna 1 -> P1..P4 */
  for (int i=0;i<4;i++){
    uint8_t idx = COL1_IDX[i];
    if (B4(up_bits,i)) ws2812_encode_led(idx, Rr,Rg,Rb);
    else               ws2812_encode_led(idx, Gr,Gg,Gb);
  }

  /* separadores (col 2 y 3) */
#if MIDDLE_WHITE
  for (int i=0;i<4;i++){ ws2812_encode_led(COL2_IDX[i], Wr,Wg,Wb); }
  for (int i=0;i<4;i++){ ws2812_encode_led(COL3_IDX[i], Wr,Wg,Wb); }
#endif

  /* columna 4 -> P5..P8 */
  for (int i=0;i<4;i++){
    uint8_t idx = COL4_IDX[i];
    if (B4(dn_bits,i)) ws2812_encode_led(idx, Rr,Rg,Rb);
    else               ws2812_encode_led(idx, Gr,Gg,Gb);
  }

  ws2812_show();
}

static void do_i2c_exchange(void){
  uint8_t old_up = pk_up_bits & 0x0F;
  uint8_t old_dn = pk_dn_bits & 0x0F;

  /* 1) locales */
  uint8_t new_up = read_P1_P4() & 0x0F;
  pk_up_bits = new_up;

  /* 2) avisar si cambió */
  static uint8_t last_sent_up = 0xFF;
  if (new_up != last_sent_up){
    (void)I2C_WriteReg(REG_UP, new_up);
    last_sent_up = new_up;
  }

  /* 3) leer remotos */
  uint8_t all=0xFF;
  if (I2C_ReadReg(REG_ALL, &all)){
    uint8_t up = (all & 0x0F);
    uint8_t dn = (all >> 4) & 0x0F;
    pk_dn_bits = (dn & 0x0F);
    if (up != pk_up_bits) pk_up_bits = up;
  }

  /* 4) DRDY si cambió */
  if ((old_up != pk_up_bits) || (old_dn != pk_dn_bits)) drdy_pulse();

  /* 5) display + leds */
  refresh_display_if_needed(pk_up_bits, pk_dn_bits);
  neopixel_update(pk_up_bits & 0x0F, pk_dn_bits & 0x0F);
}

/* ================== MAIN ================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  pk_up_bits = read_P1_P4();
  pk_dn_bits = 0x00;
  prev_disp  = 0xFF;
  refresh_display_if_needed(pk_up_bits, pk_dn_bits);

  ws2812_clear();
  ws2812_show();
  neopixel_update(pk_up_bits & 0x0F, pk_dn_bits & 0x0F);

  while (1){
    do_i2c_exchange();
    HAL_Delay(15);
  }
}

/* ================== RCC ================== */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){ Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){ Error_Handler(); }
}

/* ================== I2C1 ================== */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK){ Error_Handler(); }
}

/* ================== SPI1 (WS2812) ================== */
static void MX_SPI1_Init(void)
{
  /* Transmit Only Master por MOSI (PA7) */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; /* ≈2.6 Mbit/s */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK){ Error_Handler(); }
}

/* ================== GPIO ================== */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, e_Pin|b_Pin|d_Pin|c_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, f_Pin|g_Pin|a_Pin, GPIO_PIN_RESET);

  /* IR: entradas con pulldown */
  GPIO_InitStruct.Pin = Lugar1_Pin|Lugar2_Pin|Lugar3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Lugar4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Lugar4_GPIO_Port, &GPIO_InitStruct);

  /* 7 segmentos */
  GPIO_InitStruct.Pin = e_Pin|b_Pin|d_Pin|c_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = f_Pin|g_Pin|a_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DRDY */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);
}

/* ================== Error ================== */
void Error_Handler(void)
{
  __disable_irq();
  while (1){ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
