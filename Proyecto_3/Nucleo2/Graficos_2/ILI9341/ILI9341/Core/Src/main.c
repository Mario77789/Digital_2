/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Parqueo_Matic en ILI9341 (320x240, landscape) + I2C1 maestro -> ESP32@0x30
  *
  * - LCD paralelo 8 bits (ILI9341) + overlay del carro con color-key 0xFFFF.
  * - IR locales en PC0/PC2/PC3/PC5 -> P5..P8 (1=ocupado).
  * - I2C1 MASTER hacia ESP32 (esclavo 0x30):
  *       * REG_DN  (0x02, W) = P5..P8 (b0..b3) [se envía solo si cambia]
  *       * REG_ALL (0x03, R) = (P5..P8<<4) | (P1..P4)
  * - Redibujo INCREMENTAL: sólo slots que cambian + panel si cambia el conteo.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "ili9341.h"
#include "bitmaps.h"

/* ---- Dígitos grandes tipo display (bitmaps) ----
   NOTA: NO declarar DIGITS_MAP aquí para evitar conflicto; ya viene de bitmaps.h */
#define DIG_H  96
static const uint8_t DIG_W[10] = { 61,60,58,59,60,61,60,58,58,60 };
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IR_ACTIVE_LOW   0     /* 1: sensor baja a 0V cuando detecta (ocupado); 0: activo en HIGH */
#define DEBOUNCE_MS     12

/* ===== Orden de bits P5..P8 (local) =====
   1: b0=P5, b1=P6, b2=P7, b3=P8
   0: b0=P8, b1=P7, b2=P6, b3=P5  */
#define MAP_DN_LSB_IS_P5   1

/* LCD */
#define LCD_W 320
#define LCD_H 240

/* Colores */
#define COL_BG  0xFFFF
#define COL_FG  0x0000

/* Dimensiones originales de bitmaps */
#define PK_W0   63
#define PK_H0  100
#define SEM_W0  40
#define SEM_H0  20
#define CAR_W0  43
#define CAR_H0  92

/* Dimensiones escaladas */
#define PK_W   50    /* ~80% */
#define PK_H   80
#define CAR_W  26    /* ~60% */
#define CAR_H  55
#define SEM_W  36    /* ~90% */
#define SEM_H  18

/* Márgenes / layout */
#define LM   3
#define RM   3
#define TITLE_H 14
#define TITLE_Y 2
#define TITLE_SHIFT_LEFT  6
#define SEM_LABEL_PAD   4
#define ROW_GAP         (SEM_H + 14)
#define EXTRA_ROW_DROP  (SEM_H)
#define Y_UP    (TITLE_H + 2)
#define Y_DOWN (Y_UP + PK_H + ROW_GAP + EXTRA_ROW_DROP)

/* Panel (contador) */
#define PANEL_W_MAX  61
#define PANEL_SHIFT_LEFT   8
#define PANEL_X   (LCD_W - RM - PANEL_W_MAX - PANEL_SHIFT_LEFT)
#define DIG_Y     ((LCD_H - 32)/2)
#define PANEL_TXT_X   (PANEL_X + 4)
#define PANEL_TXT_Y   (DIG_Y - 14)

/* --- I2C protocolo hacia ESP32 esclavo @0x30 --- */
#define ESP32_ADDR      (0x30 << 1)      /* 7-bit -> dirección HAL */
#define REG_UP          0x01
#define REG_DN          0x02
#define REG_ALL         0x03
#define REG_FREE        0x04
#define I2C_TIMEOUT_MS  20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline int PK_X(int i){ return LM + i*PK_W; }
static inline int CAR_X(int i){ return PK_X(i) + (PK_W - CAR_W)/2; }
static inline int CAR_Y(int row_y){ return row_y + (PK_H - CAR_H)/2; }
static inline int SEM_X(int i){ return PK_X(i) + (PK_W - SEM_W)/2; }
static inline int SEM_Y_UP_LABEL(void){ return Y_UP   + PK_H + SEM_LABEL_PAD; }
static inline int SEM_Y_DOWN_LABEL(void){ return Y_DOWN - SEM_H - SEM_LABEL_PAD; }

static inline void UI_Clear(uint16_t color){ LCD_Clear(color); }
static inline void UI_TextSmall(const char* s, int x, int y, uint16_t fg, uint16_t bg){ LCD_Print((char*)s, x, y, 1, fg, bg); }
static inline void UI_TextBig(const char* s, int x, int y, uint16_t fg, uint16_t bg){ LCD_Print((char*)s, x, y, 3, fg, bg); }
static void UI_TextCenteredOffset(const char* s, int y, int xoff, uint16_t fg, uint16_t bg){
  int x = (LCD_W - (int)strlen(s)*6)/2 + xoff; if (x<0) x=0;
  UI_TextSmall(s, x, y, fg, bg);
}
static inline void UI_Bitmap(int x,int y,int w,int h,const uint8_t* data){ LCD_Bitmap(x,y,w,h,(uint8_t*)data); }

static inline uint16_t rd565HL(const uint8_t* p){ return ((uint16_t)p[0]<<8) | p[1]; }
static inline void     wr565HL(uint8_t* p, uint16_t c){ p[0]=(uint8_t)(c>>8); p[1]=(uint8_t)c; }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim11;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Buffers escalados */
static uint8_t g_pk_up[PK_W*PK_H*2];
static uint8_t g_pk_dn[PK_W*PK_H*2];
static uint8_t g_car  [CAR_W*CAR_H*2];
static uint8_t g_semR [SEM_W*SEM_H*2];
static uint8_t g_semV [SEM_W*SEM_H*2];
static uint8_t g_car_comp[CAR_W*CAR_H*2];

static const uint8_t* PK_UP = g_pk_up;
static const uint8_t* PK_DN = g_pk_dn;
static const uint8_t* CAR   = g_car;
static const uint8_t* SEM_R = g_semR;
static const uint8_t* SEM_V = g_semV;

/* Estado (refresco UI) */
static uint8_t prev_up_bits = 0xFF;   // fuerza primer dibujo
static uint8_t prev_dn_bits = 0xFF;
static uint8_t prev_disp     = 0xFF;

/* Debounce IR */
static uint32_t last_scan_ms = 0;

/* Nibbles actuales */
static uint8_t pk_up_bits = 0x00;   /* P1..P4 (arriba) — 1=ocupado (desde ESP32) */
static uint8_t pk_dn_bits = 0x00;   /* P5..P8 (abajo)  — 1=ocupado (LOCAL) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void Scale565_HL_to_HL(const uint8_t* srcHL, int sw, int sh, uint8_t* dstHL, int dw, int dh){
  for (int y = 0; y < dh; ++y) {
    int sy = (y * sh) / dh;
    int srow = sy * sw * 2;
    int drow = y  * dw * 2;
    for (int x = 0; x < dw; ++x) {
      int sx = (x * sw) / dw;
      int si = srow + sx*2;
      int di = drow + x*2;
      dstHL[di + 0] = srcHL[si + 0];
      dstHL[di + 1] = srcHL[si + 1];
    }
  }
}
static void Assets_PrepareScaled(void){
  Scale565_HL_to_HL(ParqueoArriba_map, PK_W0, PK_H0, g_pk_up, PK_W, PK_H);
  Scale565_HL_to_HL(ParqueoABAJO_map,  PK_W0, PK_H0, g_pk_dn, PK_W, PK_H);
  Scale565_HL_to_HL(Carroparqueo_map,  CAR_W0, CAR_H0, g_car,  CAR_W, CAR_H);
  Scale565_HL_to_HL(SemaforoROJO_map,  SEM_W0, SEM_H0, g_semR, SEM_W, SEM_H);
  Scale565_HL_to_HL(SemaforoVERDE_map, SEM_W0, SEM_H0, g_semV, SEM_W, SEM_H);
}

#define CK_WHITE  0xFFFF
static void Compose_Car_Over_Slot(const uint8_t* pkHL, int dx, int dy){
  for (int y=0; y<CAR_H; ++y){
    const uint8_t* pkp = pkHL + ((dy + y) * PK_W * 2) + (dx * 2);
    uint8_t* dp = g_car_comp + (y * CAR_W * 2);
    const uint8_t* cp = CAR + (y * CAR_W * 2);
    for (int x=0; x<CAR_W; ++x){
      uint16_t cC = rd565HL(cp);
      if (cC != CK_WHITE) wr565HL(dp, cC);
      else                wr565HL(dp, rd565HL(pkp));
      dp += 2; cp += 2; pkp += 2;
    }
  }
}

static void DrawSlotBase(int col, int row_y){
  const uint8_t* pk  = (row_y==Y_UP) ? PK_UP : PK_DN;
  UI_Bitmap(PK_X(col), row_y, PK_W, PK_H, pk);
}
static void DrawCarOverlay(int col, int row_y){
  const uint8_t* pk  = (row_y==Y_UP) ? PK_UP : PK_DN;
  int cx = CAR_X(col), cy = CAR_Y(row_y);
  int dx = cx - PK_X(col), dy = cy - row_y;
  Compose_Car_Over_Slot(pk, dx, dy);
  UI_Bitmap(cx, cy, CAR_W, CAR_H, g_car_comp);
}
static void DrawSemLabel(int col, int row_y, bool ocupado){
  const uint8_t* sem = ocupado ? SEM_R : SEM_V;
  int sx = SEM_X(col);
  int sy = (row_y==Y_UP) ? SEM_Y_UP_LABEL() : SEM_Y_DOWN_LABEL();
  UI_Bitmap(sx, sy, SEM_W, SEM_H, sem);
}
static void DrawParqueoSlot_All(int col, int row_y, bool ocupado){
  DrawSlotBase(col, row_y);
  if (ocupado) DrawCarOverlay(col, row_y);
  DrawSemLabel(col, row_y, ocupado);
}

/* === Panel con dígito bitmap centrado === */
static void DrawPanelDigit(uint8_t value){
  if (value > 9) value = 9;  /* display de un solo dígito 0..9 */
  UI_TextSmall("Parqueos disponibles", PANEL_TXT_X, PANEL_TXT_Y, COL_FG, COL_BG);
  int dw = DIG_W[value];
  int dx = PANEL_X + (PANEL_W_MAX - dw)/2; if (dx<0) dx=0;
  UI_Bitmap(dx, DIG_Y, dw, DIG_H, DIGITS_MAP[value]);
}

static inline bool bit_is_set(uint8_t v, int i){ return ((v >> i) & 1) != 0; }

static void UI_RefreshDiff(uint8_t new_up_bits, uint8_t new_dn_bits){
  for (int i=0; i<4; ++i) {
    bool old_up = bit_is_set(prev_up_bits, i);
    bool new_up = bit_is_set(new_up_bits, i);
    if (old_up != new_up) DrawParqueoSlot_All(i, Y_UP, new_up);
  }
  for (int i=0; i<4; ++i) {
    bool old_dn = bit_is_set(prev_dn_bits, i);
    bool new_dn = bit_is_set(new_dn_bits, i);
    if (old_dn != new_dn) DrawParqueoSlot_All(i, Y_DOWN, new_dn);
  }
  int total_free = 0;
  for (int i=0;i<4;i++){
    total_free += (!bit_is_set(new_up_bits, i)) + (!bit_is_set(new_dn_bits, i));
  }
  uint8_t disp = (uint8_t)total_free;
  if (disp != prev_disp) { DrawPanelDigit(disp); prev_disp = disp; }
  prev_up_bits = new_up_bits;
  prev_dn_bits = new_dn_bits;
}

/* -------- Helpers I2C (master) -------- */
static bool I2C_WriteReg(uint8_t reg, uint8_t val){
  return (HAL_I2C_Mem_Write(&hi2c1, ESP32_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK);
}
static bool I2C_ReadReg(uint8_t reg, uint8_t *val){
  return (HAL_I2C_Mem_Read(&hi2c1, ESP32_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, I2C_TIMEOUT_MS) == HAL_OK);
}

/* -------- Lectura IR P5..P8 (debounce + mapeo) -------- */
static inline uint8_t read_pin(GPIO_TypeDef* port, uint16_t pin){
#if IR_ACTIVE_LOW
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
#else
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)   ? 1u : 0u;
#endif
}
static inline uint8_t pack_P5P8(uint8_t p5,uint8_t p6,uint8_t p7,uint8_t p8){
#if MAP_DN_LSB_IS_P5
  return (uint8_t)((p5<<0)|(p6<<1)|(p7<<2)|(p8<<3));
#else
  return (uint8_t)((p5<<3)|(p6<<2)|(p7<<1)|(p8<<0));
#endif
}
static uint8_t ReadIR_P5P8(void){
  uint32_t now = HAL_GetTick();
  if ((now - last_scan_ms) < DEBOUNCE_MS) return pk_dn_bits & 0x0F;
  last_scan_ms = now;
  uint8_t p5 = read_pin(IR_P5_GPIO_Port, IR_P5_Pin);
  uint8_t p6 = read_pin(IR_P6_GPIO_Port, IR_P6_Pin);
  uint8_t p7 = read_pin(IR_P7_GPIO_Port, IR_P7_Pin);
  uint8_t p8 = read_pin(IR_P8_GPIO_Port, IR_P8_Pin);
  return pack_P5P8(p5,p6,p7,p8) & 0x0F;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  LCD_Init();
  Assets_PrepareScaled();

  /* Dibujo inicial (todo libre) */
  {
    bool occ_up[4] = {false,false,false,false};
    bool occ_dn[4] = {false,false,false,false};
    uint8_t disp = 8;
    UI_Clear(COL_BG);
    UI_TextCenteredOffset("Parqueo_Matic", TITLE_Y, -TITLE_SHIFT_LEFT, COL_FG, COL_BG);
    for (int i=0;i<4;i++) DrawParqueoSlot_All(i, Y_UP,   occ_up[i]);
    for (int i=0;i<4;i++) DrawParqueoSlot_All(i, Y_DOWN, occ_dn[i]);
    DrawPanelDigit(disp);
    prev_up_bits = 0x00; prev_dn_bits = 0x00; prev_disp = disp;
  }
  /* USER CODE END 2 */

  while (1)
  {
    /* 1) P5..P8 SIEMPRE locales */
    uint8_t new_dn_local = ReadIR_P5P8();

    /* 2) Enviar REG_DN si cambió */
    static uint8_t dn_prev_sent = 0xFF;
    if ((new_dn_local & 0x0F) != (dn_prev_sent & 0x0F)) {
      (void)I2C_WriteReg(REG_DN, (new_dn_local & 0x0F));
      dn_prev_sent = new_dn_local & 0x0F;
    }

    /* 3) Leer REG_ALL y tomar SOLO P1..P4 (nibble bajo).
          Si falla la lectura: P1..P4 = 0. */
    uint8_t all = 0xFF;
    if (I2C_ReadReg(REG_ALL, &all)) {
      pk_up_bits = (all & 0x0F);        /* P1..P4 (arriba) desde ESP32 */
    } else {
      pk_up_bits = 0x00;                /* sin dato del ESP32 → arriba=0 */
    }

    /* 4) Para la UI, abajo SIEMPRE el local recién leído */
    pk_dn_bits = new_dn_local & 0x0F;

    /* 5) Refresco incremental */
    UI_RefreshDiff(pk_up_bits, pk_dn_bits);

    HAL_Delay(50); /* ~20 Hz */
  }
}

/* === Resto: init HAL tal como lo tenías === */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM11_Init(void)
{
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* IR P5..P8 como INPUT (pull-up según tu HW) */
  GPIO_InitStruct.Pin = IR_P5_Pin|IR_P6_Pin|IR_P7_Pin|IR_P8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LCD control/data (como tenías) */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin|LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin|LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
