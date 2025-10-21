/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Bad Ice Cream — 2 jugadores + SD Scores + Help + Credits
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>   /* abs */
#include "ili9341.h"

#include "audio_pwm.h"
#include "song_loop.h"
#include "uart_gamepad.h"
#include "ui_menu.h"
#include "bitmaps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ST_MENU = 0,
  ST_PLAY,
  ST_END,
  /* nuevas escenas del menú */
  ST_MENU_SCORES,
  ST_MENU_HELP,
  ST_MENU_CREDITS
} AppState;

typedef struct { const uint8_t *data; int w,h; int columns; } Sheet;

typedef struct {
  uint8_t   active;
  const Sheet *sheet;
  const Sheet *resume_sheet;
  uint32_t  t_start;
  uint32_t  duration_ms;
} OneShot;

typedef enum { D_NONE=0, D_RIGHT, D_DOWN, D_LEFT, D_UP } Dir;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P2_SAFE_SINGLEFRAME 0

#define SONG_PATH_MENU  "song_bad_ice_cream.c"
#define SONG_PATH_PLAY  "converted_bad_ice_cream_ingame.c"
#define SONG_SPEED_X    3

#define LCD_W         320
#define LCD_H         240
#define TILE_W        10
#define TILE_H        14
#define MAP_COLS      (LCD_W / TILE_W)   // 32
#define MAP_ROWS      (LCD_H / TILE_H)   // 17

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0

#define P_STEP        2
#define E_STEP        ((P_STEP*2)/3)
#define ANIM_MS       120

#define BAN_W         60
#define BAN_H         60
#define VIDA_W        24
#define VIDA_H        24

/* hueco central libre */
#define C_TX0  12
#define C_TX1  19
#define C_TY0   6
#define C_TY1  10

/* archivo de puntajes */
#define SCORES_FILE      "scores.txt"
#define MAX_LINES_SHOWN  8
#define MAX_LINE_LEN     64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP(v,a,b) ( ((v) < (a)) ? (a) : ( ((v) > (b)) ? (b) : (v) ) )
static inline bool RectIntersects(int x1,int y1,int w1,int h1,int x2,int y2,int w2,int h2){
  return !(x1+w1<=x2 || x2+w2<=x1 || y1+h1<=y2 || y2+h2<=y1);
}

/* === Texto reducido (1/3 del tamaño previo) ===
   Firma de tu LCD_Print:
   void LCD_Print(char *text, int x, int y, int fontSize, int color, int background)
   Usamos fontSize = 1 (small). */
static inline void DrawTextC(const char* s, int x, int y, uint16_t fg, uint16_t bg){
  LCD_Print((char*)s, x, y, 1, fg, bg);
}

/* Helper local renombrado para no chocar con la lib: */
static inline void UI_FillRect(int x,int y,int w,int h,uint16_t c){
  static uint16_t line[320];
  if (w > 320) w = 320;
  for (int i = 0; i < w; i++) line[i] = c;
  for (int r = 0; r < h; r++) LCD_Bitmap(x, y + r, w, 1, (uint8_t*)line);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim10;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static FATFS s_fs;
static AppState app_state = ST_MENU;

/* Hojas/sprites */
static const Sheet P1_S_F = { Jugador1_F, 20, 27, 2 };
static const Sheet P1_S_B = { Jugador1_B, 24, 26, 2 };
static const Sheet P1_S_L = { Jugador1_L, 24, 32, 2 };
static const Sheet P1_S_R = { Jugador1_R, 28, 32, 2 };
static const Sheet P1_S_C = { Jugador1_Crear,  34, 40, 3 };
static const Sheet P1_S_X = { Jugador1_Romper, 26, 32, 3 };

static const Sheet P2_S_F = { Jugador2_F, 24, 32, 3 };
static const Sheet P2_S_B = { Jugador2_B, 24, 30, 3 };
static const Sheet P2_S_L = { Jugador2_L, 22, 30, 3 };
static const Sheet P2_S_R = { Jugador2_R, 22, 32, 3 };
static const Sheet P2_S_C = { Jugador2_Crear,  34, 42, 3 };
static const Sheet P2_S_X = { Jugador2_Romper, 26, 30, 3 };

static const Sheet EN_S_F  = { enemigo_F, 30, 26, 3 }; /* bajando */
static const Sheet EN_S_B  = { enemigo_B, 30, 26, 3 }; /* subiendo */
static const Sheet EN_S_L  = { enemigo_L, 28, 26, 3 };
static const Sheet EN_S_R  = { enemigo_R, 28, 26, 3 };

static uint8_t g_map[MAP_ROWS][MAP_COLS];    /* 1=hielo, 0=vacío */
static uint8_t g_fixed[MAP_ROWS][MAP_COLS];  /* 1=contorno fijo, intocable */

/* P1 */
static int p1_x, p1_y; static const Sheet *p1_sh; static int p1_fr; static uint32_t p1_tanim;
static int p1_score = 0; static uint8_t p1_alive = 1;
static OneShot p1_act; static uint8_t p1_A_prev=0, p1_B_prev=0;
static Dir p1_dir = D_DOWN;

/* P2 */
static int p2_x, p2_y; static const Sheet *p2_sh; static int p2_fr; static uint32_t p2_tanim;
static int p2_score = 0; static uint8_t p2_alive = 1;
static OneShot p2_act; static uint8_t p2_A_prev=0, p2_B_prev=0;
static Dir p2_dir = D_DOWN;

/* Enemigo */
static int en_x, en_y; static const Sheet *en_sh; static int en_fr; static uint32_t en_tanim;
static Dir en_dir = D_RIGHT;

/* BANANO escala a TILE_W×TILE_H */
static const uint8_t* g_banana_src = (const uint8_t*)banano;
static int g_ban_src_w = BAN_W, g_ban_src_h = BAN_H;
static uint16_t g_banana_scaled[BAN_W * BAN_H];
static const uint8_t* g_banana_draw = (const uint8_t*)banano;
static int g_ban_draw_w = BAN_W, g_ban_draw_h = BAN_H;

/* Bananos */
typedef struct { int x,y; uint8_t alive; } Banana;
#define MAX_BANANAS  32
static Banana s_bananas[MAX_BANANAS];
static int s_nban=0;

/* Vida (escalado a bloque) */
static uint16_t vida1_scaled[TILE_W*TILE_H];
static uint16_t vida2_scaled[TILE_W*TILE_H];
static const uint8_t* vida1_draw = (const uint8_t*)contadorJ1;
static const uint8_t* vida2_draw = (const uint8_t*)contadorJ2;
static int vida_draw_w = VIDA_W, vida_draw_h = VIDA_H;
#define VIDA_SRC_W 24
#define VIDA_SRC_H 24

/* Fin de juego */
static uint8_t g_game_over = 0;
static uint8_t g_score_saved = 0;
/* USER CODE END PV */

/* Prototypes ----------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM11_Init(void);

/* USER CODE BEGIN PFP */
/*** Mapa ***/
static void Map_Generate(void);
static void Map_Draw(void);
static void PatchFromMap(int x, int y, int w, int h);
static void DrawBlackTile(int tx, int ty);
static bool RectCollidesMap(int x, int y, int w, int h);

/*** HUD / texto ***/
static void HUD_Draw(void);
static void HUD_RedrawIfOverlapped(int x,int y,int w,int h);

/*** BANANO ***/
static void Banana_ResizeTo(int dw, int dh);

/*** Vida (resize) ***/
static void LifeIcon_ResizeTo(int dw, int dh);

/*** Bananos ***/
static void Bananas_Init(void);
static void Bananas_DrawAll(void);
static void Bananas_RedrawIfOverlapped(int x,int y,int w,int h);
static void Bananas_CheckPickup(int jx,int jy,int jw,int jh,int who);
static int  Bananas_Left(void);

/*** Acciones A/B sobre líneas ***/
static Dir SheetToDir(const Sheet *s, Dir fallback);
static bool EntityBlocksTile(int tx,int ty, int ex,int ey,int ew,int eh);
static void Line_RemoveFrom(int tx,int ty, Dir d);
static void Line_CreateFrom(int tx,int ty, Dir d, int ex,int ey,int ew,int eh, int ex2,int ey2,int ew2,int eh2);
static void TryActionErase_Create(const char who);

/*** Jugadores / enemigo ***/
static void DecideAxisMove(const PlayerInput *pi, int *dx, int *dy, const Sheet **out_sheet,
                           const Sheet *S_F,const Sheet *S_B,const Sheet *S_L,const Sheet *S_R);
static void TryMovePlayer(int *x,int *y,int w,int h,int dx,int dy,int ox,int oy,int ow,int oh);
static void Players_HandleInput(void);

static void Enemy_InitPatrol(void);
static void Enemy_Tick(void);

/* Perseguidor */
static void Enemy_GetStartTile(int *sx,int *sy);
static void Enemy_GetPlayerTargetTile(int *gx,int *gy, int *has_target);
static bool TilePassableForEnemy(int tx,int ty, const Sheet* sh);
static Dir  BFS_NextDirForEnemy(int sx,int sy,int gx,int gy, const Sheet* sh);
static void Enemy_FaceTowardsPixel(int px,int py);
static bool Enemy_TryStep(Dir d, int *nx, int *ny);

static void GoToPlay(void);
static void ShowEndScreen(void);

/* Scores/Help/Credits */
static void Scores_Append(int p1, int p2);
static void Scores_ShowScreen(void);
static void Help_ShowScreen(void);
static void Credits_ShowScreen(void);
static bool Wait_B_ToGoMenu(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void GoToPlay(void)
{
  SongLoop_Stop();
  f_mount(&s_fs, "", 1);
  SongLoop_Start(SONG_PATH_PLAY);

  LCD_Clear(COLOR_BLACK);
  Map_Generate();
  Map_Draw();

  LifeIcon_ResizeTo(TILE_W, TILE_H);
  HUD_Draw();

  Banana_ResizeTo(TILE_W, TILE_H);
  Bananas_Init();
  Bananas_DrawAll();

  p1_score = p2_score = 0;
  p1_alive = p2_alive = 1;
  memset(&p1_act,0,sizeof(p1_act)); memset(&p2_act,0,sizeof(p2_act));
  p1_A_prev=p1_B_prev=0; p2_A_prev=p2_B_prev=0;
  p1_dir = D_DOWN; p2_dir = D_DOWN;

  p1_sh = &P1_S_F; p1_fr=0; p1_tanim=HAL_GetTick();
  p2_sh = &P2_S_F; p2_fr=0; p2_tanim=HAL_GetTick();

  int midx = LCD_W/2, midy = LCD_H/2;
  p1_x = midx - p1_sh->w - 4; p1_y = midy - p1_sh->h/2;
  p2_x = midx + 4;            p2_y = midy - p2_sh->h/2;

  if (RectCollidesMap(p1_x,p1_y,p1_sh->w,p1_sh->h)) { p1_x = CLAMP(p1_x-10,0,LCD_W-p1_sh->w); }
  if (RectCollidesMap(p2_x,p2_y,p2_sh->w,p2_sh->h)) { p2_x = CLAMP(p2_x+10,0,LCD_W-p2_sh->w); }
  if (RectIntersects(p1_x,p1_y,p1_sh->w,p1_sh->h,p2_x,p2_y,p2_sh->w,p2_sh->h)) { p2_x = CLAMP(p2_x+12,0,LCD_W-p2_sh->w); }

  if (p1_alive) LCD_Sprite(p1_x,p1_y, p1_sh->w,p1_sh->h, (uint8_t*)p1_sh->data, p1_sh->columns, 0, 0, 0);
  if (p2_alive) LCD_Sprite(p2_x,p2_y, p2_sh->w,p2_sh->h, (uint8_t*)p2_sh->data, p2_sh->columns, 0, 0, 0);

  en_dir = D_RIGHT;
  en_sh=&EN_S_R; en_fr=0; en_tanim=HAL_GetTick();
  Enemy_InitPatrol();
  LCD_Sprite(en_x,en_y, en_sh->w,en_sh->h, (uint8_t*)en_sh->data, en_sh->columns, 0, 0, 0);

  g_game_over = 0;
  g_score_saved = 0;
  app_state = ST_PLAY;
}
/* USER CODE END 0 */

/* Entry --------------------------------------------------------------------*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();

  LCD_Init();
  AudioPWM_Init(&htim11);
  Gamepad_Init(&huart3);
  Menu_Init();

  SongLoop_Init(&htim10, SONG_SPEED_X);
  if (f_mount(&s_fs, "", 1) == FR_OK) {
    SongLoop_Start(SONG_PATH_MENU);
  }

  while (1)
  {
    if (app_state == ST_MENU) {
      Menu_Tick();
      if (Menu_ActionPressed()) {
        int idx = Menu_GetIndex();
        if (idx == 0) {
          GoToPlay();
        } else if (idx == 1) {
          Scores_ShowScreen();
          app_state = ST_MENU_SCORES;
        } else if (idx == 2) {
          Help_ShowScreen();
          app_state = ST_MENU_HELP;
        } else if (idx == 3) {
          Credits_ShowScreen();
          app_state = ST_MENU_CREDITS;
        }
      }
    }
    else if (app_state == ST_MENU_SCORES) {
      if (Wait_B_ToGoMenu()) { Menu_Init(); app_state = ST_MENU; }
    }
    else if (app_state == ST_MENU_HELP) {
      if (Wait_B_ToGoMenu()) { Menu_Init(); app_state = ST_MENU; }
    }
    else if (app_state == ST_MENU_CREDITS) {
      if (Wait_B_ToGoMenu()) { Menu_Init(); app_state = ST_MENU; }
    }
    else if (app_state == ST_PLAY) {
      Players_HandleInput();
      Enemy_Tick();

      if (!g_game_over) {
        int bananas_left = Bananas_Left();
        if (bananas_left == 0) { g_game_over = 1; }
        if (!p1_alive && !p2_alive) { g_game_over = 2; }
        if (g_game_over) {
          if (!g_score_saved) { Scores_Append(p1_score, p2_score); g_score_saved = 1; }
          ShowEndScreen();
          app_state = ST_END;
          HAL_Delay(1);
          continue;
        }
      }

      uint32_t now = HAL_GetTick();
      if (p1_alive && (now - p1_tanim >= ANIM_MS)) { p1_tanim=now; p1_fr=(p1_fr+1)%p1_sh->columns; LCD_Sprite(p1_x,p1_y,p1_sh->w,p1_sh->h,(uint8_t*)p1_sh->data,p1_sh->columns,p1_fr,0,0); }
      if (p2_alive && (now - p2_tanim >= ANIM_MS)) { p2_tanim=now; p2_fr=(p2_fr+1)%p2_sh->columns; LCD_Sprite(p2_x,p2_y,p2_sh->w,p2_sh->h,(uint8_t*)p2_sh->data,p2_sh->columns,p2_fr,0,0); }
      if (now - en_tanim >= ANIM_MS) { en_tanim=now; en_fr=(en_fr+1)%en_sh->columns; LCD_Sprite(en_x,en_y,en_sh->w,en_sh->h,(uint8_t*)en_sh->data,en_sh->columns,en_fr,0,0); }
    }
    else if (app_state == ST_END) {
      PlayerInput i1, i2; __disable_irq(); Gamepad_GetP1(&i1); Gamepad_GetP2(&i2); __enable_irq();
      if (i1.A || i2.A) {
        GoToPlay();
      } else if (i1.B || i2.B) {
        SongLoop_Stop();
        Menu_Init();
        SongLoop_Start(SONG_PATH_MENU);
        app_state = ST_MENU;
      }
    }

    HAL_Delay(1);
  }
}

/* ===================== MAPA ===================== */
static void Map_Generate(void)
{
  memset(g_map,   0, sizeof(g_map));
  memset(g_fixed, 0, sizeof(g_fixed));

  for (int c=0;c<MAP_COLS;c++){ g_map[0][c]=1; g_map[MAP_ROWS-1][c]=1; g_fixed[0][c]=g_fixed[MAP_ROWS-1][c]=1; }
  for (int r=0;r<MAP_ROWS;r++){ g_map[r][0]=1; g_map[r][MAP_COLS-1]=1; g_fixed[r][0]=g_fixed[r][MAP_COLS-1]=1; }

  const int L=6, R=MAP_COLS-1-6;
  for (int r=3;r<MAP_ROWS-3;r++){ g_map[r][L]=1; g_map[r][R]=1; }

  g_map[3][L]=0; g_map[MAP_ROWS-4][L]=0;
  g_map[3][R]=0; g_map[MAP_ROWS-4][R]=0;

  int top=4, bot=MAP_ROWS-5;
  for (int c=L;c<=L+2;c++){ g_map[top][c]=g_map[bot][c]=1; }
  for (int c=R-2;c<=R;c++){ g_map[top][c]=g_map[bot][c]=1; }

  for (int r=C_TY0;r<=C_TY1;r++) for (int c=C_TX0;c<=C_TX1;c++) g_map[r][c]=0;
}

static void Map_Draw(void)
{
  for (int r=0; r<MAP_ROWS; r++)
    for (int c=0; c<MAP_COLS; c++)
      if (g_map[r][c]) LCD_Bitmap(c*TILE_W, r*TILE_H, TILE_W, TILE_H, bloque);
}

static void DrawBlackTile(int tx, int ty)
{
  static uint16_t black[TILE_W*TILE_H];
  static int inited=0; if (!inited){ for(int i=0;i<TILE_W*TILE_H;i++) black[i]=COLOR_BLACK; inited=1; }
  LCD_Bitmap(tx*TILE_W, ty*TILE_H, TILE_W, TILE_H, (uint8_t*)black);
}

static void PatchFromMap(int x, int y, int w, int h)
{
  x=CLAMP(x,0,LCD_W-1); y=CLAMP(y,0,LCD_H-1);
  int x2=CLAMP(x+w-1,0,LCD_W-1), y2=CLAMP(y+h-1,0,LCD_H-1);

  int tx0 = x / TILE_W, ty0 = y / TILE_H;
  int tx1 = x2 / TILE_W, ty1 = y2 / TILE_H;

  for (int ty=ty0; ty<=ty1; ++ty)
    for (int tx=tx0; tx<=tx1; ++tx)
      if (g_map[ty][tx]) LCD_Bitmap(tx*TILE_W, ty*TILE_H, TILE_W, TILE_H, bloque);
      else               DrawBlackTile(tx, ty);
}

static bool RectCollidesMap(int x, int y, int w, int h)
{
  if (x<0 || y<0 || x+w>LCD_W || y+h>LCD_H) return true;
  int tx0 = x / TILE_W, ty0 = y / TILE_H;
  int tx1 = (x + w - 1) / TILE_W;
  int ty1 = (y + h - 1) / TILE_H;
  for (int ty=ty0; ty<=ty1; ++ty)
    for (int tx=tx0; tx<=tx1; ++tx)
      if (g_map[ty][tx]) return true;
  return false;
}

/* ===================== HUD / TEXTO ===================== */
static void HUD_Draw(void)
{
  int hx = (MAP_COLS-1) * TILE_W;
  int y_center = LCD_H/2;
  int y1 = y_center - (TILE_H + 2);
  int y2 = y_center + 2;

  PatchFromMap(hx, y1, TILE_W, TILE_H);
  PatchFromMap(hx, y2, TILE_W, TILE_H);

  if (p1_alive) LCD_Bitmap(hx, y1, TILE_W, TILE_H, (uint8_t*)vida1_draw);
  if (p2_alive) LCD_Bitmap(hx, y2, TILE_W, TILE_H, (uint8_t*)vida2_draw);
}

static void HUD_RedrawIfOverlapped(int x,int y,int w,int h)
{
  int hx = (MAP_COLS-1) * TILE_W;
  int y_center = LCD_H/2;
  int hy1 = y_center - (TILE_H + 2);
  int hy2 = y_center + 2;

  if (RectIntersects(x,y,w,h, hx,hy1,TILE_W,TILE_H)) {
    PatchFromMap(hx, hy1, TILE_W, TILE_H);
    if (p1_alive) LCD_Bitmap(hx, hy1, TILE_W, TILE_H, (uint8_t*)vida1_draw);
  }
  if (RectIntersects(x,y,w,h, hx,hy2,TILE_W,TILE_H)) {
    PatchFromMap(hx, hy2, TILE_W, TILE_H);
    if (p2_alive) LCD_Bitmap(hx, hy2, TILE_W, TILE_H, (uint8_t*)vida2_draw);
  }
}

/* ===================== BANANO ===================== */
static void Banana_ResizeTo(int dw, int dh)
{
  if (dw <= 0 || dh <= 0) return;
  int sw = g_ban_src_w, sh = g_ban_src_h;
  if (dw > sw) dw = sw; if (dh > sh) dh = sh;

  const uint16_t* src = (const uint16_t*)g_banana_src;
  uint16_t*       dst = g_banana_scaled;

  for (int y = 0; y < dh; ++y) {
    int sy = (int)((long long)y * sh / dh);
    for (int x = 0; x < dw; ++x) {
      int sx = (int)((long long)x * sw / dw);
      dst[y*dw + x] = src[ sy*sw + sx ];
    }
  }

  g_banana_draw = (const uint8_t*)dst;
  g_ban_draw_w  = dw;
  g_ban_draw_h  = dh;
}

/* ===================== VIDA (resize) ===================== */
static void LifeIcon_ResizeTo(int dw, int dh)
{
  if (dw <= 0 || dh <= 0) return;
  const uint16_t* src1 = (const uint16_t*)contadorJ1;
  const uint16_t* src2 = (const uint16_t*)contadorJ2;
  uint16_t*       dst1 = vida1_scaled;
  uint16_t*       dst2 = vida2_scaled;

  for (int y = 0; y < dh; ++y) {
    int sy = (int)((long long)y * VIDA_SRC_H / dh);
    for (int x = 0; x < dw; ++x) {
      int sx = (int)((long long)x * VIDA_SRC_W / dw);
      dst1[y*dw + x] = src1[ sy*VIDA_SRC_W + sx ];
      dst2[y*dw + x] = src2[ sy*VIDA_SRC_W + sx ];
    }
  }

  vida1_draw = (const uint8_t*)dst1;
  vida2_draw = (const uint8_t*)dst2;
  vida_draw_w  = dw;
  vida_draw_h  = dh;
}

/* ===================== BANANOS ===================== */
static void Bananas_Add(int x,int y)
{
  if (s_nban>=MAX_BANANAS) return;
  if (x<0||y<0||x+g_ban_draw_w>LCD_W||y+g_ban_draw_h>LCD_H) return;
  if (RectCollidesMap(x,y,g_ban_draw_w,g_ban_draw_h)) return;
  s_bananas[s_nban++] = (Banana){x,y,1};
}

static void Bananas_Init(void)
{
  s_nban=0;
  Bananas_Add(1*TILE_W,             1*TILE_H);
  Bananas_Add((MAP_COLS-2)*TILE_W,  1*TILE_H);
  Bananas_Add(1*TILE_W,            (MAP_ROWS-2)*TILE_H);
  Bananas_Add((MAP_COLS-2)*TILE_W, (MAP_ROWS-2)*TILE_H);

  int midx = (C_TX0 + C_TX1 + 1)/2;
  int midy = (C_TY0 + C_TY1 + 1)/2;
  Bananas_Add(midx*TILE_W, (C_TY0-1)*TILE_H);
  Bananas_Add(midx*TILE_W, (C_TY1+1)*TILE_H);
  Bananas_Add((C_TX0-1)*TILE_W, midy*TILE_H);
  Bananas_Add((C_TX1+1)*TILE_W, midy*TILE_H);
  Bananas_Add((C_TX0-1)*TILE_W, (C_TY0-1)*TILE_H);
  Bananas_Add((C_TX1+1)*TILE_W, (C_TY0-1)*TILE_H);
  Bananas_Add((C_TX0-1)*TILE_W, (C_TY1+1)*TILE_H);
  Bananas_Add((C_TX1+1)*TILE_W, (C_TY1+1)*TILE_H);
}

static void Bananas_DrawAll(void)
{
  for (int i=0;i<s_nban;i++)
    if (s_bananas[i].alive)
      LCD_Bitmap(s_bananas[i].x, s_bananas[i].y, g_ban_draw_w, g_ban_draw_h, (uint8_t*)g_banana_draw);
}

static void Bananas_RedrawIfOverlapped(int x,int y,int w,int h)
{
  for (int i=0;i<s_nban;i++){
    if (!s_bananas[i].alive) continue;
    if (RectIntersects(x,y,w,h, s_bananas[i].x,s_bananas[i].y, g_ban_draw_w, g_ban_draw_h))
      LCD_Bitmap(s_bananas[i].x, s_bananas[i].y, g_ban_draw_w, g_ban_draw_h, (uint8_t*)g_banana_draw);
  }
}

static void Bananas_CheckPickup(int jx,int jy,int jw,int jh,int who)
{
  for (int i=0;i<s_nban;i++){
    Banana *b = &s_bananas[i];
    if (!b->alive) continue;
    if (RectIntersects(jx,jy,jw,jh, b->x,b->y, g_ban_draw_w, g_ban_draw_h)) {
      b->alive = 0;
      PatchFromMap(b->x, b->y, g_ban_draw_w, g_ban_draw_h);
      if (who==1) p1_score += 50; else p2_score += 50;
    }
  }
}
static int Bananas_Left(void){
  int left=0; for(int i=0;i<s_nban;i++) if (s_bananas[i].alive) left++; return left;
}

/* ===================== A/B: helpers ===================== */
static Dir SheetToDir(const Sheet *s, Dir fallback){
  if (s==&P1_S_R || s==&P2_S_R) return D_RIGHT;
  if (s==&P1_S_L || s==&P2_S_L) return D_LEFT;
  if (s==&P1_S_F || s==&P2_S_F) return D_DOWN;
  if (s==&P1_S_B || s==&P2_S_B) return D_UP;
  return fallback;
}
static bool EntityBlocksTile(int tx,int ty, int ex,int ey,int ew,int eh){
  int x=tx*TILE_W, y=ty*TILE_H;
  return RectIntersects(x,y,TILE_W,TILE_H, ex,ey,ew,eh);
}

static void Line_RemoveFrom(int tx,int ty, Dir d)
{
  int dx=0,dy=0;
  if (d==D_RIGHT) dx=1; else if(d==D_LEFT) dx=-1; else if(d==D_DOWN) dy=1; else if(d==D_UP) dy=-1; else return;

  while (tx>=0 && ty>=0 && tx<MAP_COLS && ty<MAP_ROWS){
    if (g_fixed[ty][tx]) break;
    if (!g_map[ty][tx]) break;
    g_map[ty][tx]=0;
    DrawBlackTile(tx,ty);
    tx+=dx; ty+=dy;
  }
}

static void Line_CreateFrom(int tx,int ty, Dir d,
                            int ex1,int ey1,int ew1,int eh1,
                            int ex2,int ey2,int ew2,int eh2)
{
  int dx=0,dy=0;
  if (d==D_RIGHT) dx=1; else if(d==D_LEFT) dx=-1; else if(d==D_DOWN) dy=1; else if(d==D_UP) dy=-1; else return;

  while (tx>=0 && ty>=0 && tx<MAP_COLS && ty<MAP_ROWS){
    if (g_fixed[ty][tx]) break;
    if (g_map[ty][tx]) break;
    if (EntityBlocksTile(tx,ty, ex1,ey1,ew1,eh1)) break;
    if (EntityBlocksTile(tx,ty, ex2,ey2,ew2,eh2)) break;

    g_map[ty][tx]=1;
    LCD_Bitmap(tx*TILE_W, ty*TILE_H, TILE_W, TILE_H, bloque);

    tx+=dx; ty+=dy;
  }
}

static void TryActionErase_Create(const char who)
{
  int jx, jy, jw, jh; const Sheet *sh; Dir *pdir;
  if (who==1){ jx=p1_x; jy=p1_y; sh=p1_sh; jw=p1_sh->w; jh=p1_sh->h; pdir=&p1_dir; }
  else       { jx=p2_x; jy=p2_y; sh=p2_sh; jw=p2_sh->w; jh=p2_sh->h; pdir=&p2_dir; }

  Dir d = SheetToDir(sh, *pdir);
  *pdir = d;

  int tx,ty;
  if (d==D_RIGHT){ tx = (jx + jw)/TILE_W; ty = (jy + jh/2)/TILE_H; }
  else if (d==D_LEFT){ tx = (jx - 1)/TILE_W; ty = (jy + jh/2)/TILE_H; }
  else if (d==D_DOWN){ tx = (jx + jw/2)/TILE_W; ty = (jy + jh)/TILE_H; }
  else if (d==D_UP){ tx = (jx + jw/2)/TILE_W; ty = (jy - 1)/TILE_H; }
  else return;

  if (tx<0||ty<0||tx>=MAP_COLS||ty>=MAP_ROWS) return;

  if (who==1){
    if (p1_act.active && (p1_act.sheet==&P1_S_X)) { if (g_map[ty][tx] && !g_fixed[ty][tx]) Line_RemoveFrom(tx,ty,d); }
    else if (p1_act.active && (p1_act.sheet==&P1_S_C)) { Line_CreateFrom(tx,ty,d, p2_x,p2_y,p2_sh->w,p2_sh->h, en_x,en_y,en_sh->w,en_sh->h); }
  } else {
    if (p2_act.active && (p2_act.sheet==&P2_S_X)) { if (g_map[ty][tx] && !g_fixed[ty][tx]) Line_RemoveFrom(tx,ty,d); }
    else if (p2_act.active && (p2_act.sheet==&P2_S_C)) { Line_CreateFrom(tx,ty,d, p1_x,p1_y,p1_sh->w,p1_sh->h, en_x,en_y,en_sh->w,en_sh->h); }
  }
}

/* ===================== JUGADORES ===================== */
static inline bool IsVerticalSheet(const Sheet *s){
  return (s==&P1_S_F || s==&P1_S_B || s==&P2_S_F || s==&P2_S_B);
}
static inline int FixAnimIndex(const Sheet *s, int fr){
  int idx = fr % s->columns;
  return IsVerticalSheet(s) ? (s->columns - 1 - idx) : idx;
}
static inline void DrawSheet(int x,int y,const Sheet* s,int fr){
  int use_fr = FixAnimIndex(s, fr);
  LCD_Sprite(x,y, s->w,s->h, (uint8_t*)s->data, s->columns, use_fr, 0, 0);
}

/* === Draw/Clear con repintado de objetos/HUD === */
static inline void DrawP1(void){
  if (p1_alive) DrawSheet(p1_x, p1_y, p1_sh, p1_fr);
}
static inline void ClrP1(void){
  PatchFromMap(p1_x, p1_y, p1_sh->w, p1_sh->h);
  Bananas_RedrawIfOverlapped(p1_x, p1_y, p1_sh->w, p1_sh->h);
  HUD_RedrawIfOverlapped(p1_x, p1_y, p1_sh->w, p1_sh->h);
}
static inline void DrawP2(void){
  if (p2_alive) DrawSheet(p2_x, p2_y, p2_sh, p2_fr);
}
static inline void ClrP2(void){
  PatchFromMap(p2_x, p2_y, p2_sh->w, p2_sh->h);
  Bananas_RedrawIfOverlapped(p2_x, p2_y, p2_sh->w, p2_sh->h);
  HUD_RedrawIfOverlapped(p2_x, p2_y, p2_sh->w, p2_sh->h);
}
static inline void DrawEN(void){
  DrawSheet(en_x, en_y, en_sh, en_fr);
  HUD_RedrawIfOverlapped(en_x, en_y, en_sh->w, en_sh->h);
}
static inline void ClrEN(void){
  PatchFromMap(en_x, en_y, en_sh->w, en_sh->h);
  Bananas_RedrawIfOverlapped(en_x, en_y, en_sh->w, en_sh->h);
  HUD_RedrawIfOverlapped(en_x, en_y, en_sh->w, en_sh->h);
}

static void DecideAxisMove(const PlayerInput *pi, int *dx, int *dy, const Sheet **out_sheet,
                           const Sheet *S_F,const Sheet *S_B,const Sheet *S_L,const Sheet *S_R)
{
  *dx=0; *dy=0;
  if ((pi->dir & DIR_LEFT) ^ (pi->dir & DIR_RIGHT)) {
    if (pi->dir & DIR_LEFT)  { *dx = -P_STEP; *out_sheet = S_L; }
    else                     { *dx =  P_STEP; *out_sheet = S_R; }
  } else if ((pi->dir & DIR_UP) ^ (pi->dir & DIR_DOWN)) {
    if (pi->dir & DIR_UP)    { *dy = -P_STEP; *out_sheet = S_B; }
    else                     { *dy =  P_STEP; *out_sheet = S_F; }
  }
}

static void TryMovePlayer(int *x,int *y,int w,int h,int dx,int dy,int ox,int oy,int ow,int oh)
{
  if (dx != 0) {
    int nx = *x + dx;
    if (!RectCollidesMap(nx, *y, w, h) && !RectIntersects(nx,*y,w,h, ox,oy,ow,oh)) {
      PatchFromMap(*x, *y, w, h); *x = nx;
      Bananas_RedrawIfOverlapped(*x, *y, w, h); HUD_RedrawIfOverlapped(*x, *y, w, h);
    }
  }
  if (dy != 0) {
    int ny = *y + dy;
    if (!RectCollidesMap(*x, ny, w, h) && !RectIntersects(*x,ny,w,h, ox,oy,ow,oh)) {
      PatchFromMap(*x, *y, w, h); *y = ny;
      Bananas_RedrawIfOverlapped(*x, *y, w, h); HUD_RedrawIfOverlapped(*x, *y, w, h);
    }
  }
}

static void Players_HandleInput(void)
{
  PlayerInput i1, i2;
  __disable_irq(); Gamepad_GetP1(&i1); Gamepad_GetP2(&i2); __enable_irq();

  uint32_t now = HAL_GetTick();

  /* ------- P1 ------- */
  if (p1_alive){
    int dx1=0,dy1=0; const Sheet* move1 = p1_sh;
    DecideAxisMove(&i1, &dx1, &dy1, &move1, &P1_S_F,&P1_S_B,&P1_S_L,&P1_S_R);

    uint8_t A1_edge = (i1.A && !p1_A_prev);
    uint8_t B1_edge = (i1.B && !p1_B_prev);
    p1_A_prev = i1.A; p1_B_prev = i1.B;

    if (!p1_act.active) {
      if (A1_edge || B1_edge) {
        p1_act.active = 1;
        p1_act.sheet  = A1_edge ? &P1_S_X : &P1_S_C;
        p1_act.resume_sheet = move1;
        p1_act.t_start = now;
        p1_act.duration_ms = p1_act.sheet->columns * ANIM_MS;
        TryActionErase_Create(1);
        ClrP1(); p1_sh = p1_act.sheet; p1_fr=0; p1_tanim=now; DrawP1();
      } else {
        if (move1 != p1_sh) { ClrP1(); p1_sh = move1; p1_fr=0; p1_tanim=now; DrawP1(); }
      }
    } else if (now - p1_act.t_start >= p1_act.duration_ms){
      ClrP1(); p1_sh = p1_act.resume_sheet; p1_fr=0; p1_act.active=0; DrawP1();
    }

    if (dx1||dy1) p1_dir = SheetToDir(move1, p1_dir);

    TryMovePlayer(&p1_x,&p1_y,p1_sh->w,p1_sh->h, dx1,dy1, p2_x,p2_y,p2_sh->w,p2_sh->h);
    DrawP1();
    Bananas_CheckPickup(p1_x,p1_y,p1_sh->w,p1_sh->h, 1);
  }

  /* ------- P2 ------- */
  if (p2_alive){
    int dx2=0,dy2=0; const Sheet* move2 = p2_sh;
    DecideAxisMove(&i2, &dx2, &dy2, &move2, &P2_S_F,&P2_S_B,&P2_S_L,&P2_S_R);

    uint8_t A2_edge = (i2.A && !p2_A_prev);
    uint8_t B2_edge = (i2.B && !p2_B_prev);
    p2_A_prev = i2.A; p2_B_prev = i2.B;

    if (!p2_act.active) {
      if (A2_edge || B2_edge) {
        p2_act.active = 1;
        p2_act.sheet  = A2_edge ? &P2_S_X : &P2_S_C;
        p2_act.resume_sheet = move2;
        p2_act.t_start = now;
        p2_act.duration_ms = p2_act.sheet->columns * ANIM_MS;
        TryActionErase_Create(2);
        ClrP2(); p2_sh = p2_act.sheet; p2_fr=0; p2_tanim=now; DrawP2();
      } else {
        if (move2 != p2_sh) { ClrP2(); p2_sh = move2; p2_fr=0; p2_tanim=now; DrawP2(); }
      }
    } else if (now - p2_act.t_start >= p2_act.duration_ms){
      ClrP2(); p2_sh = p2_act.resume_sheet; p2_fr=0; p2_act.active=0; DrawP2();
    }

    if (dx2||dy2) p2_dir = SheetToDir(move2, p2_dir);

    TryMovePlayer(&p2_x,&p2_y,p2_sh->w,p2_sh->h, dx2,dy2, p1_x,p1_y,p1_sh->w,p1_sh->h);
    DrawP2();
    Bananas_CheckPickup(p2_x,p2_y,p2_sh->w,p2_sh->h, 2);
  }

  if (p1_alive && RectIntersects(p1_x,p1_y,p1_sh->w,p1_sh->h, en_x,en_y,en_sh->w,en_sh->h)) {
    p1_alive = 0; p1_act.active=0;
    ClrP1();
    HUD_Draw();
  }
  if (p2_alive && RectIntersects(p2_x,p2_y,p2_sh->w,p2_sh->h, en_x,en_y,en_sh->w,en_sh->h)) {
    p2_alive = 0; p2_act.active=0;
    ClrP2();
    HUD_Draw();
  }
}

/* ===================== ENEMIGO ===================== */
static void Enemy_InitPatrol(void)
{
  en_x = TILE_W; en_y = TILE_H;
  en_dir = D_RIGHT;
  en_sh  = &EN_S_R;
}

static bool EnemyRectCollidesMap_Padded(int x,int y,int w,int h, int pad)
{
  int nx = x + pad, ny = y + pad;
  int nw = w - 2*pad, nh = h - 2*pad;
  if (nw < 1) nw = 1; if (nh < 1) nh = 1;
  return RectCollidesMap(nx, ny, nw, nh);
}

static bool Enemy_TryStep(Dir d, int *nx, int *ny)
{
  int vx=0,vy=0;
  if (d==D_RIGHT) vx= E_STEP; else if(d==D_LEFT) vx=-E_STEP;
  else if (d==D_DOWN) vy= E_STEP; else if(d==D_UP) vy=-E_STEP;

  *nx = en_x + vx; *ny = en_y + vy;

  int pad = 3;
  if (vy != 0) pad = 5;

  if (EnemyRectCollidesMap_Padded(*nx,*ny, en_sh->w,en_sh->h, pad)) return false;
  return true;
}

static inline int CenterTileX(int x,int w){ int cx = x + w/2; return CLAMP(cx / TILE_W, 0, MAP_COLS-1); }
static inline int CenterTileY(int y,int h){ int cy = y + h/2; return CLAMP(cy / TILE_H, 0, MAP_ROWS-1); }

static inline void TileToPixelForSprite(int tx,int ty,int sw,int sh, int *px,int *py){
  int cx = tx*TILE_W + TILE_W/2;
  int cy = ty*TILE_H + TILE_H/2;
  *px = cx - sw/2; *py = cy - sh/2;
}

static bool TilePassableForEnemy(int tx,int ty, const Sheet* sh)
{
  if (tx<0||ty<0||tx>=MAP_COLS||ty>=MAP_ROWS) return false;
  int px,py; TileToPixelForSprite(tx,ty, sh->w, sh->h, &px,&py);
  return !RectCollidesMap(px,py, sh->w, sh->h);
}

static void Enemy_GetStartTile(int *sx,int *sy){
  *sx = CenterTileX(en_x, en_sh->w);
  *sy = CenterTileY(en_y, en_sh->h);
}

static void Enemy_GetPlayerTargetTile(int *gx,int *gy, int *has_target)
{
  *has_target = 0;
  int best_d = 1<<30, best_tx=0, best_ty=0;

  if (p1_alive){
    int t1x = CenterTileX(p1_x, p1_sh->w);
    int t1y = CenterTileY(p1_y, p1_sh->h);
    int d = abs(t1x - CenterTileX(en_x,en_sh->w)) + abs(t1y - CenterTileY(en_y,en_sh->h));
    if (d < best_d){ best_d=d; best_tx=t1x; best_ty=t1y; *has_target=1; }
  }
  if (p2_alive){
    int t2x = CenterTileX(p2_x, p2_sh->w);
    int t2y = CenterTileY(p2_y, p2_sh->h);
    int d = abs(t2x - CenterTileX(en_x,en_sh->w)) + abs(t2y - CenterTileY(en_y,en_sh->h));
    if (d < best_d){ best_d=d; best_tx=t2x; best_ty=t2y; *has_target=1; }
  }

  *gx = best_tx; *gy = best_ty;
}

static Dir BFS_NextDirForEnemy(int sx,int sy,int gx,int gy, const Sheet* sh)
{
  if (sx==gx && sy==gy) return D_NONE;

  static uint8_t vis[MAP_ROWS][MAP_COLS];
  static int16_t px[MAP_ROWS][MAP_COLS], py[MAP_ROWS][MAP_COLS];
  memset(vis, 0, sizeof(vis));

  int qx[MAP_COLS*MAP_ROWS], qy[MAP_COLS*MAP_ROWS], qh=0, qt=0;

  if (!TilePassableForEnemy(sx,sy, sh)) return D_NONE;
  vis[sy][sx]=1; px[sy][sx]=-1; py[sy][sx]=-1;
  qx[qt]=sx; qy[qt]=sy; qt++;

  const int dx[4]={+1,0,-1,0};
  const int dy[4]={0,+1,0,-1};

  while (qh<qt){
    int x=qx[qh], y=qy[qh]; qh++;
    if (x==gx && y==gy) break;
    for (int i=0;i<4;i++){
      int nx=x+dx[i], ny=y+dy[i];
      if (nx<0||ny<0||nx>=MAP_COLS||ny>=MAP_ROWS) continue;
      if (vis[ny][nx]) continue;
      if (!TilePassableForEnemy(nx,ny, sh)) continue;
      vis[ny][nx]=1; px[ny][nx]=x; py[ny][nx]=y;
      qx[qt]=nx; qy[qt]=ny; qt++;
    }
  }

  if (!vis[gy][gx]) return D_NONE;

  int cx=gx, cy=gy;
  int bx=-1, by=-1;
  while (px[cy][cx]!=-1){
    bx=cx; by=cy;
    int tx=px[cy][cx], ty=py[cy][cx];
    cx=tx; cy=ty;
  }
  if (bx==-1) return D_NONE;

  if      (bx==sx+1 && by==sy)   return D_RIGHT;
  else if (bx==sx-1 && by==sy)   return D_LEFT;
  else if (bx==sx && by==sy+1)   return D_DOWN;
  else if (bx==sx && by==sy-1)   return D_UP;

  return D_NONE;
}

static void Enemy_FaceTowardsPixel(int px,int py)
{
  int ecx = en_x + en_sh->w/2;
  int ecy = en_y + en_sh->h/2;
  int dx = px - ecx;
  int dy = py - ecy;
  if (abs(dx) > abs(dy)) en_dir = (dx>0)? D_RIGHT : D_LEFT;
  else                   en_dir = (dy>0)? D_DOWN  : D_UP;
}

static void Enemy_Tick(void)
{
  if      (en_dir==D_RIGHT) en_sh = &EN_S_R;
  else if (en_dir==D_LEFT)  en_sh = &EN_S_L;
  else if (en_dir==D_DOWN)  en_sh = &EN_S_F;
  else if (en_dir==D_UP)    en_sh = &EN_S_B;

  int gx=0, gy=0, has_target=0;
  Enemy_GetPlayerTargetTile(&gx,&gy,&has_target);
  if (!has_target) return;

  int sx, sy; Enemy_GetStartTile(&sx,&sy);
  Dir step = BFS_NextDirForEnemy(sx,sy,gx,gy, en_sh);

  int nx=en_x, ny=en_y;
  bool moved=false;

  if (step!=D_NONE && Enemy_TryStep(step,&nx,&ny)) {
    en_dir = step; moved=true;
  } else {
    int tpx, tpy; TileToPixelForSprite(gx,gy, en_sh->w,en_sh->h, &tpx,&tpy);
    int dx = tpx - en_x; int dy = tpy - en_y;
    Dir try1, try2;
    if (abs(dx) > abs(dy)) { try1 = (dx>0)?D_RIGHT:D_LEFT; try2 = (dy>0)?D_DOWN:D_UP; }
    else                   { try1 = (dy>0)?D_DOWN:D_UP;    try2 = (dx>0)?D_RIGHT:D_LEFT; }

    if (!moved && Enemy_TryStep(try1,&nx,&ny)){ en_dir=try1; moved=true; }
    if (!moved && Enemy_TryStep(try2,&nx,&ny)){ en_dir=try2; moved=true; }

    if (!moved){
      int tpx2, tpy2; TileToPixelForSprite(gx,gy, en_sh->w,en_sh->h, &tpx2,&tpy2);
      Enemy_FaceTowardsPixel(tpx2,tpy2);
    }
  }

  if      (en_dir==D_RIGHT) en_sh = &EN_S_R;
  else if (en_dir==D_LEFT)  en_sh = &EN_S_L;
  else if (en_dir==D_DOWN)  en_sh = &EN_S_F;
  else if (en_dir==D_UP)    en_sh = &EN_S_B;

  if (moved){ PatchFromMap(en_x,en_y,en_sh->w,en_sh->h); en_x = nx; en_y = ny; LCD_Sprite(en_x,en_y,en_sh->w,en_sh->h,(uint8_t*)en_sh->data,en_sh->columns,en_fr,0,0); }
  else { LCD_Sprite(en_x,en_y,en_sh->w,en_sh->h,(uint8_t*)en_sh->data,en_sh->columns,en_fr,0,0); }
}

/* ===================== FIN DEL JUEGO ===================== */
static void ShowEndScreen(void)
{
  UI_FillRect(20, 70, LCD_W-40, 110, COLOR_BLUE);

  char buf[64];
  snprintf(buf, sizeof(buf), "P1: %03d  P2: %03d", p1_score, p2_score);
  DrawTextC(buf, 40, 82, COLOR_YELLOW, COLOR_BLUE);

  if (g_game_over==1) DrawTextC("Ganaron",   40, 96, COLOR_WHITE, COLOR_BLUE);
  else                DrawTextC("Perdieron", 40, 96, COLOR_WHITE, COLOR_BLUE);

  DrawTextC("A: Reintentar", 40, 112, COLOR_WHITE, COLOR_BLUE);
  DrawTextC("B: Menu",       40, 126, COLOR_WHITE, COLOR_BLUE);
}

/* ===================== SCORES / HELP / CREDITS ===================== */
static void Scores_Append(int p1, int p2)
{
  f_mount(&s_fs, "", 1);

  FIL f; UINT wr=0;
  FRESULT fr = f_open(&f, SCORES_FILE, FA_OPEN_APPEND | FA_WRITE);
  if (fr != FR_OK) fr = f_open(&f, SCORES_FILE, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr == FR_OK) {
    char line[64];
    int n = snprintf(line, sizeof(line), "P1=%03d, P2=%03d\r\n", p1, p2);
    f_write(&f, line, (UINT)n, &wr);
    f_close(&f);
  }
}

static void Scores_ShowScreen(void)
{
  LCD_Clear(COLOR_BLACK);
  UI_FillRect(10, 10, LCD_W-20, LCD_H-20, COLOR_BLUE);
  DrawTextC("SCORES (ultimas partidas)", 20, 16, COLOR_YELLOW, COLOR_BLUE);

  char ring[MAX_LINES_SHOWN][MAX_LINE_LEN];
  int   ring_count = 0;

  f_mount(&s_fs, "", 1);
  FIL f; FRESULT fr = f_open(&f, SCORES_FILE, FA_READ);
  if (fr == FR_OK) {
    char chunk[128];
    UINT br=0;
    char line[MAX_LINE_LEN]; int llen=0;

    do {
      br=0;
      fr = f_read(&f, chunk, sizeof(chunk), &br);
      for (UINT i=0;i<br;i++){
        char ch = chunk[i];
        if (ch=='\n' || ch=='\r') {
          if (llen>0) {
            line[ CLAMP(llen,0,MAX_LINE_LEN-1) ] = '\0';
            strncpy(ring[ring_count % MAX_LINES_SHOWN], line, MAX_LINE_LEN);
            ring[ring_count % MAX_LINES_SHOWN][MAX_LINE_LEN-1]='\0';
            ring_count++;
            llen=0;
          }
        } else if (llen < (MAX_LINE_LEN-1)) {
          line[llen++] = ch;
        }
      }
    } while (fr == FR_OK && br == sizeof(chunk));

    if (llen>0){
      line[ CLAMP(llen,0,MAX_LINE_LEN-1) ] = '\0';
      strncpy(ring[ring_count % MAX_LINES_SHOWN], line, MAX_LINE_LEN);
      ring[ring_count % MAX_LINES_SHOWN][MAX_LINE_LEN-1]='\0';
      ring_count++;
    }
    f_close(&f);
  } else {
    strncpy(ring[0], "No hay partidas guardadas.", MAX_LINE_LEN);
    ring[0][MAX_LINE_LEN-1]='\0';
    ring_count = 1;
  }

  int y = 36;                    /* antes 50 */
  int to_show = (ring_count < MAX_LINES_SHOWN) ? ring_count : MAX_LINES_SHOWN;
  for (int i=to_show-1; i>=0; --i){
    const char* s = ring[(ring_count-1 - i) % MAX_LINES_SHOWN];
    DrawTextC(s, 20, y, COLOR_WHITE, COLOR_BLUE);
    y += 10;                     /* antes 18 */
    if (y > (LCD_H-24)) break;
  }
  DrawTextC("B: Volver", 20, LCD_H-20, COLOR_WHITE, COLOR_BLUE);
}

static void Help_ShowScreen(void)
{
  LCD_Clear(COLOR_BLACK);
  UI_FillRect(10, 10, LCD_W-20, LCD_H-20, COLOR_BLUE);
  DrawTextC("HELP", 20, 16, COLOR_YELLOW, COLOR_BLUE);

  DrawTextC("Arriba / Abajo en el eje X.",      20, 40, COLOR_WHITE, COLOR_BLUE);
  DrawTextC("Derecha / Izquierda en el eje Y.", 20, 52, COLOR_WHITE, COLOR_BLUE);

  DrawTextC("B: Volver", 20, LCD_H-20, COLOR_WHITE, COLOR_BLUE);
}

static void Credits_ShowScreen(void)
{
  LCD_Clear(COLOR_BLACK);
  UI_FillRect(10, 10, LCD_W-20, LCD_H-20, COLOR_BLUE);
  DrawTextC("CREDITOS", 20, 16, COLOR_YELLOW, COLOR_BLUE);

  DrawTextC("Willy Cuellar 23182.", 20, 40, COLOR_WHITE, COLOR_BLUE);
  DrawTextC("Mario Cano 23589.",    20, 52, COLOR_WHITE, COLOR_BLUE);

  DrawTextC("B: Volver", 20, LCD_H-20, COLOR_WHITE, COLOR_BLUE);
}

/* Espera no bloqueante a B para volver al menú */
static bool Wait_B_ToGoMenu(void)
{
  PlayerInput i1, i2;
  __disable_irq(); Gamepad_GetP1(&i1); Gamepad_GetP2(&i2); __enable_irq();
  HAL_Delay(1);
  return (i1.B || i2.B);
}

/* ===================== HAL init ===================== */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
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
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  /* prioridad alta para el gamepad */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static void MX_TIM11_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM11_CLK_ENABLE();

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim11)  != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim11);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin|LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

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

  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin|SD_SS_Pin;
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
/* End of file */
