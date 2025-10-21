/*
 * ui_menu.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#include "ui_menu.h"
#include "ili9341.h"
#include "bitmaps.h"
#include "bitmaps.h"


#define LCD_W   320
#define LCD_H   240
#define BYTES_PP 2

#define ICON_W  10
#define ICON_H  11
#define LEFT_X  110
#define RIGHT_X 200
#define BLINK_MS 500

#define OPT_COUNT 4
static const int OPT_Y[OPT_COUNT] = { 125, 140, 155, 170 };

#define NAV_DEBOUNCE_MS 80U



typedef enum { NAV_NEUTRAL=0, NAV_HELD_UP, NAV_HELD_DOWN } NavLatch;
static NavLatch  nav_state_p1 = NAV_NEUTRAL, nav_state_p2 = NAV_NEUTRAL;
static uint32_t  last_tick_p1 = 0, last_tick_p2 = 0;

static uint8_t  menu_idx = 0;
static uint8_t  helados_on = 1;
static uint32_t tBlink = 0;

// --- helpers gráficos ---
static void draw_bg(void){ LCD_Bitmap(0,0,LCD_W,LCD_H,fondo); }
static void draw_helados(int y){
  LCD_Bitmap(LEFT_X,  y, ICON_W, ICON_H, Helado);
  LCD_Bitmap(RIGHT_X, y, ICON_W, ICON_H, Helado);
}
static void patch_bg(int x,int y,int w,int h){
  if (x<0||y<0||x+w>LCD_W||y+h>LCD_H) return;
  for (int r=0;r<h;r++){
    const uint8_t*src=&fondo[((y+r)*LCD_W+x)*BYTES_PP];
    LCD_Bitmap(x, y+r, w, 1, (uint8_t*)src);
  }
}
static void move_selection(int new_idx)
{
  if (new_idx<0) new_idx=OPT_COUNT-1;
  if (new_idx>=OPT_COUNT) new_idx=0;
  if (new_idx==(int)menu_idx) return;

  int y_old=OPT_Y[menu_idx], y_new=OPT_Y[new_idx];
  patch_bg(LEFT_X, y_old, ICON_W, ICON_H);
  patch_bg(RIGHT_X,y_old, ICON_W, ICON_H);

  menu_idx=(uint8_t)new_idx;
  if (helados_on) draw_helados(y_new);
}

// --- entrada con latch ---
static int process_player(DirMask d, NavLatch *st, uint32_t *last)
{
  uint32_t now = HAL_GetTick();
  bool up   = (d & DIR_UP)   && !(d & DIR_DOWN);
  bool down = (d & DIR_DOWN) && !(d & DIR_UP);
  bool neutral = !(up||down);

  if (neutral) { *st= NAV_NEUTRAL; return 0; }
  if ((int32_t)(now - *last) < (int32_t)NAV_DEBOUNCE_MS) return 0;

  if (*st == NAV_NEUTRAL) {
    *last = now;
    if (up)   { *st = NAV_HELD_UP;   return -1; }
    if (down) { *st = NAV_HELD_DOWN; return +1; }
  }
  return 0;
}

static uint8_t prevA1=0, prevA2=0;
static bool action_edge(uint8_t a_now, uint8_t *prev)
{
  bool edge = a_now && !(*prev);
  *prev = a_now;
  return edge;
}

// --- API ---
void Menu_Init(void)
{
  menu_idx = 0; helados_on = 1; tBlink = HAL_GetTick();
  draw_bg();
  draw_helados(OPT_Y[menu_idx]);
}

void Menu_Tick(void)
{
  // blink por parches
  uint32_t now = HAL_GetTick();
  if ((now - tBlink) >= BLINK_MS) {
    tBlink = now; helados_on ^= 1;
    int y = OPT_Y[menu_idx];
    if (helados_on) draw_helados(y);
    else { patch_bg(LEFT_X,y,ICON_W,ICON_H); patch_bg(RIGHT_X,y,ICON_W,ICON_H); }
  }

  // entrada P1+P2
  PlayerInput p1, p2; Gamepad_GetP1(&p1); Gamepad_GetP2(&p2);
  int a1 = process_player(p1.dir, &nav_state_p1, &last_tick_p1);
  int a2 = process_player(p2.dir, &nav_state_p2, &last_tick_p2);

  int act = a1; if (act==0) act = a2;  // evita doble salto en un mismo frame
  if (act<0) move_selection((int)menu_idx - 1);
  else if (act>0) move_selection((int)menu_idx + 1);
}

int Menu_GetIndex(void) { return (int)menu_idx; }

bool Menu_ActionPressed(void)
{
  PlayerInput p1,p2; Gamepad_GetP1(&p1); Gamepad_GetP2(&p2);
  bool e1 = action_edge(p1.A, &prevA1);
  bool e2 = action_edge(p2.A, &prevA2);
  return (e1 || e2);
}

