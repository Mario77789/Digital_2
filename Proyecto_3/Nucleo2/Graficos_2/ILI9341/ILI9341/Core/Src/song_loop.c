/*
 * song_loop.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#include "song_loop.h"
#include "fatfs.h"
#include "audio_pwm.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =================== Config por defecto =================== */
#ifndef SONG_READ_BUFSZ
#define SONG_READ_BUFSZ 128
#endif
#ifndef SONG_MAX_NOTES
#define SONG_MAX_NOTES  2048
#endif
#ifndef SONG_SPEED_X
#define SONG_SPEED_X    1     /* 1x = normal */
#endif

/* =================== Tipos internos =================== */
/* Formato interno para RAM/SD */
typedef struct { uint16_t midi; uint16_t ticks; } SNote;

/* Modo de reproducción */
typedef enum {
  MODE_NONE  = 0,
  MODE_SD    = 1,   /* s_notes[] llenado desde archivo .c */
  MODE_ARRAY = 2    /* reproducir desde arreglo en Flash (Ingame) */
} SongMode;

/* =================== Estado global =================== */
static TIM_HandleTypeDef *s_tim = NULL; /* TIM10 (scheduler) */
static uint32_t  s_speed_x = SONG_SPEED_X;

/* --- SD/RAM --- */
static SNote     s_notes[SONG_MAX_NOTES];
static uint16_t  s_note_count = 0;
static uint16_t  s_note_index = 0;
static uint32_t  s_tick_ms    = 62;
static char      s_linebuf[SONG_READ_BUFSZ];

/* --- ARRAY --- */
static const SNote *s_arr      = NULL;  /* Nota: aceptamos {u8,u16} -> cabe en u16 */
static uint32_t     s_arr_len  = 0;
static uint32_t     s_arr_idx  = 0;
static uint32_t     s_arr_tick = 62;    /* tick_ms para el arreglo */
static int          s_arr_loop = 1;

/* --- Modo actual --- */
static SongMode s_mode = MODE_NONE;

/* =================== Helpers =================== */
static uint32_t midi2hz(uint32_t m)
{
  if (m == 0) return 0;
  float f = 440.0f * powf(2.0f, ((float)m - 69.0f) / 12.0f);
  if (f < 1.0f) f = 1.0f;
  if (f > 20000.0f) f = 20000.0f;
  return (uint32_t)(f + 0.5f);
}

static void sched_next(uint32_t dur_ms)
{
  if (!s_tim) return;
  if (dur_ms == 0) dur_ms = 1;

  /* Base 10 kHz (0.1 ms/tick) => ARR = dur_ms*10 - 1 */
  uint32_t ticks_0p1ms = dur_ms * 10U;
  if (ticks_0p1ms == 0) ticks_0p1ms = 1;
  if (ticks_0p1ms > 65535U) ticks_0p1ms = 65535U;

  __HAL_TIM_SET_AUTORELOAD(s_tim, (uint16_t)(ticks_0p1ms - 1));
  __HAL_TIM_SET_COUNTER(s_tim, 0);
  HAL_TIM_Base_Start_IT(s_tim);
}

/* =================== Parser SD (.c) =================== */
static bool find_tick_ms(FIL *fp)
{
  DWORD pos = f_tell(fp);
  char buf[SONG_READ_BUFSZ];
  s_tick_ms = 62;

  for (int i=0; i<200; i++) {
    if (!f_gets(buf, sizeof(buf), fp)) break;
    unsigned long t=0;
    if (sscanf(buf, " #define TICK_MS %lu", &t) == 1 ||
        sscanf(buf, "#define TICK_MS %lu", &t) == 1)
    {
      if (t > 0 && t < 1000) s_tick_ms = (uint32_t)t;
      break;
    }
  }
  f_lseek(fp, pos);
  return true;
}

static bool load_to_ram(const char *path)
{
  FIL f;
  if (f_open(&f, path, FA_READ) != FR_OK) return false;

  find_tick_ms(&f);
  s_note_count = 0;

  while (f_gets(s_linebuf, sizeof(s_linebuf), &f)) {
    for (char *p = s_linebuf; *p; ++p) if (*p == ',') *p = ' ';
    unsigned long m=0, tk=0;
    int got = sscanf(s_linebuf, " { %lu %lu }", &m, &tk);
    if (got != 2) got = sscanf(s_linebuf, " %lu %lu", &m, &tk);

    if (got == 2 && s_note_count < SONG_MAX_NOTES) {
      s_notes[s_note_count].midi  = (uint16_t)m;
      s_notes[s_note_count].ticks = (uint16_t)tk;
      s_note_count++;
    }
  }
  f_close(&f);
  return (s_note_count > 0);
}

/* =================== API =================== */
void SongLoop_Init(TIM_HandleTypeDef *htim_sched, uint32_t speed_x)
{
  s_tim = htim_sched;
  s_speed_x = (speed_x == 0) ? 1 : speed_x;

  /* Config TIM10 a 10 kHz (APB2=80 MHz => PSC=7999) */
  __HAL_RCC_TIM10_CLK_ENABLE();

  s_tim->Instance               = TIM10;
  s_tim->Init.Prescaler         = 7999;                        /* 80MHz/(7999+1)=10kHz */
  s_tim->Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_tim->Init.Period            = 1000 - 1;                    /* placeholder */
  s_tim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(s_tim);

  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

  s_mode = MODE_NONE;
  s_arr = NULL; s_arr_len = s_arr_idx = 0;
}

bool SongLoop_Start(const char *path)
{
  s_mode = MODE_NONE;
  s_note_index = 0;

  if (!load_to_ram(path)) {
    AudioPWM_SetFreq(0);
    return false;
  }

  s_mode = MODE_SD;

  /* Primera nota (acelerada por s_speed_x) */
  uint16_t m  = s_notes[s_note_index].midi;
  uint16_t tk = s_notes[s_note_index].ticks;
  s_note_index++;

  uint32_t dur = (uint32_t)tk * s_tick_ms / s_speed_x;
  if (dur == 0) dur = 1;

  AudioPWM_SetFreq(midi2hz(m));
  sched_next(dur);
  return true;
}

void SongLoop_StartFromArray(const void *notes, uint32_t count, uint32_t tick_ms, int loop)
{
  /* Detener lo que esté sonando */
  SongLoop_Stop();

  s_arr      = (const SNote*)notes;  /* Nota: si tu array tiene midi u8, cabe en u16 */
  s_arr_len  = count;
  s_arr_idx  = 0;
  s_arr_tick = (tick_ms == 0) ? 1 : tick_ms;
  s_arr_loop = loop ? 1 : 0;

  if (!s_arr || s_arr_len == 0) {
    AudioPWM_SetFreq(0);
    s_mode = MODE_NONE;
    return;
  }

  s_mode = MODE_ARRAY;

  /* Primera nota */
  uint16_t m  = s_arr[s_arr_idx].midi;
  uint16_t tk = s_arr[s_arr_idx].ticks;
  s_arr_idx++;

  uint32_t dur = (uint32_t)tk * s_arr_tick / s_speed_x;
  if (dur == 0) dur = 1;

  AudioPWM_SetFreq(midi2hz(m));
  sched_next(dur);
}

void SongLoop_Stop(void)
{
  if (s_tim) {
    HAL_TIM_Base_Stop_IT(s_tim);
  }
  AudioPWM_SetFreq(0);

  s_mode = MODE_NONE;

  /* Limpieza de índices */
  s_note_index = s_note_count = 0;
  s_arr = NULL; s_arr_len = s_arr_idx = 0;
}

/* =================== ISR (loop infinito) =================== */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* Usamos s_tim como base del scheduler */
  if (!s_tim) return;

  if (__HAL_TIM_GET_FLAG(s_tim, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(s_tim, TIM_IT_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(s_tim, TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(s_tim);

    if (s_mode == MODE_SD) {
      if (s_note_index >= s_note_count) {
        s_note_index = 0;  /* LOOP SD */
      }

      uint16_t m  = s_notes[s_note_index].midi;
      uint16_t tk = s_notes[s_note_index].ticks;
      s_note_index++;

      uint32_t dur = (uint32_t)tk * s_tick_ms / s_speed_x;
      if (dur == 0) dur = 1;

      AudioPWM_SetFreq(midi2hz(m));
      sched_next(dur);
      return;
    }

    if (s_mode == MODE_ARRAY) {
      if (s_arr_idx >= s_arr_len) {
        if (!s_arr_loop) { AudioPWM_SetFreq(0); s_mode = MODE_NONE; return; }
        s_arr_idx = 0;  /* LOOP ARRAY */
      }

      uint16_t m  = s_arr[s_arr_idx].midi;
      uint16_t tk = s_arr[s_arr_idx].ticks;
      s_arr_idx++;

      uint32_t dur = (uint32_t)tk * s_arr_tick / s_speed_x;
      if (dur == 0) dur = 1;

      AudioPWM_SetFreq(midi2hz(m));
      sched_next(dur);
      return;
    }

    /* Si llegamos aquí: no hay modo activo */
    AudioPWM_SetFreq(0);
  }
}
