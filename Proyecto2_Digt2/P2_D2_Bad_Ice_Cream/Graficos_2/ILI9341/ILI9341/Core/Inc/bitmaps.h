#ifndef INC_BITMAPS_H_
#define INC_BITMAPS_H_

#include <stdint.h>

/* Alineación opcional por si usas DMA/SPI y quieres 4-byte alignment */
#ifndef BITMAP_ALIGN
#define BITMAP_ALIGN __attribute__((aligned(4)))
#endif

/* ---- Declaraciones (solo extern) ---- */
/* Fondo 320x240 RGB565 */
extern const uint8_t BITMAP_ALIGN fondo[];

/* Tiles / sprites */
extern const uint8_t BITMAP_ALIGN bloque[];      /* 10x14   RGB565 */
extern const uint8_t BITMAP_ALIGN Helado[];      /* 10x11   RGB565 */

/* Enemigo (spritesheets) */
extern const uint8_t BITMAP_ALIGN enemigo_F[];   /* 90x26 -> 3 frames (30x26) */
extern const uint8_t BITMAP_ALIGN enemigo_B[];   /* 90x26 -> 3 frames (30x26) */
extern const uint8_t BITMAP_ALIGN enemigo_L[];   /* 84x26 -> 3 frames (28x26) */
extern const uint8_t BITMAP_ALIGN enemigo_R[];   /* 84x26 -> 3 frames (28x26) */

/* Jugador 1 (spritesheets) */
extern const uint8_t BITMAP_ALIGN Jugador1_F[];  /* 40x27 -> 2 frames (20x27) */
extern const uint8_t BITMAP_ALIGN Jugador1_B[];  /* 48x26 -> 2 frames (24x26) */
extern const uint8_t BITMAP_ALIGN Jugador1_L[];  /* 48x32 -> 2 frames (24x32) */
extern const uint8_t BITMAP_ALIGN Jugador1_R[];  /* 56x32 -> 2 frames (28x32) */


extern const uint8_t BITMAP_ALIGN Jugador1_Romper[];
extern const uint8_t BITMAP_ALIGN Jugador1_Crear[];

/* Jugador 2 (spritesheets) */
extern const uint8_t BITMAP_ALIGN Jugador2_F[];
extern const uint8_t BITMAP_ALIGN Jugador2_B[];
extern const uint8_t BITMAP_ALIGN Jugador2_L[];
extern const uint8_t BITMAP_ALIGN Jugador2_R[];


extern const uint8_t BITMAP_ALIGN Jugador2_Romper[];
extern const uint8_t BITMAP_ALIGN Jugador2_Crear[];


extern const uint8_t BITMAP_ALIGN banano[];
extern const uint8_t BITMAP_ALIGN contadorJ1[];
extern const uint8_t BITMAP_ALIGN contadorJ2[];

extern const uint8_t BITMAP_ALIGN bowser[];
extern const uint8_t BITMAP_ALIGN pesaSprite[];

#endif /* INC_BITMAPS_H_ */

