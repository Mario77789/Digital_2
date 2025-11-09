#ifndef INC_BITMAPS_H_
#define INC_BITMAPS_H_




#include <stdint.h>

#ifndef BITMAP_ALIGN
#define BITMAP_ALIGN __attribute__((aligned(4)))
#endif

/* Externs of all sprite maps */

extern const uint8_t BITMAP_ALIGN ParqueoArriba_map[];
extern const uint8_t BITMAP_ALIGN ParqueoABAJO_map[];
extern const uint8_t BITMAP_ALIGN Carroparqueo_map[];
extern const uint8_t BITMAP_ALIGN SemaforoVERDE_map[];
extern const uint8_t BITMAP_ALIGN SemaforoROJO_map[];
extern const uint8_t BITMAP_ALIGN Dp_0_map[];
extern const uint8_t BITMAP_ALIGN Dp_1_map[];
extern const uint8_t BITMAP_ALIGN Dp_2_map[];
extern const uint8_t BITMAP_ALIGN Dp_3_map[];
extern const uint8_t BITMAP_ALIGN Dp_4_map[];
extern const uint8_t BITMAP_ALIGN Dp_5_map[];
extern const uint8_t BITMAP_ALIGN Dp_6_map[];
extern const uint8_t BITMAP_ALIGN Dp_7_map[];
extern const uint8_t BITMAP_ALIGN Dp_8_map[];
extern const uint8_t BITMAP_ALIGN Dp_9_map[];

/* Quick access to 0..9 */
extern const uint8_t* const DIGITS_MAP[10];

#endif /* BITMAPS_H */


