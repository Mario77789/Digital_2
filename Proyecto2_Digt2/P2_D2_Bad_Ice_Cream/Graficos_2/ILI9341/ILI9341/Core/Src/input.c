#include "input.h"
#include <string.h>
#include <stdio.h>

#ifndef INPUT_RX_BUFSZ
#define INPUT_RX_BUFSZ 256
#endif

// ====== Estado interno ======
static UART_HandleTypeDef *g_huart = NULL;
static uint8_t  rx_byte;
static char     segbuf[INPUT_RX_BUFSZ];
static uint16_t seglen = 0;

// Cola muy simple (hasta 4 eventos)
static PlayerEvent q[4];
static volatile uint8_t q_head = 0, q_tail = 0;

static inline bool q_push(const PlayerEvent *e) {
    uint8_t nxt = (uint8_t)((q_head + 1) & 3);
    if (nxt == q_tail) return false; // llena
    q[q_head] = *e; q_head = nxt; return true;
}
bool Input_TryDequeue(PlayerEvent *out_evt) {
    if (q_tail == q_head) return false;
    *out_evt = q[q_tail];
    q_tail = (uint8_t)((q_tail + 1) & 3);
    return true;
}

static inline DirMask calc_dir(uint16_t X, uint16_t Y) {
    DirMask d = DIR_NONE;
    if (X >= TH_HIGH || X >= MAX_SAMPLE) d = (DirMask)(d | DIR_UP);
    else if (X < TH_LOW)                 d = (DirMask)(d | DIR_DOWN);

    if (Y >= TH_HIGH)                    d = (DirMask)(d | DIR_RIGHT);
    else if (Y < TH_LOW)                 d = (DirMask)(d | DIR_LEFT);
    return d;
}

static inline uint16_t clamp_u12(int v) {
    if (v < 0) return 0;
    if (v > MAX_SAMPLE) return MAX_SAMPLE;
    return (uint16_t)v;
}

static void apply_event(uint8_t id, int x, int y, int a, int b) {
    PlayerEvent e;
    e.player = id;
    e.X = clamp_u12(x);
    e.Y = clamp_u12(y);
    e.A = (uint8_t)(a ? 1 : 0);
    e.B = (uint8_t)(b ? 1 : 0);
    e.dir = calc_dir(e.X, e.Y);
    (void)q_push(&e);
}

// Acepta: "M1:X1768Y1763SW0A0B0" (con o sin ';' final).
static void parse_segment(const char *s, uint16_t n) {
    int id=0, x=0, y=0, sw=0, a=0, b=0;

    // Copia segura + recorte de ';'
    char tmp[INPUT_RX_BUFSZ];
    if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
    memcpy(tmp, s, n);
    tmp[n] = '\0';
    char *pc = strchr(tmp, ';');
    if (pc) *pc = '\0';

    // Formato principal (exacto)
    int matched = sscanf(tmp, "M%d:X%dY%dSW%dA%dB%d", &id, &x, &y, &sw, &a, &b);
    if (matched < 6) {
        // Fallback: admite espacios extras entre tokens
        matched = sscanf(tmp, " M%d : X%d Y%d SW%d A%d B%d", &id, &x, &y, &sw, &a, &b);
        if (matched < 6) return;
    }
    if (id != 1 && id != 2) return;

    apply_event((uint8_t)id, x, y, a, b);
}

static void on_byte(uint8_t b) {
    if (seglen >= INPUT_RX_BUFSZ - 1) {
        seglen = 0; // overflow → reset
    }
    segbuf[seglen++] = (char)b;

    if (b == ';') {
        parse_segment(segbuf, seglen);
        seglen = 0;
    } else if (b == '\n' || b == '\r') {
        if (seglen > 0) { parse_segment(segbuf, seglen); seglen = 0; }
    }
}

// ====== API ======
void Input_Init(UART_HandleTypeDef *huart2) {
    g_huart = huart2;
    seglen = 0; q_head = q_tail = 0;
    HAL_UART_Receive_IT(g_huart, &rx_byte, 1);
}

void Input_RxIRQ_Handler(void) {
    // Llamada desde HAL_UART_RxCpltCallback
    on_byte(rx_byte);
    HAL_UART_Receive_IT(g_huart, &rx_byte, 1);
}
