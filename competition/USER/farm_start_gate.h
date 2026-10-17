#ifndef FARM_START_GATE_H
#define FARM_START_GATE_H
#include <stdint.h>
#include "farm_speed_config.h"

/* White start and unconnected/pulled-up sensors both read 0xff.
 * Permit ONE bounded departure attempt, then require stable black detection.
 * A timeout remains stopped until firmware reset.
 */
#define FARM_START_TIMEOUT_MS 3000U
#define FARM_START_LINE_CONFIRM_MS 50U
typedef enum { FARM_START_LEAVING, FARM_START_TRACKING, FARM_START_TIMEOUT } FarmStartPhase;
typedef struct {
    FarmStartPhase phase;
    uint32_t moving_ms;
    uint32_t black_ms;
    uint8_t was_moving;
} FarmStartGate;

/* Called from the motor task AFTER initialization. active=0 when harvesting
 * owns the motors. elapsed_ms is actual time since the previous call.
 * Returns 1 to replace line-following speed with *speed on both wheels.
 */
static inline int FarmStartGate_Update(FarmStartGate *g, uint8_t line,
                                       uint8_t active, uint32_t elapsed_ms,
                                       int16_t *speed)
{
    *speed=0;
    if(g->phase==FARM_START_TIMEOUT) return 1;
    if(g->phase==FARM_START_TRACKING) return 0;
    if(g->was_moving) {
        uint32_t remaining=FARM_START_TIMEOUT_MS-g->moving_ms;
        g->moving_ms+=elapsed_ms<remaining?elapsed_ms:remaining;
    }
    g->was_moving=0;
    if(!active) { g->black_ms=0;return 1; }
    if(line!=0xff) {
        uint32_t remaining=FARM_START_LINE_CONFIRM_MS-g->black_ms;
        g->black_ms+=elapsed_ms<remaining?elapsed_ms:remaining;
        if(g->black_ms>=FARM_START_LINE_CONFIRM_MS) {
            g->phase=FARM_START_TRACKING;return 0;
        }
    } else g->black_ms=0;
    if(g->moving_ms>=FARM_START_TIMEOUT_MS) {
        g->phase=FARM_START_TIMEOUT;return 1;
    }
    *speed=FARM_START_SPEED;
    g->was_moving=1;
    return 1;
}
#endif
