#ifndef FARM_PEPPER_PRECHECK_H
#define FARM_PEPPER_PRECHECK_H
#include <stdint.h>
#define FARM_NO_RED_CONFIRM_MS 500U
typedef struct {
    uint8_t observing;
    uint8_t complete;
    uint32_t since_ms;
} FarmPepperPrecheck;

/* Called only on a completed color read after motor initialization.
 * A red target or active harvesting resets the no-red observation window.
 */
static inline uint8_t FarmPepperPrecheck_Update(FarmPepperPrecheck *p,
    uint8_t red, uint8_t harvesting, uint32_t now_ms)
{
    if(p->complete) return 1;
    if(red || harvesting) { p->observing=0;return 0; }
    if(!p->observing) { p->observing=1;p->since_ms=now_ms; }
    if((uint32_t)(now_ms-p->since_ms)>=FARM_NO_RED_CONFIRM_MS) p->complete=1;
    return p->complete;
}
#endif
