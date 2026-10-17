#ifndef FARM_HARVEST_RECOVERY_H
#define FARM_HARVEST_RECOVERY_H
#include <stdint.h>
#define FARM_HARVEST_MAX_ATTEMPTS 2U /* 首次 + 1次重试 */
#define FARM_HARVEST_WAIT_MS 1000U
#define FARM_HARVEST_CLEAR_MS 500U
typedef enum { FARM_HARVEST_WAIT, FARM_HARVEST_CLEARED,
               FARM_HARVEST_RETRY, FARM_HARVEST_SKIP } FarmHarvestDecision;
typedef struct {
    uint8_t attempts, clearing;
    uint32_t wait_since, clear_since;
} FarmHarvestRecovery;

static inline int FarmHarvest_StartAttempt(FarmHarvestRecovery *p)
{
    if(p->attempts>=FARM_HARVEST_MAX_ATTEMPTS)return 0;
    p->attempts++;
    return 1;
}
static inline void FarmHarvest_BeginWait(FarmHarvestRecovery *p,uint32_t now)
{
    p->wait_since=now;p->clearing=0;
}
/* Called only after motion completes. Missing/out-of-range TOF is NOT evidence
 * that a red fruit disappeared. retry_ready requires a new valid in-range frame. */
static inline FarmHarvestDecision FarmHarvest_Check(FarmHarvestRecovery *p,
    uint32_t now,int red,int retry_ready)
{
    if(!red) {
        if(!p->clearing){p->clearing=1;p->clear_since=now;}
        if((uint32_t)(now-p->clear_since)>=FARM_HARVEST_CLEAR_MS) {
            p->attempts=0;return FARM_HARVEST_CLEARED;
        }
    } else p->clearing=0;
    if((uint32_t)(now-p->wait_since)<FARM_HARVEST_WAIT_MS)return FARM_HARVEST_WAIT;
    /* Allow a pending 500ms absence confirmation to finish. */
    if(!red)return FARM_HARVEST_WAIT;
    if(p->attempts>=FARM_HARVEST_MAX_ATTEMPTS || !retry_ready)return FARM_HARVEST_SKIP;
    return FARM_HARVEST_RETRY;
}
/* Keep ignoring a skipped fruit while driving past it. Rearm only after a
 * stable absence, so the same red target cannot immediately stop the vehicle. */
static inline int FarmHarvest_PassedSkippedTarget(FarmHarvestRecovery *p,uint32_t now,int red)
{
    if(red){p->clearing=0;return 0;}
    if(!p->clearing){p->clearing=1;p->clear_since=now;}
    if((uint32_t)(now-p->clear_since)<FARM_HARVEST_CLEAR_MS)return 0;
    p->attempts=0;p->clearing=0;return 1;
}
#endif
