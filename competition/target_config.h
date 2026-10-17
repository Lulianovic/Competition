/**@defgroup los_config System configuration items
 * @ingroup kernel
 */

#ifndef _TARGET_CONFIG_H
#define _TARGET_CONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __cplusplus */

/*=============================================================================
                                        System clock module configuration
=============================================================================*/
#define OS_SYS_CLOCK SystemCoreClock
#define LOSCFG_BASE_CORE_TICK_PER_SECOND (1000UL)
#define LOSCFG_BASE_CORE_TICK_HW_TIME 0
#define LOSCFG_BASE_CORE_TICK_WTIMER 0
#define LOSCFG_BASE_CORE_TICK_RESPONSE_MAX SysTick_LOAD_RELOAD_Msk

/*=============================================================================
                                        Hardware interrupt module configuration
=============================================================================*/
#define LOSCFG_PLATFORM_HWI 0
#define LOSCFG_USE_SYSTEM_DEFINED_INTERRUPT 0
#define LOSCFG_PLATFORM_HWI_LIMIT 128

/*=============================================================================
                                        Openharmony Kernel configuration
=============================================================================*/

/*=============================================================================
                                       Task module configuration
=============================================================================*/
#define LOSCFG_BASE_CORE_TSK_LIMIT 24
#define LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE (0x500U)
#define LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE (0x2D0U)
#define LOSCFG_BASE_CORE_TSK_MIN_STACK_SIZE (0x130U)
#define LOSCFG_BASE_CORE_TIMESLICE 1
#define LOSCFG_BASE_CORE_TIMESLICE_TIMEOUT 20000
/*=============================================================================
                                       Semaphore module configuration
=============================================================================*/
#define LOSCFG_BASE_IPC_SEM 1
#define LOSCFG_BASE_IPC_SEM_LIMIT 48

/*=============================================================================
                                       Mutex module configuration
=============================================================================*/
#define LOSCFG_BASE_IPC_MUX 1
#define LOSCFG_BASE_IPC_MUX_LIMIT 24
/*=============================================================================
                                       Queue module configuration
=============================================================================*/
#define LOSCFG_BASE_IPC_QUEUE 1
#define LOSCFG_BASE_IPC_QUEUE_LIMIT 24
/*=============================================================================
                                       Software timer module configuration
=============================================================================*/
#define LOSCFG_BASE_CORE_SWTMR 1
#define LOSCFG_BASE_CORE_SWTMR_ALIGN 1
#define LOSCFG_BASE_CORE_SWTMR_LIMIT 48
/*=============================================================================
                                       Memory module configuration
=============================================================================*/
#define LOSCFG_MEM_MUL_POOL 1
#define OS_SYS_MEM_NUM 20
/*=============================================================================
                                       Exception module configuration
=============================================================================*/
#define LOSCFG_PLATFORM_EXC 1

/*=============================================================================
                                        TestSuite configuration
=============================================================================*/
#define LOSCFG_TEST 0

#ifndef LOSCFG_BACKTRACE_TYPE
#define LOSCFG_BACKTRACE_TYPE 1
#endif
/**
 * @ingroup los_config
 * Configuration backtrace depth.
 */
#ifndef LOSCFG_BACKTRACE_DEPTH
#define LOSCFG_BACKTRACE_DEPTH 15
#endif
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _TARGET_CONFIG_H */
