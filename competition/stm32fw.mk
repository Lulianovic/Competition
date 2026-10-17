# Topdir 顶层目录
STM32FWROOT=STM32F4xx_FWLib
LITEOSROOT=liteos_m
# Build directory for STM32FW_m library
STM32FW_BUILD_DIR = $(STM32FWROOT)/build
STM32FW_LIB = lib/libstm32fwlib.a

PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif

# Common 内核源文件及头文件目录
STM32FW_C_SOURCES =  \
STM32F4xx_FWLib/src/misc.c    \
STM32F4xx_FWLib/src/stm32f4xx_adc.c    \
STM32F4xx_FWLib/src/stm32f4xx_can.c    \
STM32F4xx_FWLib/src/stm32f4xx_crc.c    \
STM32F4xx_FWLib/src/stm32f4xx_cryp_aes.c    \
STM32F4xx_FWLib/src/stm32f4xx_cryp.c    \
STM32F4xx_FWLib/src/stm32f4xx_cryp_des.c    \
STM32F4xx_FWLib/src/stm32f4xx_cryp_tdes.c    \
STM32F4xx_FWLib/src/stm32f4xx_dac.c    \
STM32F4xx_FWLib/src/stm32f4xx_dbgmcu.c    \
STM32F4xx_FWLib/src/stm32f4xx_dcmi.c    \
STM32F4xx_FWLib/src/stm32f4xx_dma2d.c    \
STM32F4xx_FWLib/src/stm32f4xx_dma.c    \
STM32F4xx_FWLib/src/stm32f4xx_exti.c    \
STM32F4xx_FWLib/src/stm32f4xx_flash.c    \
STM32F4xx_FWLib/src/stm32f4xx_flash_ramfunc.c    \
STM32F4xx_FWLib/src/stm32f4xx_fsmc.c    \
STM32F4xx_FWLib/src/stm32f4xx_gpio.c    \
STM32F4xx_FWLib/src/stm32f4xx_hash.c    \
STM32F4xx_FWLib/src/stm32f4xx_hash_md5.c    \
STM32F4xx_FWLib/src/stm32f4xx_hash_sha1.c    \
STM32F4xx_FWLib/src/stm32f4xx_i2c.c    \
STM32F4xx_FWLib/src/stm32f4xx_iwdg.c    \
STM32F4xx_FWLib/src/stm32f4xx_ltdc.c    \
STM32F4xx_FWLib/src/stm32f4xx_pwr.c    \
STM32F4xx_FWLib/src/stm32f4xx_rcc.c    \
STM32F4xx_FWLib/src/stm32f4xx_rng.c    \
STM32F4xx_FWLib/src/stm32f4xx_rtc.c    \
STM32F4xx_FWLib/src/stm32f4xx_sai.c    \
STM32F4xx_FWLib/src/stm32f4xx_sdio.c    \
STM32F4xx_FWLib/src/stm32f4xx_spi.c    \
STM32F4xx_FWLib/src/stm32f4xx_syscfg.c    \
STM32F4xx_FWLib/src/stm32f4xx_tim.c    \
STM32F4xx_FWLib/src/stm32f4xx_usart.c    \
STM32F4xx_FWLib/src/stm32f4xx_wwdg.c    \


# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DSTM32F407xx \
-DSTM32F40_41xxx

# STM32FW include paths
STM32FW_C_INCLUDES =  \
    -I$(LITEOSROOT)/third_party/cmsis/CMSIS/Core/Include \
    -ISTM32F4xx_FWLib/inc \
    -IUSER \
    -I./


# MCU settings (from main Makefile)
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Optimization and debug flags
OPT = -O0
DEBUG = 1

# Compiler flags for STM32FW_m (with debug info)
STM32FW_CFLAGS = $(MCU) $(STM32FW_C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
STM32FW_CFLAGS += -nostdlib
ifeq ($(DEBUG), 1)
STM32FW_CFLAGS += -g -gdwarf-2
endif

# Assembler flags
STM32FW_ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
STM32FW_ASFLAGS += -imacros $(STM32FWROOT)/kernel/include/los_config.h -DCLZ=CLZ
ifeq ($(DEBUG), 1)
STM32FW_ASFLAGS += -g -gdwarf-2
endif

# Object files
STM32FW_C_OBJECTS = $(addprefix $(STM32FW_BUILD_DIR)/,$(notdir $(STM32FW_C_SOURCES:.c=.o)))
STM32FW_ASM_OBJECTS = $(addprefix $(STM32FW_BUILD_DIR)/,$(notdir $(STM32FW_ASM_SOURCES:.s=.o)))
STM32FW_OBJECTS = $(STM32FW_C_OBJECTS) $(STM32FW_ASM_OBJECTS)

# VPATH for source files
vpath %.c $(sort $(dir $(STM32FW_C_SOURCES)))
vpath %.s $(sort $(dir $(STM32FW_ASM_SOURCES)))

# Create build directory
$(STM32FW_BUILD_DIR):
	@mkdir -p $@


# Compile C files
$(STM32FW_BUILD_DIR)/%.o: %.c | $(STM32FW_BUILD_DIR)
	@echo "STM32FW CC $<"
	@$(CC) -c $(STM32FW_CFLAGS) $(C_DEFS) $< -o $@

# Compile ASM files
$(STM32FW_BUILD_DIR)/%.o: %.s | $(STM32FW_BUILD_DIR)
	@echo "STM32FW AS $<"
	@$(AS) -c $(STM32FW_CFLAGS) $(C_DEFS) $< -o $@

# Create static library
$(STM32FW_LIB): $(STM32FW_OBJECTS)
	@echo "STM32FW AR $@"
	@$(AR) rcs $@ $^
	@rm -rf $(STM32FW_BUILD_DIR) 
# Target to build STM32FW_m library
lib: $(STM32FW_LIB)

.PHONY: clean

clean:
	rm -rf $(STM32FW_BUILD_DIR) $(STM32FW_LIB)

