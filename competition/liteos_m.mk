# Topdir 顶层目录
LITEOSROOT=liteos_m
LIB_DIR=lib

# Build directory for liteos_m library
LITEOS_BUILD_DIR = liteos_m/build
LITEOS_LIB = $(LIB_DIR)/libliteos_m.a

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
AR = $(PREFIX)ar

# Common 内核源文件及头文件目录
LITEOS_C_SOURCES =  \
    $(wildcard $(LITEOSROOT)/kernel/src/*.c) \
    $(wildcard $(LITEOSROOT)/kernel/src/mm/*.c) \
    $(wildcard $(LITEOSROOT)/components/cpup/*.c) \
    $(wildcard $(LITEOSROOT)/components/power/*.c) \
    $(wildcard $(LITEOSROOT)/components/backtrace/*.c) \
    $(wildcard $(LITEOSROOT)/components/exchook/*.c) \
    $(wildcard $(LITEOSROOT)/components/signal/*.c) \
    $(wildcard $(LITEOSROOT)/utils/*.c)

# Third party related 第三方依赖文件及头文件目录
LITEOS_C_SOURCES += \
    $(wildcard $(LITEOSROOT)/third_party/bounds_checking_function/src/*.c) \
    $(wildcard $(LITEOSROOT)/kal/cmsis/*.c) \
    $(wildcard $(LITEOSROOT)/kal/posix/src/*.c)

# Arch related
LITEOS_ASM_SOURCES = $(wildcard $(LITEOSROOT)/arch/arm/cortex-m4/gcc/*.s)
LITEOS_C_SOURCES += $(wildcard $(LITEOSROOT)/arch/arm/cortex-m4/gcc/*.c)

# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DSTM32F407xx \
-DSTM32F40_41xxx

# LiteOS include paths
LITEOS_C_INCLUDES =  \
    -I$(LITEOSROOT)/utils \
    -I$(LITEOSROOT)/kernel/include \
    -I$(LITEOSROOT)/components/cpup \
    -I$(LITEOSROOT)/components/power \
    -I$(LITEOSROOT)/components/backtrace \
    -I$(LITEOSROOT)/components/exchook \
    -I$(LITEOSROOT)/components/signal \
    -I$(LITEOSROOT)/components/fs/vfs

LITEOS_C_INCLUDES += \
    -I$(LITEOSROOT)/third_party/bounds_checking_function/include \
    -I$(LITEOSROOT)/third_party/bounds_checking_function/src \
    -I$(LITEOSROOT)/third_party/cmsis/CMSIS/RTOS2/Include \
    -I$(LITEOSROOT)/third_party/cmsis/CMSIS/Core/Include \
    -I$(LITEOSROOT)/third_party/musl/porting/liteos_m/kernel/include \
    -I$(LITEOSROOT)/kal/cmsis \
    -I$(LITEOSROOT)/kal/posix/include \
    -I$(LITEOSROOT)/kal/posix/musl_src/internal \
    -I$(LITEOSROOT)/arch/include \
    -I$(LITEOSROOT)/arch/arm/cortex-m4/gcc

LITEOS_C_INCLUDES += \
    -ISTM32F4xx_FWLib/inc \
    -IUSER \
    -I./ \
    -I.

# MCU settings (from main Makefile)
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Optimization and debug flags
OPT = -O0
DEBUG = 1

# Compiler flags for liteos_m (with debug info)
LITEOS_CFLAGS = $(MCU) $(LITEOS_C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
LITEOS_CFLAGS += -nostdinc -nostdlib
ifeq ($(DEBUG), 1)
LITEOS_CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
LITEOS_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Assembler flags
LITEOS_ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
LITEOS_ASFLAGS += -imacros $(LITEOSROOT)/kernel/include/los_config.h -DCLZ=CLZ
ifeq ($(DEBUG), 1)
LITEOS_ASFLAGS += -g -gdwarf-2
endif

# Generate dependency information
LITEOS_ASFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Object files (preserve source directory structure to avoid name collisions)
LITEOS_C_OBJECTS = $(addprefix $(LITEOS_BUILD_DIR)/,$(LITEOS_C_SOURCES:.c=.o))
LITEOS_ASM_OBJECTS = $(addprefix $(LITEOS_BUILD_DIR)/,$(LITEOS_ASM_SOURCES:.s=.o))
LITEOS_OBJECTS = $(LITEOS_C_OBJECTS) $(LITEOS_ASM_OBJECTS)
LITEOS_DEPS = $(LITEOS_OBJECTS:.o=.d)

# VPATH for source files
vpath %.c $(sort $(dir $(LITEOS_C_SOURCES)))
vpath %.s $(sort $(dir $(LITEOS_ASM_SOURCES)))

# Create build directory
$(LITEOS_BUILD_DIR):
	@mkdir -p $@


# Compile C files
$(LITEOS_BUILD_DIR)/%.o: %.c | $(LITEOS_BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "LITEOS CC $<"
	@$(CC) -c $(LITEOS_CFLAGS) $(C_DEFS) $< -o $@

# Compile ASM files
$(LITEOS_BUILD_DIR)/%.o: %.s | $(LITEOS_BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "LITEOS AS $<"
	@$(AS) -c $(LITEOS_CFLAGS) $(C_DEFS) $< -o $@

# Create static library
$(LITEOS_LIB): $(LITEOS_OBJECTS)
	@mkdir -p $(LIB_DIR)
	@echo "LITEOS AR $@"
	@$(AR) rcs $@ $^
# Target to build liteos_m library
lib: $(LITEOS_LIB)

# dependencies
-include $(LITEOS_DEPS)


clean:
	rm -rf $(LITEOS_BUILD_DIR) $(LITEOS_LIB)

