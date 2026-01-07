# ===========================================
# Project
# ===========================================
PROJECT     := gimbal-turret
TARGET      := firmware
BUILD_DIR   := bin

# ===========================================
# Toolchain
# ===========================================
CC          := arm-none-eabi-gcc
OBJCOPY     := arm-none-eabi-objcopy
OBJDUMP     := arm-none-eabi-objdump
SIZE        := arm-none-eabi-size

# ===========================================
# MCU
# ===========================================
CPU         := cortex-m4
FPU         := fpv4-sp-d16
FLOAT_ABI   := hard
MCU_FLAGS   := -mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)

# ===========================================
# Directories
# ===========================================
SRC_DIR     := src
INC_DIR     := inc
STARTUP     := $(SRC_DIR)/startup/stm32f411ceu.s

CUBE_DIR    := driver/STM32CubeF4
CMSIS_DIR   := $(CUBE_DIR)/Drivers/CMSIS
HAL_DIR     := $(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver

# ===========================================
# Sources
# ===========================================
C_SOURCES := \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/syscalls.c \
    $(CMSIS_DIR)/Device/ST/STM32F4xx/Source/system_stm32f4xx.c \
    $(SRC_DIR)/system_stubs.c \
    $(HAL_DIR)/Src/stm32f4xx_hal.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_tim.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_uart.c

ASM_SOURCES := $(STARTUP)

# ===========================================
# Objects (all go to bin/)
# ===========================================
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(C_SOURCES))) \
           $(patsubst %.s,$(BUILD_DIR)/%.o,$(notdir $(ASM_SOURCES)))

# ===========================================
# Flags
# ===========================================
CFLAGS := $(MCU_FLAGS) -Wall -Wextra -O2 -ffreestanding \
          -fdata-sections -ffunction-sections \
          -I$(INC_DIR) \
          -I$(CUBE_DIR)/Drivers/CMSIS/Core/Include \
          -I$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
          -I$(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc \
          -DSTM32F411xE -DUSE_HAL_DRIVER

ASFLAGS := $(MCU_FLAGS)

LDFLAGS := $(MCU_FLAGS) -T linker.ld -nostartfiles -Wl,--gc-sections

# ===========================================
# Build rules
# ===========================================
all: $(BUILD_DIR)/$(TARGET).elf \
     $(BUILD_DIR)/$(TARGET).bin \
     $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Generic C compile rule
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: driver/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: driver/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Assembly compile
$(BUILD_DIR)/%.o: src/startup/%.s | $(BUILD_DIR)
	$(CC) $(ASFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary \
		--change-section-address .isr_vector=0x0 \
		--change-section-address .text=0x180 \
		--change-section-address .ARM.exidx=0xc4e0 \
		--only-section .isr_vector \
		--only-section .text \
		--only-section .ARM.exidx \
		$< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD_DIR)

dump:
	$(OBJDUMP) -h -d $(BUILD_DIR)/$(TARGET).elf

.PHONY: all clean dump
