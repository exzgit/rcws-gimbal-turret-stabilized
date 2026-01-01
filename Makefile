CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS  = -mcpu=cortex-m4 -mthumb -O0 -g
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -Wall

LDFLAGS = -T linker.ld

TARGET  = firmware

all: $(TARGET).elf $(TARGET).bin

$(TARGET).elf: startup_stm32f411.s main.c
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f *.elf *.bin
