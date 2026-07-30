# Makefile for moteus_g431 — ARM GCC build
# Usage:
#   make          — build
#   make flash    — flash via st-link (st-flash or openocd)
#   make clean    — clean

# Toolchain
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
LD      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

# Project
TARGET  = moteus_g431
BUILD   = build

# MCU flags
CPU     = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEFS    = -DUSE_HAL_DRIVER -DSTM32G431xx

# Includes
INCLUDES = \
  -ICore/Inc \
  -IMoteus/protocol \
  -IMoteus/control \
  -IMoteus/hw \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include

# Sources
C_SOURCES = \
  Core/Src/main.c \
  Core/Src/peripherals.c \
  Moteus/protocol/moteus_protocol.c \
  Moteus/protocol/register_handler.c \
  Moteus/control/foc.c \
  Moteus/control/motor_control.c \
  Moteus/hw/encoder.c \
  Moteus/hw/can_interface.c \
  Drivers/CMSIS/DeviceSupport/system_stm32g4xx.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c

ASM_SOURCES = \
  Drivers/CMSIS/DeviceSupport/startup_stm32g431cbux.s

# Linker
LDSCRIPT = STM32G431CBUx_FLASH.ld

# Flags
OPT     = -Os
CFLAGS  = $(CPU) $(DEFS) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ASFLAGS = $(CPU) $(OPT) -Wall -fdata-sections -ffunction-sections
LDFLAGS = $(CPU) -specs=nano.specs -specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT) -Wl,--gc-sections -lm
LDFLAGS += -Wl,-Map=$(BUILD)/$(TARGET).map,--cref

# Object files
OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Rules
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
	$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

# Flash via st-flash (ST-Link)
flash: $(BUILD)/$(TARGET).bin
	st-flash write $< 0x08000000

# Flash via OpenOCD
flash-openocd: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
	  -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# Debug via OpenOCD + GDB
debug: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg &
	$(PREFIX)gdb -ex "target remote :3333" -ex "monitor reset halt" $<

-include $(wildcard $(BUILD)/*.d)

.PHONY: all clean flash flash-openocd debug
