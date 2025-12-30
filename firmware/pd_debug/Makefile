PROJECT = pd_debug
BUILD_DIR = build
DEVICE = stm32f072cb
LDSCRIPT = stm32f072cbx.ld

SRC_DIR = "."
LIBOPENCM3 = ../libopencm3

SRCS := $(shell find $(SRC_DIR) -name '*.c')
OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o)

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -Os -ggdb -std=c11 -Wall -Wextra -Wno-unused-parameter \
         -I$(LIBOPENCM3)/include -I$(SRC_DIR) \
         -DSTM32F0 -DUSE_USB

LDFLAGS = -L$(LIBOPENCM3)/lib -Wl,--gc-sections -T$(LDSCRIPT) -nostartfiles -static
LDLIBS = -lopencm3_stm32f0

# Build rules
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -mcpu=cortex-m0 -mthumb -MMD -c $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS) $(LIBOPENCM3)/lib/libopencm3_stm32f0.a
	$(CC) -mcpu=cortex-m0 -mthumb -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash write $(BUILD_DIR)/$(PROJECT).bin 0x08000000

clean:
	rm -rf $(BUILD_DIR)

$(LIBOPENCM3)/lib/libopencm3_stm32f0.a:
	$(MAKE) -C $(LIBOPENCM3)

.PHONY: all clean flash
