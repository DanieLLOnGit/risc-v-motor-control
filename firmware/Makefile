# Firmware Makefile — Robo-V SoC
# Targets RV32I bare-metal.  Detects riscv32 or riscv64 toolchain automatically.

RISCV_PATH := /opt/homebrew/var/homebrew/tmp/.cellar/riscv-gnu-toolchain/main/bin

ifneq (,$(wildcard $(RISCV_PATH)/riscv32-unknown-elf-gcc))
    CC := $(RISCV_PATH)/riscv32-unknown-elf-gcc
    OBJCOPY := $(RISCV_PATH)/riscv32-unknown-elf-objcopy
    OBJDUMP := $(RISCV_PATH)/riscv32-unknown-elf-objdump
else ifneq (,$(wildcard $(RISCV_PATH)/riscv64-unknown-elf-gcc))
    CC := $(RISCV_PATH)/riscv64-unknown-elf-gcc
    OBJCOPY := $(RISCV_PATH)/riscv64-unknown-elf-objcopy
    OBJDUMP := $(RISCV_PATH)/riscv64-unknown-elf-objdump
else ifneq (,$(shell which riscv32-unknown-elf-gcc 2>/dev/null))
    CC := riscv32-unknown-elf-gcc
    OBJCOPY := riscv32-unknown-elf-objcopy
    OBJDUMP := riscv32-unknown-elf-objdump
else
    CC := riscv64-unknown-elf-gcc
    OBJCOPY := riscv64-unknown-elf-objcopy
    OBJDUMP := riscv64-unknown-elf-objdump
endif

ARCH := rv32i
ABI := ilp32
CFLAGS := -march=$(ARCH) -mabi=$(ABI) -O2 \
          -nostdlib -nostartfiles -ffreestanding \
          -Wall -Wextra
LDFLAGS := -T link.ld

SRCS := crt0.s main.c
TARGET := firmware

.PHONY: all clean disasm

all: $(TARGET).hex

$(TARGET).elf: $(SRCS) link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS)
	@echo "Built $@ ($(shell $(OBJDUMP) -h $@ | grep '\.text' | awk '{print $$3}') bytes .text)"

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O verilog --verilog-data-width=4 $< $@
	@echo "Generated $@"

disasm: $(TARGET).elf
	$(OBJDUMP) -d $<

clean:
	rm -f $(TARGET).elf $(TARGET).hex
