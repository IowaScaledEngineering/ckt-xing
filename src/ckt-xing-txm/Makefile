#*************************************************************************
#Title:    CKT-XING-ADV Makefile
#Authors:  Michael Petersen <railfan@drgw.net>
#          Nathan Holmes <maverick@drgw.net>
#License:  GNU General Public License v3
#*************************************************************************

BASE_NAME := ckt-xing-txm
MCU := attiny861
F_CPU := 8000000

FUSE_L ?= 0xE2
FUSE_H ?= 0xD6
FUSE_E ?= 0xFE

PROGRAMMER_TYPE ?= iseavrprog
PROGRAMMER_PORT ?= usb

CPPFLAGS := -DF_CPU=$(F_CPU) -I.
DEPFLAGS := -MMD -MP
CFLAGS := -Wall -Wextra -Os -std=gnu99 -ffunction-sections -fdata-sections -flto -mmcu=$(MCU)
LDFLAGS := -Wl,--gc-sections -mmcu=$(MCU)

CC := avr-gcc
OBJCOPY := avr-objcopy
SIZE := avr-size
OBJDUMP := avr-objdump
AVRDUDE := avrdude

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)
DEPS := $(OBJS:.o=.d)
TARGET := $(BASE_NAME).elf
HEX := $(BASE_NAME).hex

-include $(DEPS)

AVRDUDE_CMD = $(AVRDUDE) -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(MCU) -B2 -F
AVRDUDE_SLOW = $(AVRDUDE) -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(MCU) -B32

.PHONY: all clean help hex flash fuse program read size terminal disasm firmware

all: hex

help:
	@echo "make hex ........ build $(HEX)"
	@echo "make flash ...... flash the firmware"
	@echo "make fuse ....... flash the fuses"
	@echo "make program .... flash fuses and firmware"
	@echo "make firmware ... flash firmware from a previously built .hex file"
	@echo "make read ....... read fuse settings"
	@echo "make size ....... memory usage"
	@echo "make clean ...... delete objects and build artifacts"
	@echo "make terminal ... open avrdude terminal"

%.o: %.c
	$(CC) $(CPPFLAGS) $(DEPFLAGS) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS)

$(HEX): $(TARGET)
	@rm -f $(HEX) $(BASE_NAME).eep.hex
	$(OBJCOPY) -j .text -j .data -O ihex $< $(HEX)
	$(SIZE) --format=avr --mcu=$(MCU) $<

hex: $(HEX)

fuse:
	@[ "$(FUSE_H)" != "" ] && [ "$(FUSE_L)" != "" ] && [ "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid fuse values."; exit 1; }
	$(AVRDUDE_SLOW) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

flash: $(HEX)
	$(AVRDUDE_CMD) -U flash:w:$(HEX):i

program: fuse flash

firmware:
	@echo "Usage: make firmware HEX=/path/to/file.hex"
	@test -n "$(HEX)" || { echo "HEX must be set."; exit 1; }
	$(AVRDUDE_CMD) -U flash:w:$(HEX):i

read:
	$(AVRDUDE_CMD) -v

size:
	$(SIZE) --format=avr --mcu=$(MCU) $(TARGET)

terminal:
	$(AVRDUDE_SLOW) -t

clean:
	rm -f $(BASE_NAME).hex $(BASE_NAME).elf $(BASE_NAME).eep.hex $(BASE_NAME).map $(BASE_NAME).lst *.o *~

disasm: $(TARGET)
	$(OBJDUMP) -d $(TARGET)

release: hex
	@echo "Release artifacts are not active for this minimal build; use the generated $(HEX) file as the release binary."
