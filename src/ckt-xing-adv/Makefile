#*************************************************************************
#Title:    CKT-XING-ADV Makefile
#Authors:  Michael Petersen <railfan@drgw.net>
#          Nathan Holmes <maverick@drgw.net>
#License:  GNU General Public License v3
#*************************************************************************

BASE_NAME := ckt-xing-adv
MCU := attiny861
F_CPU := 8000000

FUSE_L ?= 0xE2
FUSE_H ?= 0xD6
FUSE_E ?= 0xFE

PROGRAMMER_TYPE ?= iseavrprog
PROGRAMMER_PORT ?= usb
FIRMWARE_IMG ?= ./release-firmware/ckt-xing-adv-38607ac-10Oct2026.hex
AVRDUDE := avrdude
ISPL_TOOLS := ../../../ispltools/tools/
PYTHON ?= python3

AVRDUDE_CMD = $(AVRDUDE) -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(MCU) -B2 -F
AVRDUDE_SLOW = $(AVRDUDE) -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(MCU) -B32

.PHONY: all flash fuse terminal help

help:
	@echo "make flash ...... flash the firmware"
	@echo "make fuse ....... flash the fuses"
	@echo "make program .... flash fuses and firmware"
	@echo "make terminal ... open avrdude terminal"

fuse:
	@[ "$(FUSE_H)" != "" ] && [ "$(FUSE_L)" != "" ] && [ "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid fuse values."; exit 1; }
	$(AVRDUDE_SLOW) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

flash: $(HEX)
	echo "Writing production firmware [$(FIRMWARE_IMG)]"
	$(AVRDUDE_CMD) -U flash:w:$(FIRMWARE_IMG):i

all: fuse flash

terminal:
	$(AVRDUDE_SLOW) -t

ispl:
	$(PYTHON) $(ISPL_TOOLS)/isplcreate/isplcreate.py -i . -o $(BASE_NAME).sbp
	
spiflash-dev: 
	$(PYTHON) $(ISPL_TOOLS)/spiflash-pyprog/flash.py $(BASE_NAME).sbp

