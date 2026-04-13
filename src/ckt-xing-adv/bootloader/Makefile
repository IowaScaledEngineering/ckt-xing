DEVICE  = attiny861
F_CPU   = 8000000  # Hz
FUSE_L  = 0xE2
FUSE_H  = 0xD6
FUSE_E  = 0xFE

BOOTLOADER_HEX = loader-bef048bd-1Jan2025.hex

PROGRAMMER_TYPE=iseavrprog
PROGRAMMER_PORT=usb

AVRDUDE = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B8 -F
AVRDUDE_SLOW = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B32


help:
	@echo "make flash ..... flash the firmware"
	@echo "make fuse ...... flash the fuses"

program: fuse flash

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" -a "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid Fuse values."; exit 1; }
	$(AVRDUDE_SLOW) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

read:
	$(AVRDUDE) -v

# rule for uploading firmware:
flash:
	$(AVRDUDE) -U flash:w:$(BOOTLOADER_HEX):i

reset:
	$(AVRDUDE_SLOW) -U eeprom:w:bl-reset.eep:i
