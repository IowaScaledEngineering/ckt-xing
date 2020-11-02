LDSCRIPT = xdk-asf/samd21g18a_flash.ld
BOOTUP = xdk-asf/startup_samd21.o xdk-asf/system_samd21.o
MCUTYPE=__SAMD21G18A__

PROJ_NAME=ckt-xing

ELF=$(PROJ_NAME).elf
BIN=$(PROJ_NAME).bin

OBJS=$(BOOTUP) $(PROJ_NAME).o


# Tools
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

LDFLAGS+= -T$(LDSCRIPT) -mthumb -mcpu=cortex-m0 -Wl,--gc-sections -Wl,--section-start=.text=0x0000
CFLAGS+= -mcpu=cortex-m0 -mthumb -g
CFLAGS+= -I xdk-asf -I xdk-asf/include -I xdk-asf/cmsis -I .
CFLAGS+= -D$(MCUTYPE)

$(PROJ_NAME).bin: $(PROJ_NAME).elf $(PROJ_NAME).lst
	$(OBJCOPY) $< $@ -O binary

$(PROJ_NAME).lst: $(PROJ_NAME).elf
	$(OBJDUMP) -D $^ > $@

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

# compile and generate dependency info
%.o:    %.c
	$(CC) -c $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

%.o:    %.s
	$(AS) $< -o $@

clean:
	rm -f $(OBJS) $(OBJS:.o=.d) $(PROJ_NAME).elf $(PROJ_NAME).bin $(PROJ_NAME).lst startup_stm32f* $(CLEANOTHER)

debug:  $(ELF)
	arm-none-eabi-gdb -iex "target extended-remote localhost:3333" $(ELF)

# pull in dependencies
-include        $(OBJS:.o=.d)

flash:
	arm-none-eabi-gdb -ex 'target extended-remote /dev/ttyACM2' -ex 'monitor swdp_scan' -ex 'attach 1' -ex 'load' -ex 'quit' ckt-xing.elf
	
