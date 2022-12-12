# ===================================================================================
# Project:  TinyUPS
# Author:   Stefan Wagner
# Year:     2020
# URL:      https://github.com/wagiminator
# ===================================================================================
# Type "make help" in the command line.
# ===================================================================================

# Input and Output File Names
SKETCH   = TinyUPS.ino
TARGET   = tinyups

# Microcontroller Settings
DEVICE   = attiny13a
CLOCK    = 1200000
LFUSE    = 0x2a
HFUSE    = 0xff

# Programmer Settings
PROGRMR ?= usbasp
TGTDEV   = attiny13

# Toolchain
CC       = avr-gcc
OBJCOPY  = avr-objcopy
OBJDUMP  = avr-objdump
AVRSIZE  = avr-size
AVRDUDE  = avrdude -c $(PROGRMR) -p $(TGTDEV)
CLEAN    = rm -f *.lst *.obj *.cof *.list *.map *.eep.hex *.o *.s *.d

# Compiler Flags
CFLAGS   = -Wall -Os -flto -mmcu=$(DEVICE) -DF_CPU=$(CLOCK) -x c++

# Symbolic Targets
help:
	@echo "Use the following commands:"
	@echo "make all       compile and build $(TARGET).elf/.bin/.hex/.asm for $(DEVICE)"
	@echo "make hex       compile and build $(TARGET).hex for $(DEVICE)"
	@echo "make asm       compile and disassemble to $(TARGET).asm for $(DEVICE)"
	@echo "make bin       compile and build $(TARGET).bin for $(DEVICE)"
	@echo "make upload    compile and upload to $(DEVICE) using $(PROGRMR)"
	@echo "make fuses     burn fuses of $(DEVICE) using $(PROGRMR) programmer"
	@echo "make install   compile, upload and burn fuses for $(DEVICE)"
	@echo "make clean     remove all build files"

all:	buildelf buildbin buildhex buildasm removetemp size

elf:	buildelf removetemp size

bin:	buildelf buildbin removetemp size removeelf

hex:	buildelf buildhex removetemp size removeelf

asm:	buildelf buildasm removetemp size removeelf

flash:	upload fuses

install: upload fuses

upload:	hex
	@echo "Uploading $(TARGET).hex to $(DEVICE) using $(PROGRMR) ..."
	@$(AVRDUDE) -U flash:w:$(TARGET).hex:i

fuses:
	@echo "Burning fuses of $(DEVICE) ..."
	@$(AVRDUDE) -U lfuse:w:$(LFUSE):m  -U hfuse:w:$(HFUSE):m

clean:
	@echo "Cleaning all up ..."
	@$(CLEAN)
	@rm -f $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).asm

buildelf:
	@echo "Compiling $(SKETCH) for $(DEVICE) @ $(CLOCK)Hz ..."
	@$(CC) $(CFLAGS) $(SKETCH) -o $(TARGET).elf

buildbin:
	@echo "Building $(TARGET).bin ..."
	@$(OBJCOPY) -O binary -R .eeprom $(TARGET).elf $(TARGET).bin

buildhex:
	@echo "Building $(TARGET).hex ..."
	@$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex

buildasm:
	@echo "Disassembling to $(TARGET).asm ..."
	@$(OBJDUMP) -d $(TARGET).elf > $(TARGET).asm

size:
	@echo "------------------"
	@echo "FLASH: $(shell $(AVRSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$1 + $$2}') bytes"
	@echo "SRAM:  $(shell $(AVRSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$2 + $$3}') bytes"
	@echo "------------------"

removetemp:
	@echo "Removing temporary files ..."
	@$(CLEAN)

removeelf:
	@echo "Removing $(TARGET).elf ..."
	@rm -f $(TARGET).elf
