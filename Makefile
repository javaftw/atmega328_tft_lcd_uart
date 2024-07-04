

# Author: henniek 18-06-2018

# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega328p 
CLOCK      = 16000000
PROGRAMMER = -c avrispmkII -P usb
#SOURCES	   = main.c test.S
#OBJECTS    = main.o 
SRC	    = main.c
ASRC	    = test.S
OBJECTS     = $(SRC:.c=.o) $(ASRC:.S=.o)
TARGET	    = main
#::::::::::::::::::::http://www.engbedded.com/fusecalc/
FUSES      = -U lfuse:w:0xde:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m #--EXT 16MHz XTAL/ERASE EEPROM
#FUSES      = -U lfuse:w:0xd2:m -U hfuse:w:0xd8:m -U efuse:w:0x07:m #--INT 8MHz XTAL/ERASE EEPROM# Name: Makefile

######################################################################
######################################################################

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -v #-vvv #-B10
#CXX = avr-gcc -print-file-name=include -v -Wall -g -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) 
CXX = avr-gcc -Wall -g -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)  

# symbolic targets:
all:	$(TARGET).hex

.c.o:
	$(CXX) -c  $< -o $@

.S.o:
	$(CXX) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(CXX) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

# if you use a bootloader, change the command below appropriately:
#load: all
#	bootloadHID main.hex

upload: flash fuse

clean:
	rm -f $(TARGET).hex $(TARGET).elf $(OBJECTS)

# file targets:
$(TARGET).elf: $(OBJECTS)
	$(CXX) -o $(TARGET).elf $(OBJECTS)

$(TARGET).hex: $(TARGET).elf
	rm -f $(TARGET).hex
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
# Note: see https://www.nongnu.org/avr-libc/user-manual/using_tools.html
# for machine-specific options for the AVR (-mmcu=architecture)
# "avr5" = "Enhanced" CPU core, 16 KB to 64 KB of ROM
disasm:	$(TARGET).hex
	avr-objdump -D -m avr5 $(TARGET).hex > $(TARGET).dmp

cpp:
	$(CXX) -E $(TARGET).c