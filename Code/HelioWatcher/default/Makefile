###############################################################################
# Makefile for the project HelioWatcher
###############################################################################

## General Flags
PROJECT = HelioWatcher
MCU = atmega1280
TARGET = HelioWatcher.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                               -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm -Wl,-Map=HelioWatcher.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Library Directories
LIBDIRS = -L"C:\Users\Jeremy Blum\Dropbox\08 Senior Second Semester (11-12)\ECE 4760 - Designing with Microcontrollers\HelioWatcher\Code\HelioWatcher\AVRlib" 

## Libraries
LIBS = -lm 

## Objects that must be built in order to link
OBJECTS = rprintf.o vt100.o debug.o buffer.o uart_mod.o i2csw.o nmeap01.o twimaster.o spa.o HelioWatcher.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) HelioWatcher.hex HelioWatcher.eep HelioWatcher.lss## Compile
rprintf.o: ../AVRlib/rprintf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

vt100.o: ../AVRlib/vt100.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

debug.o: ../AVRlib/debug.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

buffer.o: ../AVRlib/buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart_mod.o: ../uart_mod.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2csw.o: ../AVRlib/i2csw.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

nmeap01.o: ../nmeap01.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twimaster.o: ../i2cmaster/twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spa.o: ../spa.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

HelioWatcher.o: ../HelioWatcher.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) HelioWatcher.elf dep/* HelioWatcher.hex HelioWatcher.eep HelioWatcher.lss HelioWatcher.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

