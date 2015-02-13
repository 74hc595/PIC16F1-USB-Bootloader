# Makefile for SDCC + PIC16F1xxx with USB CDC bootloader
# Matt Sarnoff (msarnoff.org)
# February 10, 2015
#
# Run `make` to build the project as a .hex file.
# Run `make flash` to program the device.
#

########## Project-specific definitions ##########

# Project name
OUT = blink

# Source files to compile
SRC = blink.c

# Assembly files to assemble
ASM = crt_bootloader_512.S

# The SDCC-specific (case sensitive) part number of the chip
# (use `make list-devices` if not known)
CC_DEVICE = 16f1454



########## Build settings ##########

CFLAGS = --use-non-free --stack-size 8 --opt-code-speed --std-sdcc99 -DNO_BIT_DEFINES
LDFLAGS = -Wl-s16f145x_bootloader_512.lkr
PROG = ../usb16f1prog	# change this once you've installed the script
CC = sdcc
AS = gpasm
LD = gplink
DASM = gpdasm
CC_FAMILY = pic14


########## Make rules ##########

OBJ = $(SRC:.c=.o) $(ASM:.S=.o)
HEX = $(OUT).hex

# Compile C file
.c.o:
	$(CC) $(CFLAGS) -m$(CC_FAMILY) -p$(CC_DEVICE) -c $<

.S.o:
	$(AS) -p$(CC_DEVICE) -o $@ -c $<

# Link
$(HEX): $(OBJ)
	$(CC) $(LDFLAGS) -o $(HEX) $(CFLAGS) -m$(CC_FAMILY) -p$(CC_DEVICE) $(OBJ)

# Disassemble
dis: $(HEX)
	$(DASM) -p p$(CC_DEVICE) $(HEX) | less

# Flash
flash: $(HEX)
	$(PROG) $(HEX)

# Clean
clean:
	rm -f $(SRC:.c=.asm) $(SRC:.c=.lst) $(SRC:.c=.o) $(ASM:.S=.lst) $(ASM:.S=.o) $(HEX) $(OUT).cod $(OUT).lst

.PHONY: all flash clean dis 

