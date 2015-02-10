# Mac OS X Makefile for PIC programming using SDCC
# Matt Sarnoff (msarnoff.org)
# November 7, 2014
#
# Run `make` to build the project as a .hex file.
# Run `make flash` to program the device.
#
# MPLAB X is required if using a PICkit 3 to program the device.
# This Makefile assumes it's installed in /Applications/microchip.

########## Project-specific definitions ##########

# Project name
OUT = blink

# Source files to compile
SRC = blink.c

# Assembly files to assemble
ASM = crt_bootloader_4k.S

# Either `pic14` for 14-bit devices or `pic16` for 16-bit devices
CC_FAMILY = pic14

# The SDCC-specific (case sensitive) part number of the chip
# (use `make list-devices` if not known)
CC_DEVICE = 16f1454

# The MDB-specific part number of the chip, used for programming with MDB
# (should be the actual PIC part number, e.g. PIC16LF1454)
MDB_DEVICE = PIC16F1454


########## Build settings ##########

CFLAGS = --use-non-free --stack-size 8 --opt-code-speed --std-sdcc99 -DNO_BIT_DEFINES
LDFLAGS = -Wl-s16f1xxx_bootloader_4k.lkr
CC = sdcc
AS = gpasm
LD = gplink
DASM = gpdasm
SDCC_DIR = /usr/local/share/sdcc
MPLABX_DIR = /Applications/microchip/mplabx
MDB = $(MPLABX_DIR)/mplab_ide.app/Contents/Resources/mplab_ide/bin/mdb.sh



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
	@echo "Device $(MDB_DEVICE)" \
		"\nSet system.disableerrormsg true" \
		"\nHwtool PICkit3 -p" \
		"\nSet programoptions.eraseb4program true" \
		"\nProgram \"$(HEX)\"" \
		"\nQuit\n" > __prog.cmd
	@$(MDB) __prog.cmd; status=$$?; rm -f __prog.cmd MPLABXLog.*; exit $$status

# List supported device types
list-devices:
	@touch ___.c
	@-$(CC) -m$(CC_FAMILY) -phelp ___.c
	@rm ___.c

# Clean
clean:
	rm -f $(SRC:.c=.asm) $(SRC:.c=.lst) $(SRC:.c=.o) $(ASM:.S=.lst) $(ASM:.S=.o) $(HEX) $(OUT).cod $(OUT).lst

.PHONY: all flash clean list-devices

