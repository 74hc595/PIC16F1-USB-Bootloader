# Mac OS X Makefile for PIC programming using GPASM
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
OUT = bootloader

# Source files to assemble
ASM = bootloader.asm

# (use `make list-devices` if not known)
AS_DEVICE = p16f1454

# The MDB-specific part number of the chip, used for programming with MDB
# (should be the actual PIC part number, e.g. PIC16LF1454)
MDB_DEVICE = PIC16F1454



########## Build settings ##########

AS = gpasm
DASM = gpdasm
MPLABX_DIR = /Applications/microchip/mplabx
MDB = $(MPLABX_DIR)/mplab_ide.app/Contents/Resources/mplab_ide/bin/mdb.sh



########## Make rules ##########

HEX = $(OUT).hex

# Link
$(HEX): $(ASM)
	$(AS) -p $(AS_DEVICE) -o $(HEX) $(ASM)

# Disassemble
dis: $(HEX)
	$(DASM) -p p$(AS_DEVICE) $(HEX)

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
	@$(AS) -l

# Clean
clean:
	rm -f $(ASM:.asm=.lst) $(HEX) $(OUT).cod $(OUT).lst

.PHONY: all flash clean list-devices

