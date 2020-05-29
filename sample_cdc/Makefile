# Makefile for SDCC + PIC16F1xxx with USB CDC bootloader
# Matt Sarnoff (msarnoff.org)
# February 10, 2015
#
# Run `make` to build the project as a .hex file.
# Run `make flash` to program the device.
#

########## Project-specific definitions ##########

# Project name
OUT = sample_cdc

# Assembly files to assemble
ASM = sample_cdc.asm

# The SDCC-specific (case sensitive) part number of the chip
# (use `make list-devices` if not known)
AS_DEVICE = 16f1454


########## Build settings ##########

PROG = ../usb16f1prog	# change this once you've installed the script
AS = gpasm
DASM = gpdasm
PORT = -p /dev/ttyACM0

########## Make rules ##########

HEX = $(OUT).hex


# Link
$(HEX): $(ASM) 
	$(AS) -I .. -p$(AS_DEVICE) -o $@ $<


# Disassemble
dis: $(HEX)
	$(DASM) -p p$(AS_DEVICE) $(HEX) | less

# Flash
flash: $(HEX)
	sudo $(PROG) $(PORT) $(HEX)

# Clean
clean:
	rm -f $(ASM:.asm=.lst) $(HEX) $(OUT).cod $(OUT).lst

.PHONY: all flash clean dis

