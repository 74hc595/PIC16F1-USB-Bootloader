CFLAGS = -fstack-protector -fstack-protector-all
# CFLAGS = -O3

454HEX2BIN_C = 454hex2bin.c
454HEX2BIN_H = 

all: 454hex2bin

454hex2bin: Makefile $(454HEX2BIN_C) $(454HEX2BIN_H)
	gcc $(454HEX2BIN_C) -o $@ $(CFLAGS)

clean:
	rm -f 454hex2bin 454hex2bin.exe
