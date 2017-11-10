ifeq ($(OS),Windows_NT)
	EXE_SUFFIX = .exe
endif

IHEX_ADD_CRC14_C = ihex-add-crc14.c

all: ihex-add-crc14$(EXE_SUFFIX)

ihex-add-crc14$(EXE_SUFFIX): Makefile $(IHEX_ADD_CRC14_C)
	gcc $(IHEX_ADD_CRC14_C) -o $@ $(CFLAGS)
	strip $@

clean:
	rm -f ihex-add-crc14$(EXE_SUFFIX)

