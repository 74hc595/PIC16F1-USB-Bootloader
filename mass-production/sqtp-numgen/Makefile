ifeq ($(OS),Windows_NT)
	EXE_SUFFIX = .exe
endif

SQTP_NUMGEN_C = sqtp-numgen.c

all: sqtp-numgen$(EXE_SUFFIX)

sqtp-numgen$(EXE_SUFFIX): Makefile $(SQTP_NUMGEN_C)
	gcc $(SQTP_NUMGEN_C) -o $@ $(CFLAGS)
	strip $@

clean:
	rm -f sqtp-numgen$(EXE_SUFFIX)

