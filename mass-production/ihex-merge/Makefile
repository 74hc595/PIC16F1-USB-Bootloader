ifeq ($(OS),Windows_NT)
	EXE_SUFFIX = .exe
endif

IHEX_MERGE_C = ihex-merge.c

all: ihex-merge$(EXE_SUFFIX)

ihex-merge$(EXE_SUFFIX): Makefile $(IHEX_MERGE_C)
	gcc $(IHEX_MERGE_C) -o $@ $(CFLAGS)
	strip $@

clean:
	rm -f ihex-merge$(EXE_SUFFIX)

