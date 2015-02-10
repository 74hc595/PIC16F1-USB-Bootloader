/**
 * PIC blinking LED example for SDCC
 * Blinks an LED connected to pin RA5.
 * Tested with a PIC16F1454.
 */

#include <pic14regs.h>
#include <stdint.h>

void delay(uint32_t d)
{
  uint32_t i;
  for (i = 0; i < d; i++) {
    __asm
      nop
    __endasm;
  }
}


int app_main()
{
  TRISAbits.TRISA5 = 0;
  while (1) {
    PORTA ^= (1 << 5);
    delay(60000);
  }
}

