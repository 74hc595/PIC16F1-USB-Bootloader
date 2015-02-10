/**
 * PIC blinking LED example for SDCC
 * Blinks an LED connected to pin RA5.
 * Tested with a PIC16F1454.
 */

#include <pic14regs.h>
#include <stdint.h>

void delay(uint16_t d)
{
  uint16_t i;
  for (i = 0; i < d; i++) {
    __asm nop __endasm;
  }
}


int app_main()
{
  // 48MHz clock from INTOSC (16MHz + 3x PLL)
  OSCCON = _SPLLEN|_SPLLMULT|_IRCF3|_IRCF2|_IRCF1|_IRCF0;

  TRISAbits.TRISA5 = 0;
  while (1) {
    PORTAbits.RA5 = 1;
    delay(30000);
    PORTAbits.RA5 = 0;
    delay(30000);
  }
}

