/**
 * PIC blinking LED example for SDCC
 * Blinks an LED connected to pin RA5.
 * Tested with a PIC16F1454.
 */

#include <pic14regs.h>

void app_interrupt(void)
{
  /* Clear interrupt flag and toggle LED state */
  PIR1bits.TMR1IF = 0;
  PORTA ^= (1 << 5);
}


int app_main(void)
{
  /* RA5 is an output */
  TRISAbits.TRISA5 = 0;

  /* Enable Timer1 with 1:8 prescaler and overflow interrupt */
  T1CON = _T1CKPS1|_T1CKPS0|_TMR1ON;
  PIE1bits.TMR1IE = 1;
  INTCONbits.PEIE = 1;
  INTCONbits.GIE = 1;

  while (1) {}
}

