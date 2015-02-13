/**
 * USB 512-Word CDC Bootloader Application Code
 * Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
 * v1.0, February 12, 2015
 * Released under a 3-clause BSD license: see the accompanying LICENSE file.
 *
 * PIC blinking LED example for SDCC.
 * Blinks an LED connected to pin RA5.
 * Tested with a PIC16F1454.
 *
 * Application code that is compatible with the bootloader has 5 requirements:
 *
 * - a main function named app_main(), NOT main().
 *   Defining main() will confuse the linker. As of version 3.4.0, SDCC forces
 *   code generation at address 0x0000; excluding main() prevents this.
 *
 * - an interrupt handler named app_interrupt(), even if the application does
 *   not use interrupts. (in that case, just use an empty function body.)
 *
 * - a configuration byte defined with the APP_CONFIG() macro.
 *   This macro is provided in usb_bootloader_config.h.
 *   This specifies whether the device is self-powered or bus-powered, and
 *   the maximum current usage when in bootloader mode.
 *   Ideally, a device should use as little current as possible if the PIC
 *   starts up in bootloader mode.
 *
 * - it must be compiled with the accompanying file crt_bootloader_512.S.
 *
 * - the linker script 16f145x_bootloader_512.lkr must be used.
 */

#include <pic14regs.h>
#include "usb_bootloader_config.h"

APP_CONFIG(USB_BUS_POWERED, 20 MILLIAMPS);

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

