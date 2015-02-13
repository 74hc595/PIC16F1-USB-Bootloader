; USB 512-Word CDC Bootloader Application Code
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; Startup code for C programs.

  extern __sdcc_gsinit_startup
  extern _app_main
  extern _app_interrupt
  extern _app_config
  global _main

  global PSAVE
  global SSAVE
  global WSAVE
  global STK04
  global STK03
  global STK02
  global STK01
  global STK00
  
sharebank udata_ovr 0x0070
PSAVE res 1
SSAVE res 1
WSAVE res 1
STK04 res 1
STK03 res 1
STK02 res 1
STK01 res 1
STK00 res 1

APP_STARTUP code 0x200
  pagesel __sdcc_gsinit_startup
  goto    __sdcc_gsinit_startup

APP_CONFIG code 0x202
  pagesel _app_config
  goto    _app_config ; must be a 'retlw' instruction, i.e. a constant

APP_INTERRUPT code 0x204
  pagesel _app_interrupt
  call    _app_interrupt
  retfie

_main
  pagesel _app_main
  goto    _app_main

  end
