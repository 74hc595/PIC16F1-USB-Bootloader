  extern __sdcc_gsinit_startup
  extern _app_main
  extern _app_interrupt
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

APP_STARTUP code 0x1000
  pagesel __sdcc_gsinit_startup
  goto    __sdcc_gsinit_startup

APP_INTERRUPT code 0x1002
  pagesel _app_interrupt
  call    _app_interrupt
  retfie

_main
  pagesel _app_main
  goto    _app_main

  end
