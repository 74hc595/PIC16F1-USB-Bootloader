USB DFU Bootloader for PIC16F1454/5/9
=====================================

This bootloader has been tested with Linux, Windows, and MacOS.

Thank you to [pidcodes](http://pid.codes/) for assigning a USB VID:PID.

Matt Sarnoff demonstrated what is possible with his excellent hand-optimized assembly [512 word USB Bootloader](https://github.com/74hc595/PIC16F1-USB-Bootloader).

Its minimal size rightfully puts to shame existing C-code-based bootloaders such as [mine](https://github.com/majbthrd/pic16f1454-bootloader).

This project takes the concept of Matt's bootloader, but overhauled the code to instead implement the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/).

This bootloader also has the added advantage of CRC-14 protection of the user application. 

## Usage

For MPLAB/XC8 compilation, the following options are needed:

```
--codeoffset=0x200
--rom=default,-0-1FF,-1F7F-1F7F
```

The utility provided in the ./tools/ subdirectory converts a .hex file into a CRC-14 protected binary image:

```
454hex2dfu foo.hex foo.dfu
```

Downloading can be accomplished with the existing [dfu-util](http://dfu-util.sourceforge.net/) utilities:

```
dfu-util -D write.dfu
```

## Limitations

* A PIC16F145x chip of silicon revision A5 or later is required due to an issue with writing to program memory on revision A2 parts. The value at address 0x8006 in configuration space should be 0x1005 or later. See the [silicon errata document](http://ww1.microchip.com/downloads/en/DeviceDoc/80000546F.pdf) for more information.

* The configuration words are hard-coded in the bootloader (see the __config lines in bootloader.asm); the downloaded app inherits these settings and cannot invoke different values.

## License

The contents of this repository are released under a [3-clause BSD license](http://opensource.org/licenses/BSD-3-Clause).

