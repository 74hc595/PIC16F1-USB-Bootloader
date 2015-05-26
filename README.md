USB DFU Bootloader for PIC16F1454/5/9
=====================================

This bootloader needs a USB VID:PID.  It has been tested with Linux and Windows; it should work with MacOS as well.

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
454hex2bin foo.hex foo.bin
```

Downloading can be accomplished with the existing [dfu-util](http://dfu-util.sourceforge.net/) utilities:

```
dfu-util -U read.bin -t 64
```
```
dfu-util -D write.bin -t 64
```

## License

The contents of this repository is released under a [3-clause BSD license](https://raw.githubusercontent.com/majbthrd/PIC16F1-USB-DFU-Bootloader/master/firmware/LICENSE).

