USB DFU Bootloader for PIC16F1454/5/9
=====================================

This is a **work in progress** and needs further testing and refinement.  So far, it has only been tested with Linux.

Matt Sarnoff has demonstrated what is possible with his excellent hand-optimized assembly [512 word USB Bootloader](https://github.com/74hc595/PIC16F1-USB-Bootloader).

Its minimal size rightfully puts to shame existing C-code-based bootloaders such as [mine](https://github.com/majbthrd/pic16f1454-bootloader).

This project takes the concept of Matt's bootloader, but overhauled the code to instead implement the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/).

## Example Usage

dfu-util -U read.bin -t 64
dfu-util -D write.bin -t 64

## License

The contents of this repository is released under a [3-clause BSD license](LICENSE).

