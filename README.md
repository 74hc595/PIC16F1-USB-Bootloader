USB DFU Bootloader for PIC16F1454/5/9
=====================================

This is a **work in progress** and still **under development**.

Matt Sarnoff has demonstrated what is possible with his excellent hand-optimized assembly [512 word USB Bootloader](https://github.com/74hc595/PIC16F1-USB-Bootloader).

Its minimal size rightfully puts to shame existing C-code-based bootloaders such as [mine](https://github.com/majbthrd/pic16f1454-bootloader).

This project's goal is to take the concept of Matt's bootloader, but overhaul it to instead implement the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/).

## Example Usage

Read firmware image:
dfu-util -U backup.bin -t 64

## License

The contents of this repository is released under a [3-clause BSD license](LICENSE).

