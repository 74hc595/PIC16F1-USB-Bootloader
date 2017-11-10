sqtp-numgen
===========

Serial Quick Turn Programming (SQTP) is a protocol defined by Microchip, and allows a firmware images to be selectively patched prior to being written to the target.  This allows the option of each device being programmed slightly differently.

SQTP is supported during the programming process by MPLAB IPE and may also be available as a value-added service from Microchip for large volume orders.

'USB 512-Word DFU Bootloader for PIC16(L)F1454/5/9' was specifically designed to have enough memory words available to support an eight-character USB serial number that could be programmed by SQTP.  This serial number can be leveraged by the bootloader, but it has even greater importance as an option for the main application.

The bootloader memory is write-protected, and this allows a permanence of a unique serial number that could not be achieved within the main application memory.

USB serial numbers must be treated carefully.  If a USB device does not have a serial number (no serial string provided in the descriptor), the host PC uses the physical USB port as a means of distinguishing the device from others of the same VID:PID.  If, however, a USB device has a serial number, the host PC *assumes* that all devices with the same VID:PID and serial number are unique and is wholly unprepared if multiple devices share these same attributes.

For this reason, the bootloader.asm source code is written with "HIDE_SERIAL_NUMBER" set to 1 to help protect the novice user from 
accidentally programming two or more devices with the same image.  However, if you plan to ensure programming unique serial numbers, you could set this to 0 and recompile the code.

The sqtp-numgen tool in this directory generates a ".num" SQTP file.  The only way to absolutely guarantee a unique serial number would be a centralized authority issuing serial numbers on an as-needed basis.  Since this is not yet available, sqtp-numgen generates a random number and uses this as the starting number for a sequences of serial numbers.

What is great about the ".num" SQTP file is that MPLAB IPE automatically removes an entry from the file upon each programming operation.  This helps ensure that each entry in the file is only used once.

## Tool Usage

```
sqtp-numgen > mySNs.num
```

## User Application Usage Of Serial Number Provided by Bootloader

Assuming the user application uses [M-Stack](http://www.signal11.us/oss/m-stack/), there will be a "#define USB_STRING_DESCRIPTOR_FUNC" in usb_config.h.  Let's assume this is defined to point to the "usb_application_get_string()" function, and your code currently returns a serial string stored in user flash like so:


```
	*ptr = &serial_string;
	return sizeof(serial_string);
```

This could be replaced with a direct pointer to the serial number provided by 'USB 512-Word DFU Bootloader for PIC16(L)F1454/5/9':

```
	*ptr = (void *)0x81EE;
	return 18; /* one byte length + one byte for string type + 8x two bytes for eight character UNICODE string */
```

Tip: Note that the XC8 compiler does not reliably optimize out unused functions and structs.  So, in the above example, if you are no longer using "serial_string" and want the most flash space for your application, it behooves you to remove "serial_string" from the code.

