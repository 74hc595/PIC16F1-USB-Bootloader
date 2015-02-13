/**
 * USB 512-Word CDC Bootloader Application Code
 * Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
 * v1.0, February 12, 2015
 * Released under a 3-clause BSD license: see the accompanying LICENSE file.
 *
 * Bootloader configuration convenience macro for applications.
 * The APP_CONFIG() macro must be invoked in global scope in exactly one
 * source file.
 */
#ifndef BOOTLOADER_USB_CONFIG_H
#define BOOTLOADER_USB_CONFIG_H

/**
 * Device power configuration
 * bit 0      clear if device is bus-powered, set if device is self-powered
 * bits 7-1   maximum power consumption in 4 mA units
 *
 * The APP_CONFIG() macro takes care of formatting this bitfield properly.
 * selfpwr    either USB_BUS_POWERED or USB_SELF_POWERED
 * milliamps  maximum power consumption in milliamps
 *            max value is 508 mA; the value will be rounded down to a
 *            multiple of 4 mA
 */
#define APP_CONFIG(selfpwr, milliamps)  __code unsigned char app_config = ((selfpwr)!=0)|(((milliamps)/4)<<1)

#define USB_BUS_POWERED   0
#define USB_SELF_POWERED  1
#define MILLIAMPS /* for readability */

#endif
