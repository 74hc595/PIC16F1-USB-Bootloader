/*
    example vendor-defined custom HID using PIC16F1454 microcontroller

    based on M-Stack by Alan Ott, Signal 11 Software

    Copyright (C) 2015 Peter Lawrence

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in 
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#include "usb.h"
#include <xc.h>
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_hid.h"

/* 
since this is a downloaded app, configuration words (e.g. __CONFIG or #pragma config) are not relevant
*/

int main(void)
{
	uint8_t *TxDataBuffer;
	const uint8_t *RxDataBuffer;
	uint8_t counter = 0;

#ifdef USB_USE_INTERRUPTS
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
#endif

	usb_init();

	TxDataBuffer = usb_get_in_buffer(1);

	for (;;)
	{
#ifndef USB_USE_INTERRUPTS
		usb_service();
#endif

		/* if USB isn't configured, there is no point in proceeding further */
		if (!usb_is_configured())
			continue;

		/*
		we check these *BEFORE* calling usb_out_endpoint_has_data() as the documentation indicates this 
		must be followed usb_arm_out_endpoint() to enable reception of the next transaction
		*/
		if (usb_in_endpoint_halted(1) || usb_in_endpoint_busy(1))
			continue;

		/* if we pass this test, we are committed to make the usb_arm_out_endpoint() call */
		if (!usb_out_endpoint_has_data(1))
			continue;

		/* obtain a pointer to the receive buffer and the length of data contained within it */
		usb_get_out_buffer(1, &RxDataBuffer);

		/* pre-fill the response with an echo back of the command */
		TxDataBuffer[0] = RxDataBuffer[0];

		/*
		note to would-be developer: be VERY WARY about using a switch() statement
		the XC8 compiler generates bloated code, particularly if the case values are not all consecutive
		if, else if, else if statements are more efficient with XC8, albeit at the cost of readability
		*/
		switch (RxDataBuffer[0])
		{

		case 0x80:

			/*
			action in response to command would go here
			as an example, we are incrementing a counter than can be read by command 0x81
			*/
			counter++;

			break;

		case 0x81:

			/*
			response to query command would go here
			as an example, we are returning the counter value incremented by command 0x80
			*/
			TxDataBuffer[1] = counter;

			/* send a response back to the PC */
			usb_send_in_buffer(1, EP_1_IN_LEN);

			break;

		}

		/* re-arm the endpoint to receive the next EP1 OUT */
		usb_arm_out_endpoint(1);
	}
}

/* Callbacks. These function names are set in usb_config.h. */
int8_t app_unknown_setup_request_callback(const struct setup_packet *setup)
{
	/* To use the HID device class, have a handler for unknown setup
	 * requests and call process_hid_setup_request() (as shown here),
	 * which will check if the setup request is HID-related, and will
	 * call the HID application callbacks defined in usb_hid.h. For
	 * composite devices containing other device classes, make sure
	 * MULTI_CLASS_DEVICE is defined in usb_config.h and call all
	 * appropriate device class setup request functions here.
	 */
	return process_hid_setup_request(setup);
}

void interrupt isr()
{
#ifdef USB_USE_INTERRUPTS
    usb_service();
#endif
}
