/*
    example HID keyboard using PIC16F1454 microcontroller

    originally written for Microchip USB Framework (aka MLA), but
    now based on M-Stack by Alan Ott, Signal 11 Software

    Copyright (C) 2014,2015 Peter Lawrence

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

/*
this determines how long one has to press the button for the transmit to be triggered
it also determines how long the button must be released in order to re-arm
*/
#define MIN_TRIGGER_MS 750

/*
this determines which bit to monitor for the bootloader entry
*/
#define BOOTLOADER_ENTRY_KEYLOCK_MASK 0x04 /* scroll-lock key */

/* flag set upon USB SOF (Start Of Frame) to track approximate time */
static uint8_t ms_tick = 0;

/*
arbitrary key strokes to play back
you MUST consult Section 10 "Keyboard/Keypad Page (0x07)" of the USB "HID Usage Tables" specification
in order to look up the "Usage ID" of each key press
*/
const uint8_t key_table[] =
{
	0x4, // 'a'
	0x5, // 'b'
	0x6, // 'c'
	0x7, // 'd'
};

static uint16_t keylock_tick_count = 0;
static uint8_t last_keylock_state = 0;

int main(void)
{
	uint8_t table_index = 0;
	uint16_t count = MIN_TRIGGER_MS;
	enum
	{
		COOLDOWN,
		ARMED,
		TRANSMITTING,
	} state = COOLDOWN;
	uint8_t *hid_report_in;

	/* enable pull-up on RA3 (for pushbutton detection) */

	OPTION_REGbits.nWPUEN = 0;

	WPUA = (1 << 3);

	usb_init();

	hid_report_in = usb_get_in_buffer(1);

	for (;;)
	{
		usb_service();

		/* if USB isn't configured, there is no point in proceeding further */
		if (!usb_is_configured())
			continue;

		if (ms_tick)
		{
			if ( (COOLDOWN == state) || (ARMED == state) )
			{
				if (PORTAbits.RA3)
				{
					if (count)
						count--;
					else
						state = ARMED;
				}
				else
				{
					if (count < MIN_TRIGGER_MS)
						count++;
					else if (ARMED == state)
					{
						state = TRANSMITTING;
						table_index = 0;
					}
				}
			}

			ms_tick = 0;
		}

		/* proceed further only if it is possible to send more data */
		if (usb_in_endpoint_halted(1) || usb_in_endpoint_busy(1))
			continue;

		/* build HID report */
		if (TRANSMITTING == state)
		{
			hid_report_in[0] = 0;
			hid_report_in[1] = 0;
			if (sizeof(key_table) == table_index)
			{
				state = COOLDOWN;
				hid_report_in[2] = 0x00;
			}
			else
			{
				hid_report_in[2] = key_table[table_index];
				table_index++;
			}
			hid_report_in[3] = 0;
			hid_report_in[4] = 0;
			hid_report_in[5] = 0;
			hid_report_in[6] = 0;
			hid_report_in[7] = 0;

			/* transmit HID report */
			usb_send_in_buffer(1, EP_1_IN_LEN);
		}
	}
}

/* Callbacks. These function names are set in usb_config.h. */
void app_set_configuration_callback(uint8_t configuration)
{

}

uint16_t app_get_device_status_callback()
{
	return 0x0000;
}

void app_endpoint_halt_callback(uint8_t endpoint, bool halted)
{

}

int8_t app_set_interface_callback(uint8_t interface, uint8_t alt_setting)
{
	return 0;
}

int8_t app_get_interface_callback(uint8_t interface)
{
	return 0;
}

void app_out_transaction_callback(uint8_t endpoint)
{

}

void app_in_transaction_complete_callback(uint8_t endpoint)
{

}

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

int16_t app_unknown_get_descriptor_callback(const struct setup_packet *pkt, const void **descriptor)
{
	return -1;
}

void app_start_of_frame_callback(void)
{
       	ms_tick = 1;

	/* if chosen KEYLOCK is now on, increment keylock_tick_count until it reaches 65535 */
	if (last_keylock_state & BOOTLOADER_ENTRY_KEYLOCK_MASK)
		if (0xFFFF != keylock_tick_count)
			keylock_tick_count++;
}

void app_usb_reset_callback(void)
{

}

/* HID Callbacks. See usb_hid.h for documentation. */

static uint8_t set_report_buf[1];

static void set_report_callback(bool transfer_ok, void *context)
{

	/*
	we monitor KEYLOCK activity to provide a means to force the device back to the bootloader
	reboot to bootloader *if* KEYLOCK turns on for a second (800 to 1200 milliseconds) and then goes back off
	*/

	if (set_report_buf[0] != last_keylock_state)
	{
		if ( (keylock_tick_count > 800) && (keylock_tick_count < 1200) )
		{
			/* enable watchdog; the code doesn't clear the watchdog, so it will eventually reset */
			WDTCONbits.SWDTEN = 1;
		}

		keylock_tick_count = 0;

		last_keylock_state = set_report_buf[0];
	}
}

int8_t app_set_report_callback(uint8_t interface, uint8_t report_type,
                               uint8_t report_id)
{
	usb_start_receive_ep0_data_stage(set_report_buf, sizeof(set_report_buf), &set_report_callback, NULL);

	return 0;
}

uint8_t app_get_idle_callback(uint8_t interface, uint8_t report_id)
{
	return 0;
}

int8_t app_set_idle_callback(uint8_t interface, uint8_t report_id,
                             uint8_t idle_rate)
{
	return -1;
}

int8_t app_get_protocol_callback(uint8_t interface)
{
	return 1;
}

int8_t app_set_protocol_callback(uint8_t interface, uint8_t report_id)
{
	return -1;
}


void interrupt isr()
{
}
