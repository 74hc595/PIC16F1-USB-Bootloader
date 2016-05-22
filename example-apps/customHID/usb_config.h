/*
    example vendor-defined custom HID using PIC16F1454 microcontroller

    the intent of this example is to interoperate as-is with the HIDAPI example code
    http://www.signal11.us/oss/hidapi/

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

#ifndef USB_CONFIG_H__
#define USB_CONFIG_H__

/* Number of endpoint numbers besides endpoint zero. It's worth noting that
   and endpoint NUMBER does not completely describe an endpoint, but the
   along with the DIRECTION does (eg: EP 1 IN).  The #define below turns on
   BOTH IN and OUT endpoints for endpoint numbers (besides zero) up to the
   value specified.  For example, setting NUM_ENDPOINT_NUMBERS to 2 will
   activate endpoints EP 1 IN, EP 1 OUT, EP 2 IN, EP 2 OUT.  */
#define NUM_ENDPOINT_NUMBERS 1

/* Only 8, 16, 32 and 64 are supported for endpoint zero length. */
#define EP_0_LEN 8

#define EP_1_OUT_LEN 64
#define EP_1_IN_LEN  64

#define NUMBER_OF_CONFIGURATIONS 1

/* Ping-pong buffering mode. Valid values are:
	PPB_NONE         - Do not ping-pong any endpoints
	PPB_EPO_OUT_ONLY - Ping-pong only endpoint 0 OUT
	PPB_ALL          - Ping-pong all endpoints
	PPB_EPN_ONLY     - Ping-pong all endpoints except 0
*/

#define PPB_MODE PPB_NONE

/* Comment the following line to use polling USB operation. When using polling,
   You are responsible for calling usb_service() periodically from your
   application. */
//#define USB_USE_INTERRUPTS

/* Objects from usb_descriptors.c */
#define USB_DEVICE_DESCRIPTOR this_device_descriptor
#define USB_CONFIG_DESCRIPTOR_MAP usb_application_config_descs
#define USB_STRING_DESCRIPTOR_FUNC usb_application_get_string

/* Optional callbacks from usb.c. Leave them commented if you don't want to
   use them. For the prototypes and documentation for each one, see usb.h. */

//#define SET_CONFIGURATION_CALLBACK app_set_configuration_callback
//#define GET_DEVICE_STATUS_CALLBACK app_get_device_status_callback
//#define ENDPOINT_HALT_CALLBACK     app_endpoint_halt_callback
//#define SET_INTERFACE_CALLBACK     app_set_interface_callback
//#define GET_INTERFACE_CALLBACK     app_get_interface_callback
//#define OUT_TRANSACTION_CALLBACK   app_out_transaction_callback
//#define IN_TRANSACTION_COMPLETE_CALLBACK   app_in_transaction_complete_callback
#define UNKNOWN_SETUP_REQUEST_CALLBACK app_unknown_setup_request_callback
//#define UNKNOWN_GET_DESCRIPTOR_CALLBACK app_unknown_get_descriptor_callback
//#define START_OF_FRAME_CALLBACK    app_start_of_frame_callback
//#define USB_RESET_CALLBACK         app_usb_reset_callback

/* HID Configuration functions. See usb_hid.h for documentation. */
#define USB_HID_DESCRIPTOR_FUNC usb_application_get_hid_descriptor
#define USB_HID_REPORT_DESCRIPTOR_FUNC usb_application_get_hid_report_descriptor
//#define USB_HID_PHYSICAL_DESCRIPTOR_FUNC usb_application_get_hid_physical_descriptor

/* HID Callbacks. See usb_hid.h for documentation. */
//#define HID_GET_REPORT_CALLBACK app_get_report_callback
//#define HID_SET_REPORT_CALLBACK app_set_report_callback
//#define HID_GET_IDLE_CALLBACK app_get_idle_callback
//#define HID_SET_IDLE_CALLBACK app_set_idle_callback
//#define HID_GET_PROTOCOL_CALLBACK app_get_protocol_callback
//#define HID_SET_PROTOCOL_CALLBACK app_set_protocol_callback

#endif /* USB_CONFIG_H__ */
