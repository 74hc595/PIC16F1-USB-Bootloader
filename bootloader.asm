; vim:noet:sw=8:ts=8:ai:syn=pic
;
; this code is in development and not yet functional
; at the moment, the code only READS, not WRITES, program memory
;
; USB 512-Word DFU Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Peter Lawrence
; derived from
; USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)

; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; Bootloader is entered if:
; - the MCLR/RA3 pin is grounded at power-up or reset,
; (The internal pull-up is used; no external resistor is necessary.)
; - there is no application programmed,
; - the watchdog timed out
;
; To be detected as a valid application, the lower 8 bytes of the first
; instruction word must NOT be 0xFF.
;
; At application start, the device is configured with a 48MHz CPU clock,
; using the internal oscillator and 3x PLL. If a different oscillator
; configuration is required, it must be set by the application.
;
; A serial number between 0 and 65535 should be specified during the build
; by using the gpasm -D argument to set the SERIAL_NUMBER symbol, e.g.
;   gpasm -D SERIAL_NUMBER=12345
; If not specified, it will default to zero.
; A host may not behave correctly if multiple PICs with the same serial number
; are connected simultaneously.
;
; Code notes:
; - Labels that do not begin with an underscore can be called as functions.
;   Labels that begin with an underscore are not safe to call, they should only
;   be reached via goto.
;
; - FSR0L, FSR0H, FSR1L, and FSR1H are used as temporary registers in several
;   places, e.g. as loop counters. They're accessible regardless of the current
;   bank, and automatically saved/restored on interrupt. Neato!
;
; - As much stuff as possible is packed into bank 0 of USB RAM. This includes the
;   buffer descriptors, bootloader state, and endpoint 0 OUT and IN buffers
;
; - Using DFU has the substantive advantage of needing only EP0.  A backwards-
;   compatible extension to the protocol is to use wBlockNum in DFU_DNLOAD and 
;   DFU_UPLOAD as the PIC flash row index (and optional PMCON1 CFGS select)

	radix dec
	list n=0,st=off
	include "p16f1454.inc"
	nolist
	include "macros.inc"
	include "bdt.inc"
	include "usb.inc"
	include "protocol_constants.inc"
	list
	errorlevel -302

;;; Configuration
WRT_CONFIG		equ	_WRT_BOOT

	__config _CONFIG1, _FOSC_INTOSC & _WDTE_SWDTEN & _PWRTE_ON & _MCLRE_OFF & _CP_ON & _BOREN_ON & _IESO_OFF & _FCMEN_OFF
	__config _CONFIG2, WRT_CONFIG & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_3x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LVP_OFF



;;; Constants and varaiable addresses
SERIAL_NUMBER_DIGIT_CNT	equ	4
	ifndef SERIAL_NUMBER
	variable SERIAL_NUMBER=0	; Why doesnt 'equ' work here? Go figure
	endif

; these values are temporary and for development testing only
; If your organization has its own vendor ID/product ID, substitute it here.
; the VID:PID for the DFU bootloader must be distinct from the product itself, as Windows insists on it
USB_VENDOR_ID		equ	0x1234
USB_PRODUCT_ID		equ	0x0001

DEVICE_DESC_LEN		equ	18	; device descriptor length
CONFIG_DESC_TOTAL_LEN	equ	18	; total length of configuration descriptor and sub-descriptors
EXTRAS_LEN		equ	6	; total length of extras
SERIAL_NUM_DESC_LEN	equ	2+(SERIAL_NUMBER_DIGIT_CNT*2)
ALL_DESCS_TOTAL_LEN	equ	DEVICE_DESC_LEN+CONFIG_DESC_TOTAL_LEN+EXTRAS_LEN+SERIAL_NUM_DESC_LEN

EP0_BUF_SIZE 		equ	64	; endpoint 0 buffer size

; We're only using the USB minimum of 2 endpoints (EP0OUT and EP0IN); use the remaining BDT area for buffers.

; Use the 4 bytes normally occupied by the EP1 OUT (immediately after EP0IN) buffer descriptor for variables.
USB_STATE		equ	BANKED_EP1OUT+0
EP0_DATA_IN_PTR		equ	BANKED_EP1OUT+1	; pointer to descriptor to be sent (low byte only)
EP0_DATA_IN_COUNT	equ	BANKED_EP1OUT+2	; remaining bytes to be sent
;			equ	BANKED_EP1OUT+3	; spare

; USB data buffers go immediately after memory re-purposed for variables
EP0OUT_BUF		equ	EP1IN
BANKED_EP0OUT_BUF	equ	BANKED_EP1IN
EP0IN_BUF		equ	EP0OUT_BUF+EP0_BUF_SIZE
BANKED_EP0IN_BUF	equ	BANKED_EP0OUT_BUF+EP0_BUF_SIZE
EP_DATA_BUF_END		equ	EP0IN_BUF+EP0_BUF_SIZE

; High byte of all endpoint buffers.
EPBUF_ADRH		equ	(EP0OUT_BUF>>8)
	if ((EP0IN_BUF>>8) != (EP0OUT_BUF>>8))
	error "Endpoint buffers must be in the same 256-word region"
	endif

; Total length of all RAM (variables, buffers, BDT entries) used by the bootloader,
USED_RAM_LEN		equ	EP_DATA_BUF_END-BDT_START

BOOTLOADER_SIZE		equ	0x200

; Application code locations
APP_ENTRY_POINT		equ	BOOTLOADER_SIZE
APP_INTERRUPT		equ	BOOTLOADER_SIZE+4

; USB_STATE bit flags
IS_CONTROL_WRITE	equ	0	; current endpoint 0 transaction is a control write
ADDRESS_PENDING		equ	1	; need to set address in next IN transaction
DEVICE_CONFIGURED	equ	2	; the device is configured
IS_DFU_TRANSFER		equ	3	; when active, ep0_read_in diverts to an alternate routine

;;; Vectors
	org	0x0000
RESET_VECT
; Enable weak pull-ups
	banksel	OPTION_REG
	bcf	OPTION_REG,NOT_WPUEN
	banksel	OSCCON
	goto	bootloader_start	; to be continued further down in the file

	org	0x0004
INTERRUPT_VECT
; check the high byte of the return address (at the top of the stack)
	banksel	TOSH
; for 512-word mode: if TOSH == 0, we're in the bootloader
; if TOSH != 0, jump to the application interrupt handler
	tstf	TOSH
	bnz	APP_INTERRUPT

; executing from the bootloader? it's a USB interrupt
_bootloader_interrupt
	banksel	UIR
; reset?
	btfss	UIR,URSTIF
	goto	_utrans		; not a reset? just start servicing transactions
	call	usb_init	; if so, reset the USB interface (clears interrupts)
	banksel	PIE2
	bsf	PIE2,USBIE	; reenable USB interrupts
	banksel	UIR
	bcf	UIR,URSTIF	; clear the flag
; service transactions
_utrans	banksel	UIR
	btfss	UIR,TRNIF
	goto	_usdone
	movfw	USTAT		; stash the status in a temp register
	movwf	FSR1H
	bcf	UIR,TRNIF	; clear flag and advance USTAT fifo
	banksel	BANKED_EP0OUT_STAT
	andlw	b'01111000'	; check endpoint number
	bnz	_usdone		; bail if not endpoint 0
	call	usb_service_ep0	; handle the control message
	goto	_utrans
; clear USB interrupt
_usdone	banksel	PIR2
	bcf	PIR2,USBIF
	retfie



;;; Idle loop. In bootloader mode, the MCU just spins here, and all USB
;;; communication is interrupt-driven.
;;; This snippet is deliberately located within the first 256 words of program
;;; memory, so we can easily check in the interrupt handler if the interrupt
;;; occurred while executing application code or bootloader code.
;;; (TOSH will be 0x00 when executing bootloader code, i.e. this snippet)
bootloader_main_loop
	bsf	INTCON,GIE	; enable interrupts
_loop
	goto	_loop



;;; Handles a control transfer on endpoint 0.
;;; arguments:	expects USTAT value in FSR1H
;;;		BSR=0
;;; returns:	none
;;; clobbers:	W, FSR1H
usb_service_ep0
	btfsc	FSR1H,DIR	; is it an IN transfer or an OUT/SETUP?
	goto	_usb_ctrl_in
; it's an OUT or SETUP transfer
	movfw	BANKED_EP0OUT_STAT
	andlw	b'00111100'	; isolate PID bits
	sublw	PID_SETUP	; is it a SETUP packet?
	bnz	arm_ep0_out	; if not, it's a regular OUT, just rearm the buffer
	; it's a SETUP packet--fall through

; Handles a SETUP control transfer on endpoint 0.
; BSR=0
_usb_ctrl_setup
	bcf	USB_STATE,IS_CONTROL_WRITE
	bcf	USB_STATE,IS_DFU_TRANSFER
; set IS_CONTROL_WRITE bit in USB_STATE according to MSB in bmRequestType
	btfss	BANKED_EP0OUT_BUF+bmRequestType,7	; is this host->device?
	bsf	USB_STATE,IS_CONTROL_WRITE		; if so, this is a control write
; check if bmRequestType is DFU
	movlw	0x21
	subwf	BANKED_EP0OUT_BUF+bmRequestType,w
	andlw	b'01111111'	; mask out MSB
	bz	_its_a_dfu_transfer
; check request number: is it Get Descriptor?
	movlw	GET_DESCRIPTOR
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_get_descriptor
; is it Set Address?
	movlw	SET_ADDRESS
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_set_address
; is it Set_Configuration?
	movlw	SET_CONFIG
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_set_configuration
; is it Get Configuration?
	movlw	GET_CONFIG
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_get_configuration
; unhandled request? fall through to _usb_ctrl_invalid

; Finishes a rejected SETUP transaction: the endpoints are stalled
_usb_ctrl_invalid
	banksel	UCON
	bcf	UCON,PKTDIS	; reenable packet processing
	banksel	BANKED_EP0IN_STAT
	movlw	_DAT0|_DTSEN|_BSTALL
	call	arm_ep0_in_with_flags
arm_ep0_out
	movlw	_DAT0|_DTSEN|_BSTALL
arm_ep0_out_with_flags			; W specifies STAT flags
	movwf	BANKED_EP0OUT_STAT
	movlw	EP0_BUF_SIZE		; reset the buffer count
	movwf	BANKED_EP0OUT_CNT
	bsf	BANKED_EP0OUT_STAT,UOWN	; arm the OUT endpoint
	return

_its_a_dfu_transfer
	movfw	BANKED_EP0OUT_BUF+bRequest
	bz	_dfu_detach	; enum=0
	decw
	bz	_dfu_dnload	; enum=1
	decw
	bz	_dfu_upload	; enum=2
	decw
	bz	_dfu_getstatus	; enum=3
	decw
	bz	_dfu_clrstatus	; enum=4
	decw
	bz	_dfu_getstate	; enum=5
	decw
	bz	_dfu_abort	; enum=6

_dfu_dnload ; temporary: have been unable so far to transfer the rest of the EP0OUT data
	goto	_usb_ctrl_invalid

_dfu_getstatus
	movlw	low DFU_STATUS_RESPONSE
	movwf	EP0_DATA_IN_PTR
	movlw	6
	goto	_set_data_in_count_from_w
_dfu_getstate
	movlw	low DFU_STATE_RESPONSE
	movwf	EP0_DATA_IN_PTR
	movlw	1
	goto	_set_data_in_count_from_w
_dfu_upload
	tstf	BANKED_EP0OUT_BUF+wValueH
	bnz	_dfu_zero			; if wBlockNum is over 255, this is beyond the memory range of the device
	bsf	USB_STATE,IS_DFU_TRANSFER	; set flag to divert the transfer
_dfu_upload_already_happening
	movlw	EP0_BUF_SIZE
	goto	_set_data_in_count_from_w
_dfu_detach
_dfu_clrstatus
_dfu_abort
_dfu_zero
	movlw	0
	goto	_set_data_in_count_from_w

; Finishes a successful SETUP transaction.
_usb_ctrl_complete
	banksel	UCON
	bcf	UCON,PKTDIS		; reenable packet processing
	banksel	USB_STATE
	btfsc	USB_STATE,IS_CONTROL_WRITE
	goto	_cwrite
; this is a control read; prepare the IN endpoint for the data stage
; and the OUT endpoint for the status stage
_cread	call	ep0_read_in		; read data into IN buffer
	movlw	_DAT1|_DTSEN		; OUT buffer will be ready for status stage
; value in W is used to specify the EP0 OUT flags
_armbfs	call	arm_ep0_out_with_flags
	movlw	_DAT1|_DTSEN		; arm IN buffer
arm_ep0_in_with_flags			; W specifies STAT flags
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
	return
; this is a control write: prepare the IN endpoint for the status stage
; and the OUT endpoint for the next SETUP transaction
_cwrite	bcf	BANKED_EP0IN_STAT,UOWN	; ensure we have ownership of the buffer
	clrf	BANKED_EP0IN_CNT	; we'll be sending a zero-length packet
	movlw	_DAT0|_DTSEN|_BSTALL	; make OUT buffer ready for next SETUP packet
	goto	_armbfs			; arm OUT and IN buffers



; Handles a Get Descriptor request.
; BSR=0
_usb_get_descriptor
; check descriptor type
	movlw	DESC_CONFIG
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_config_descriptor
	movlw	DESC_STRING
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_string_descriptor
	movlw	DESC_DEVICE
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bnz	_usb_ctrl_invalid
_device_descriptor
	movlw	low DEVICE_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	DEVICE_DESC_LEN
	goto	_set_data_in_count_from_w
_config_descriptor
	movlw	low CONFIGURATION_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	CONFIG_DESC_TOTAL_LEN	; length includes all subordinate descriptors
	goto	_set_data_in_count_from_w
_string_descriptor
; only one string descriptor (serial number) is supported,
; so don't bother checking wValueL
	movlw	low SERIAL_NUMBER_STRING_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	SERIAL_NUM_DESC_LEN
_set_data_in_count_from_w
	movwf	EP0_DATA_IN_COUNT
; the count needs to be set to the minimum of the descriptor's length (in W)
; and the requested length
	subwf	BANKED_EP0OUT_BUF+wLengthL,w	; just ignore high byte...
	bc	_usb_ctrl_complete		; if W <= f, no need to adjust
	movfw	BANKED_EP0OUT_BUF+wLengthL
	movwf	EP0_DATA_IN_COUNT
	goto	_usb_ctrl_complete

; Handles a Set Address request.
; The address is actually set in the IN status stage.
_usb_set_address
	bsf	USB_STATE,ADDRESS_PENDING	; address will be assigned in the status stage
	goto	_usb_ctrl_complete

; Handles a Set Configuration request.
; For now just accept any nonzero configuration.
; BSR=0
_usb_set_configuration
	bcf	USB_STATE,DEVICE_CONFIGURED	; temporarily clear flag
	tstf	BANKED_EP0OUT_BUF+wValueL	; anything other than 0 is valid
	skpz
	bsf	USB_STATE,DEVICE_CONFIGURED
	goto	_usb_ctrl_complete

; Handles a Get Configuration request.
; BSR=0
_usb_get_configuration
; load a pointer to either a 0 or a 1 in ROM
; the 0 and 1 have been chosen so that they are adjacent
	movlw	low OPPORTUNISTIC_0_CONSTANT
	btfsc	USB_STATE,DEVICE_CONFIGURED
	incw
	movwf	EP0_DATA_IN_PTR
	movlw	1
	movwf	EP0_DATA_IN_COUNT
	goto	_usb_ctrl_complete

; Handles an IN control transfer on endpoint 0.
; BSR=0
_usb_ctrl_in
	btfsc	USB_STATE,IS_CONTROL_WRITE	; is this a control read or write?
	goto	_check_for_pending_address
; fetch more data and re-arm the IN endpoint
	call	ep0_read_in
	movlw	_DTSEN
	btfss	BANKED_EP0IN_STAT,DTS	; toggle DTS
	bsf	WREG,DTS
	goto	arm_ep0_in_with_flags	; arm the IN buffer
	
; if this is the status stage of a Set Address request, assign the address here.
; The OUT buffer has already been armed for the next SETUP.
_check_for_pending_address
	btfss	USB_STATE,ADDRESS_PENDING
	return
; read the address out of the setup packed in the OUT buffer
	bcf	USB_STATE,ADDRESS_PENDING
	movfw	BANKED_EP0OUT_BUF+wValueL
	banksel	UADDR
	movwf	UADDR
	return



;;; Reads descriptor data from EP0_DATA_IN_PTR, copies it to the EP0 IN buffer,
;;; and decrements EP0_DATA_IN_COUNT.
;;; arguments:	BSR=0
;;; returns:	EP0_DATA_IN_PTRL advanced
;;;		EP0_DATA_IN_COUNT decremented
;;; clobbers:	W, FSR0, FSR1
ep0_read_in
	bcf	BANKED_EP0IN_STAT,UOWN	; make sure we have ownership of the buffer
	clrf	BANKED_EP0IN_CNT	; initialize transmit size to 0
	btfsc	USB_STATE,IS_DFU_TRANSFER
	goto	ep0_read_dfu_in
	tstf	EP0_DATA_IN_COUNT	; do nothing if there are 0 bytes to send
	retz
	movfw	EP0_DATA_IN_PTR		; set up source pointer
	movwf	FSR0L
	movlw	DESCRIPTOR_ADRH|0x80
	movwf	FSR0H
	ldfsr1d	EP0IN_BUF		; set up destination pointer
	clrw
; byte copy loop
_bcopy	sublw	EP0_BUF_SIZE		; have we filled the buffer?
	bz	_bcdone
	moviw	FSR0++
	movwi	FSR1++
	incf	BANKED_EP0IN_CNT,f	; increase number of bytes copied
	movfw	BANKED_EP0IN_CNT	; save to test on the next iteration
	decfsz	EP0_DATA_IN_COUNT,f	; decrement number of bytes remaining
	goto	_bcopy
; write back the updated source pointer
_bcdone	movfw	FSR0L
	movwf	EP0_DATA_IN_PTR
	return

; copy flash contents (PMDATH/PMDATL) to EP0IN_BUF (FSR1)
ep0_read_dfu_in
; BANKED_EP0IN_CNT was already cleared in ep0_read_in
	ldfsr1d	EP0IN_BUF		; set up destination pointer
; PMADRH:PMADRL = wValueL << 5
	movfw	BANKED_EP0OUT_BUF+wValueL
	banksel	PMADRL
	clrf	PMCON1
	clrf	PMADRL
	lsrf	WREG,f
	btfsc   STATUS,C
	bsf	PMADRL,5
	lsrf	WREG,f
	btfsc   STATUS,C
	bsf	PMADRL,6
	lsrf	WREG,f
	btfsc   STATUS,C
	bsf	PMADRL,7
	movwf	PMADRH
	clrw
_pmcopy
	sublw	EP0_BUF_SIZE		; have we filled the buffer?
	bz	_pmbail
	banksel	PMADRL
	bsf	PMCON1,RD		; read word from flash
	nop				; 2 required nops
	nop
	movfw	PMDATL
	movwi	FSR1++
	movfw	PMDATH
	movwi	FSR1++
	incf	PMADRL,f		; increment LSB of Program Memory address
	banksel	BANKED_EP0OUT_STAT
	incf	BANKED_EP0IN_CNT,f	; increase number of bytes copied by two
	incf	BANKED_EP0IN_CNT,f
	movfw	BANKED_EP0IN_CNT	; save to test on the next iteration
	goto	_pmcopy
_pmbail
	return


; temporary to provide delay for _tflush ; incorporate into flash write
ret	return



;;; Main function
;;; BSR=1 (OSCCON bank)
bootloader_start
; Configure the oscillator (48MHz from INTOSC using 3x PLL)
	movlw	(1<<SPLLEN)|(1<<SPLLMULT)|(1<<IRCF3)|(1<<IRCF2)|(1<<IRCF1)|(1<<IRCF0)
	movwf	OSCCON

; Wait for the oscillator and PLL to stabilize
_wosc	movlw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	andwf	OSCSTAT,w
	sublw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	bnz	_wosc

; do not run application if the watchdog timed out (providing a mechanism for the app to trigger a firmware update)
	btfss	STATUS,NOT_TO
	goto	_bootloader_main

; Check for valid application code: the lower 8 bits of the first word cannot be 0xFF
	call	app_is_present
	bz	_bootloader_main	; if we have no application, enter bootloader mode

; We have a valid application? Check if the entry pin is grounded
	banksel	PORTA
	btfss	PORTA,RA3
	goto	_bootloader_main	; enter bootloader mode if input is low

; We have a valid application and the entry pin is high. Start the application.
	banksel	OPTION_REG
	bsf	OPTION_REG,NOT_WPUEN	; but first, disable weak pullups
	if APP_ENTRY_POINT>=2048
	pagesel	APP_ENTRY_POINT
	endif
	goto	APP_ENTRY_POINT

; Not entering application code: initialize the USB interface and wait for commands.
_bootloader_main
; Enable active clock tuning
	banksel	ACTCON
	movlw	(1<<ACTSRC)|(1<<ACTEN)
	movwf	ACTCON		; source = USB

; Initialize USB
	call	usb_init

; Attach to the bus (could be a subroutine, but inlining it saves 2 instructions)
_usb_attach
	banksel	UCON		; reset UCON
	clrf	UCON
	banksel	PIE2
	bsf	PIE2,USBIE	; enable USB interrupts
	bsf	INTCON,PEIE
	banksel	UCON
_usben	bsf	UCON,USBEN	; enable USB module and wait until ready
	btfss	UCON,USBEN
	goto	_usben

; Enable interrupts and enter an idle loop
; (Loop code is located at the top of the file, in the first 256 words of
; program memory)
	goto	bootloader_main_loop



;;; Determines if application code is present in flash memory.
;;; arguments:	none
;;; returns:	Z flag cleared if application code is present
;;; clobbers:	W, FSR0
app_is_present
	clrf	FSR0L
	movlw	(high APP_ENTRY_POINT)|0x80	; need to set high bit to indicate program memory
	movwf	FSR0H
	moviw	FSR0
	incw				; if W was 0xFF, it'll be 0 now
	return				; Z flag will be unset if app code is present



;;; Initializes the USB system and resets all associated registers.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_init
; disable USB interrupts
	banksel	PIE2
	bcf	PIE2,USBIE
; clear USB registers
	banksel	UEIR
	clrf	UEIR
	clrf	UIR
; disable endpoints we won't use
	clrf	UEP1
	clrf	UEP2
	clrf	UEP3
	clrf	UEP4
	clrf	UEP5
	clrf	UEP6
	clrf	UEP7
; set configuration
	clrf	UEIE		; don't need any error interrupts
	movlw	(1<<UPUEN)|(1<<FSEN)
	movwf	UCFG		; enable pullups, full speed, no ping-pong buffering
	movlw	(1<<TRNIE)|(1<<URSTIE)
	movwf	UIE		; only need interrupts for transaction complete and reset
; clear all BDT entries, variables, and buffers
	clrf	FSR0L
	movlw	high BDT_START	; BDT starts at 0x2000
	movwf	FSR0H
	movlw	USED_RAM_LEN
	movwf	FSR1H		; loop count
	movlw	0
_ramclr	movwi	FSR0++
	decfsz	FSR1H,f
	goto	_ramclr
; reset ping-pong buffers and address
	banksel	UCON
	bsf	UCON,PPBRST
	clrf	UADDR
	bcf	UCON,PKTDIS	; enable packet processing
	bcf	UCON,PPBRST	; clear ping-pong buffer reset flag
; flush pending transactions
_tflush	btfss	UIR,TRNIF
	goto	_initep
	bcf	UIR,TRNIF
	call	ret		; need at least 6 cycles before checking TRNIF again
	goto	_tflush
; initialize endpoints:
; EP0 (in and out) for control
; my intuition was that I should wait until a SET_CONFIGURATION is received
; before setting up endpoints 1 and 2... but there seemed to be a timing issue
; when doing so, so I moved them here
_initep	movlw	(1<<EPHSHK)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP0
; initialize endpoint buffers and counts
	banksel	BANKED_EP0OUT_ADRL
	movlw	low EP0OUT_BUF	; set endpoint 0 OUT address low
	movwf	BANKED_EP0OUT_ADRL
	movlw	low EP0IN_BUF	; set endpoint 0 IN address low
	movwf	BANKED_EP0IN_ADRL
	movlw	EPBUF_ADRH	; set all ADRH values
	movwf	BANKED_EP0OUT_ADRH
	movwf	BANKED_EP0IN_ADRH
	goto	arm_ep0_out



;;; Descriptors 

; Place all the descriptors at the end of the bootloader region.
; This serves 2 purposes: 1) as long as the total length of all descriptors is
; less than 256, we can address them with an 8-bit pointer,
; and 2) the assembler will raise an error if space is exhausted.
	org	BOOTLOADER_SIZE-ALL_DESCS_TOTAL_LEN
DESCRIPTOR_ADRH	equ	high $
DEVICE_DESCRIPTOR
	dt	DEVICE_DESC_LEN	; bLength
	dt	0x01		; bDescriptorType
	; bcdUSB (USB 1.0)
OPPORTUNISTIC_0_CONSTANT
	dt	0x00	; bcdUSB LSB
OPPORTUNISTIC_1_CONSTANT
	dt	0x01	; bcdUSB MSB
	dt	0xFE		; bDeviceClass
	dt	0x01		; bDeviceSubclass
	dt	0x00		; bDeviceProtocol
	dt	EP0_BUF_SIZE	; bMaxPacketSize0
	dt	low USB_VENDOR_ID, high USB_VENDOR_ID	; idVendor
	dt	low USB_PRODUCT_ID, high USB_PRODUCT_ID	; idProduct
	dt	0x01, 0x00	; bcdDevice (1)
	dt	0x00		; iManufacturer
	dt	0x00		; iProduct
	dt	0x01		; iSerialNumber
	dt	0x01		; bNumConfigurations

CONFIGURATION_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x02		; bDescriptorType
	dt	low CONFIG_DESC_TOTAL_LEN, high CONFIG_DESC_TOTAL_LEN	; wTotalLength
	dt	0x01		; bNumInterfaces
	dt	0x01		; bConfigurationValue
	dt	0x00		; iConfiguration
	dt	0x80		; bmAttributes
	dt	0x32		; bMaxPower

INTERFACE_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x04		; bDescriptorType (INTERFACE)
	dt	0x00		; bInterfaceNumber
	dt	0x00		; bAlternateSetting
	dt	0x00		; bNumEndpoints
	dt	0xFE		; bInterfaceClass
	dt	0x01		; bInterfaceSubclass
	dt	0x00		; bInterfaceProtocol
	dt	0x00		; iInterface

	if (OPPORTUNISTIC_0_CONSTANT>>8) != (OPPORTUNISTIC_1_CONSTANT>>8)
	error "CONSTANT_0 and CONSTANT_1 must be in the same 256-word region"
	endif

DFU_STATUS_RESPONSE
	dt	0x00			; bStatus = OK
	dt	0x00, 0x00, 0x00	; bwPollTimeout
DFU_STATE_RESPONSE
	dt	0x02			; bState = dfuIDLE
	dt	0x00			; iString

; extract nibbles from serial number
SN1	equ	(SERIAL_NUMBER>>12) & 0xF
SN2	equ	(SERIAL_NUMBER>>8) & 0xF
SN3	equ	(SERIAL_NUMBER>>4) & 0xF
SN4	equ	SERIAL_NUMBER & 0xF

SERIAL_NUMBER_STRING_DESCRIPTOR
	dt	SERIAL_NUM_DESC_LEN	; bLength
	dt	0x03		; bDescriptorType (STRING)
	dt	'0'+SN1+((SN1>9)*7), 0x00	; convert hex digits to ASCII
	dt	'0'+SN2+((SN2>9)*7), 0x00
	dt	'0'+SN3+((SN3>9)*7), 0x00
	dt	'0'+SN4+((SN4>9)*7), 0x00
	
; Raise an error if the descriptors aren't properly aligned. (This means you
; changed the descriptors withouth updating the definition of ALL_DESCS_TOTAL_LEN.)
	if $!=BOOTLOADER_SIZE
	error "Descriptors must be aligned with the end of the bootloader region"
	endif

	end
