; vim:noet:sw=8:ts=8:ai:syn=pic
;
; USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
;
; Bootloader is entered if the MCLR/RA3 pin is grounded at power-up or reset,
; or if there is no application programmed. (The internal pull-up is used,
; no external resistor is necessary.)
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
; With logging enabled, the bootloader will not fit in 512 words.
; Use this only for debugging!
LOGGING_ENABLED		equ	0

; If 1, the bootloader verifies each row of flash after it's written.
; I put this behind a switch in case I needed to disable it to save space.
VERIFY_WRITES		equ	1



	radix dec
	list n=0,st=off
	include "p16f1454.inc"
	nolist
	include "macros.inc"
	include "bdt.inc"
	include "usb.inc"
	include "protocol_constants.inc"
	include "log_macros.inc"
	list
	errorlevel -302



;;; Configuration
	if LOGGING_ENABLED
WRT_CONFIG		equ	_WRT_HALF
	else
WRT_CONFIG		equ	_WRT_BOOT
	endif

	__config _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF
	__config _CONFIG2, WRT_CONFIG & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_3x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LVP_OFF



;;; Constants
FOSC			equ	48000000
BAUD			equ	38400
BAUDVAL			equ	(FOSC/(16*BAUD))-1	; BRG16=0, BRGH=1

SERIAL_NUMBER_DIGIT_CNT	equ	4
	ifndef SERIAL_NUMBER
	variable SERIAL_NUMBER=0	; Why doesnt 'equ' work here? Go figure
	endif

DEVICE_DESC_LEN		equ	18	; device descriptor length
CONFIG_DESC_TOTAL_LEN	equ	67	; total length of configuration descriptor and sub-descriptors
SERIAL_NUM_DESC_LEN	equ	2+(SERIAL_NUMBER_DIGIT_CNT*2)
ALL_DESCS_TOTAL_LEN	equ	DEVICE_DESC_LEN+CONFIG_DESC_TOTAL_LEN+SERIAL_NUM_DESC_LEN

EP0_BUF_SIZE 		equ	8	; endpoint 0 buffer size
EP1_OUT_BUF_SIZE	equ	64	; endpoint 1 OUT (CDC data) buffer size
EP1_IN_BUF_SIZE		equ	1	; endpoint 1 IN (CDC data) buffer size (only need 1 byte to return status codes)

; Since we're only using 5 endpoints, use the BDT area for buffers,
; and use the 4 bytes normally occupied by the EP2 OUT buffer descriptor for variables.
USB_STATE		equ	BANKED_EP2OUT+0
EP0_DATA_IN_PTR		equ	BANKED_EP2OUT+1	; pointer to descriptor to be sent (low byte only)
EP0_DATA_IN_COUNT	equ	BANKED_EP2OUT+2	; remaining bytes to be sent
APP_POWER_CONFIG	equ	BANKED_EP2OUT+3	; application power config byte
EP0OUT_BUF		equ	EP3OUT
BANKED_EP0OUT_BUF	equ	BANKED_EP3OUT	; buffers go immediately after EP2 IN's buffer descriptor
EP0IN_BUF		equ	EP0OUT_BUF+EP0_BUF_SIZE
BANKED_EP0IN_BUF	equ	BANKED_EP0OUT_BUF+EP0_BUF_SIZE

; Use another byte to store the checksum we use to verify writes
EXTRA_VARS_LEN		equ	1
EXPECTED_CHECKSUM	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE	; for saving expected checksum

EP1IN_BUF		equ	EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN
BANKED_EP1IN_BUF	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN

EP1OUT_BUF		equ	EP1IN_BUF+EP1_IN_BUF_SIZE	; only use 1 byte for EP1 IN
BANKED_EP1OUT_BUF	equ	BANKED_EP1IN_BUF+EP1_IN_BUF_SIZE

; High byte of all endpoint buffers.
EPBUF_ADRH		equ	(EP0OUT_BUF>>8)
	if ((EP0IN_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1OUT_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1IN_BUF>>8) != (EP0OUT_BUF>>8))
	error "Endpoint buffers must be in the same 256-word region"
	endif

USED_RAM_LEN		equ	EP1OUT_BUF+EP1_OUT_BUF_SIZE-BDT_START

	if LOGGING_ENABLED
BOOTLOADER_SIZE		equ	0x1000
	else
BOOTLOADER_SIZE		equ	0x200
	endif

; Application code locations
APP_ENTRY_POINT		equ	BOOTLOADER_SIZE
APP_CONFIG		equ	BOOTLOADER_SIZE+2
APP_INTERRUPT		equ	BOOTLOADER_SIZE+4

; USB_STATE bit flags
IS_CONTROL_WRITE	equ	0	; current endpoint 0 transaction is a control write
ADDRESS_PENDING		equ	1	; need to set address in next IN transaction
DEVICE_CONFIGURED	equ	2	; the device is configured



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
	if LOGGING_ENABLED
; for 4k-word mode: if TOSH < 0x10, we're in the bootloader
; if TOSH >= 0x10, jump to the application interrupt handler
	movlw	high BOOTLOADER_SIZE
	subwf	TOSH,w
	bnc	_bootloader_interrupt
	pagesel	APP_INTERRUPT
	goto	APP_INTERRUPT
	else
; for 512-word mode: if TOSH == 0, we're in the bootloader
; if TOSH != 0, jump to the application interrupt handler
	tstf	TOSH
	bnz	APP_INTERRUPT
	endif

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
	bnz	_ucdc		; if not endpoint 0, it's a CDC message
	call	usb_service_ep0	; handle the control message
	goto	_utrans
; clear USB interrupt
_usdone	banksel	PIR2
	bcf	PIR2,USBIF
	retfie
_ucdc	call	usb_service_cdc	; USTAT value is still in FSR1H
	goto	_utrans



;;; Idle loop. In bootloader mode, the MCU just spins here, and all USB
;;; communication is interrupt-driven.
;;; This snippet is deliberately located within the first 256 words of program
;;; memory, so we can easily check in the interrupt handler if the interrupt
;;; occurred while executing application code or bootloader code.
;;; (TOSH will be 0x00 when executing bootloader code, i.e. this snippet)
bootloader_main_loop
	bsf	INTCON,GIE	; enable interrupts
_loop
	if LOGGING_ENABLED
; Print any pending characters in the log
	call	log_service
	endif
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
	if LOGGING_ENABLED
	bnz	_usb_ctrl_out	; if not, it's a regular OUT
	else
	bnz	arm_ep0_out	; if not, it's a regular OUT, just rearm the buffer
	endif
	; it's a SETUP packet--fall through

; Handles a SETUP control transfer on endpoint 0.
; BSR=0
_usb_ctrl_setup
	bcf	USB_STATE,IS_CONTROL_WRITE
; get bmRequestType, but don't bother checking whether it's standard/class/vendor...
; the CDC and standard requests we'll receive have distinct bRequest numbers
	movfw	BANKED_EP0OUT_BUF+bmRequestType
	btfss	BANKED_EP0OUT_BUF+bmRequestType,7	; is this host->device?
	bsf	USB_STATE,IS_CONTROL_WRITE		; if so, this is a control write
; print packet
	mlog
	mlogch	'P',0
	mloghex	8,LOG_NEWLINE|LOG_SPACE
	mlogf	BANKED_EP0OUT_BUF+0
	mlogf	BANKED_EP0OUT_BUF+1
	mlogf	BANKED_EP0OUT_BUF+2
	mlogf	BANKED_EP0OUT_BUF+3
	mlogf	BANKED_EP0OUT_BUF+4
	mlogf	BANKED_EP0OUT_BUF+5
	mlogf	BANKED_EP0OUT_BUF+6
	mlogf	BANKED_EP0OUT_BUF+7
	mlogend
	lbnksel	BANKED_EP0OUT_BUF
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
; unhandled request
_unhreq	mlog
	mlogch	'?',0
	mlogch	'R',0
	mloghex	1,LOG_NEWLINE
	mlogf	BANKED_EP0OUT_BUF+bRequest
	mlogend
	; fall through to _usb_ctrl_invalid

; Finishes a rejected SETUP transaction: the endpoints are stalled
_usb_ctrl_invalid
	banksel	UCON
	bcf	UCON,PKTDIS	; reenable packet processing
	banksel	USB_STATE
	logch	'X',LOG_NEWLINE
	lbnksel	BANKED_EP0IN
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
_cwrite	clrf	BANKED_EP0IN_CNT	; we'll be sending a zero-length packet
	movlw	_DAT0|_DTSEN|_BSTALL	; make OUT buffer ready for next SETUP packet
	goto	_armbfs			; arm OUT and IN buffers



; Handles a Get Descriptor request.
; BSR=0
_usb_get_descriptor
; check descriptor type
	movlw	DESC_DEVICE
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_device_descriptor
	movlw	DESC_CONFIG
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_config_descriptor
	movlw	DESC_STRING
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_string_descriptor
; unsupported descriptor type
	mlog
	mlogch	'?',0
	mlogch	'D',0
	mloghex	1,LOG_NEWLINE
	mlogf	BANKED_EP0OUT_BUF+wValueH
	mlogend
	lbnksel	BANKED_EP0OUT_BUF
	goto	_usb_ctrl_invalid
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
	call	cdc_init
	goto	_usb_ctrl_complete

; Handles a Get Configuration request.
; BSR=0
_usb_get_configuration
; load a pointer to either a 0 or a 1 in ROM
; the 0 and 1 have been chosen so that they are adjacent
	movlw	low CONFIGURATION_0_CONSTANT
	btfsc	USB_STATE,DEVICE_CONFIGURED
	incw
	movwf	EP0_DATA_IN_PTR
	movlw	1
	movwf	EP0_DATA_IN_COUNT
	goto	_usb_ctrl_complete

	if LOGGING_ENABLED
; Handles an OUT control transfer on endpoint 0.
; BSR=0
_usb_ctrl_out
	logch	'O',LOG_NEWLINE
	lbnksel	EP0_BUF_SIZE
; Only time this will get called is in the status stage of a control read,
; since we don't support any control writes with a data stage.
; All we have to do is re-arm the OUT endpoint.
	goto	arm_ep0_out
	endif

; Handles an IN control transfer on endpoint 0.
; BSR=0
_usb_ctrl_in
	logch	'I',LOG_NEWLINE
	lbnksel	USB_STATE
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
	mlog
	mlogch	'A',0
	mloghex	1,LOG_NEWLINE
	mlogf	UADDR
	mlogend
	return



;;; Reads descriptor data from EP0_DATA_IN_PTR, copies it to the EP0 IN buffer,
;;; and decrements EP0_DATA_IN_COUNT.
;;; arguments:	BSR=0
;;; returns:	EP0_DATA_IN_PTRL advanced
;;;		EP0_DATA_IN_COUNT decremented
;;; clobbers:	W, FSR0, FSR1
ep0_read_in
	bcf	BANKED_EP0IN_STAT,UOWN	; make sure we have ownership of the buffer
	mloghex 1,0
	mlogf	EP0_DATA_IN_PTR
	mloghex	1,LOG_SPACE
	mlogf	EP0_DATA_IN_COUNT
	lbnksel	BANKED_EP0IN_CNT
	clrf	BANKED_EP0IN_CNT	; initialize buffer size to 0
	tstf	EP0_DATA_IN_COUNT	; do nothing if there are 0 bytes to send
	if LOGGING_ENABLED
	bz	_nodata
	else
	retz
	endif
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
; if we're sending the configuration descriptor, we need to inject the app's
; values for bus power/self power and max current consumption
_check_for_config_bmattributes
	movlw	(low CONFIGURATION_DESCRIPTOR)+EP0_BUF_SIZE
	subwf	FSR0L,w
	bnz	_check_for_config_bmaxpower
; if we're sending the first 8 bytes of the configuration descriptor,
; set bit 6 of bmAttributes if the application is self-powered
	btfsc	APP_POWER_CONFIG,0
	bsf	BANKED_EP0IN_BUF+7,6
	if LOGGING_ENABLED
	goto	_log_copied_bytes
	else
	return
	endif
_check_for_config_bmaxpower
	movlw	(low CONFIGURATION_DESCRIPTOR)+(EP0_BUF_SIZE*2)
	subwf	FSR0L,w
	if LOGGING_ENABLED
	bnz	_log_copied_bytes
	else
	retnz
	endif
; if we're sending the second 8 bytes of the configuration descriptor,
; replace bMaxPower with the app's value
	movfw	APP_POWER_CONFIG
	bcf	WREG,0			; value is in the upper 7 bits
	movwf	BANKED_EP0IN_BUF+0
; print the bytes that were copied
_log_copied_bytes
	mlogch	'[',0
	mloghex	1,0
	mlogf	BANKED_EP0IN_CNT
	mlogch	']',0
	mloghex	8,LOG_SPACE|LOG_NEWLINE
	mlogf	BANKED_EP0IN_BUF+0
	mlogf	BANKED_EP0IN_BUF+1
	mlogf	BANKED_EP0IN_BUF+2
	mlogf	BANKED_EP0IN_BUF+3
	mlogf	BANKED_EP0IN_BUF+4
	mlogf	BANKED_EP0IN_BUF+5
	mlogf	BANKED_EP0IN_BUF+6
	mlogf	BANKED_EP0IN_BUF+7
	lbnksel	USB_STATE
	return
	if LOGGING_ENABLED
_nodata	mlogch	' ',LOG_NEWLINE
	lbnksel	USB_STATE
	return
	endif



;;; Initializes the buffers for the CDC endpoints (1 OUT, 1 IN, and 2 IN).
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR=0
cdc_init
	mlogch	'@',LOG_NEWLINE
	banksel	BANKED_EP1OUT_STAT
	call	arm_ep1_out
	; arm EP1 IN buffer, clearing data toggle bit
	clrw

; arms endpoint 1 IN, toggling DTS if W=(1<<DTS)
arm_ep1_in
	clrf	BANKED_EP1IN_CNT	; next packet will have 0 length (unless another OUT is received)
	andwf	BANKED_EP1IN_STAT,f	; clear all bits (except DTS if bit is set in W)
	xorwf	BANKED_EP1IN_STAT,f	; update data toggle (if bit is set in W)
	bsf	BANKED_EP1IN_STAT,UOWN
	return



;;; Services a transaction on one of the CDC endpoints.
;;; arguments:	USTAT value in FSR1H
;;;		BSR=0
;;; returns:	none
;;; clobbers:	W, FSR0, FSR1
usb_service_cdc
	movlw	(1<<DTS)
	retbfs	FSR1H,ENDP1		; ignore endpoint 2
	bbfs	FSR1H,DIR,arm_ep1_in	; if endpoint 1 IN, rearm buffer
	movf	BANKED_EP1OUT_CNT,f	; test for a zero-length packet
	bz	arm_ep1_out		; (just ignore them and rearm the OUT buffer)
	bcf	BANKED_EP1IN_STAT,UOWN
	call	bootloader_exec_cmd	; execute command; status returned in W
	banksel	BANKED_EP1IN_BUF
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	movlw	1
	movwf	BANKED_EP1IN_CNT	; output byte count is 1
	bsf	BANKED_EP1IN_STAT,UOWN
	; fall through to arm_ep1_out

arm_ep1_out
	movlw	EP1_OUT_BUF_SIZE	; set CNT
	movwf	BANKED_EP1OUT_CNT
	clrf	BANKED_EP1OUT_STAT	; ignore data toggle
	bsf	BANKED_EP1OUT_STAT,UOWN	; rearm OUT buffer
	return



;;; Executes a bootloader command.
;;; arguments:	command payload in EP1 OUT buffer
;;; 		BSR=0
;;; returns:	status code in W
;;; clobbers:	W, BSR, FSR0, FSR1
bootloader_exec_cmd
; check length of data packet
	movlw	BCMD_SET_PARAMS_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_set_params
	movlw	BCMD_WRITE_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_write
	movlw	BCMD_RESET_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_reset
	retlw	BSTAT_INVALID_COMMAND

; Resets the device if the received byte matches the reset character.
_bootloader_reset
	movlw	BCMD_RESET_CHAR
	subwf	BANKED_EP1OUT_BUF,w	; check received character
	skpz
	retlw	BSTAT_INVALID_COMMAND
; command is valid, reset the device
	reset

; Sets the write address, expected checksum of the next 32 words,
; and erases the row at that address if the last byte of the command matches
; the "erase" character.
; BSR=0
_bootloader_set_params
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_CKSUM	; expected checksum
	movwf	EXPECTED_CHECKSUM			; save for verification during write command
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ERASE
	movwf	FSR1L	; temp
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRL	; address lower bits
	movwf	FSR1H	; temp
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRH	; address upper bits 
	banksel	PMADRH
	movwf	PMADRH
	movfw	FSR1H	; bring lower bits out of temp
	movwf	PMADRL
; do we need to erase?
	movlw	BCMD_ERASE_CHAR
	subwf	FSR1L,w
	skpz
	retlw	BSTAT_OK	; if no reset command is given, return OK

; Erases the row of flash in PMADRH:PMADRL.
; BSR=3
_bootloader_erase
	movlw	(1<<FREE)|(1<<WREN)	; enable write and erase to program memory
	movwf	PMCON1
	call	flash_unlock		; stalls until erase finishes
_wdone	bcf	PMCON1,WREN		; clear write enable flag
	retlw	BSTAT_OK

; Verifies that the checksum of the 32 words (64 bytes) in the EP1 OUT buffer
; matches the previously sent value. If so, the 32 bytes are then written to
; flash memory at the address in PMADRH:PMADRL. (set by a prior command)
; BSR=0
_bootloader_write
; The expected checksum is the two's complement of the sum of the bytes.
; If the data is valid, we can add the checksum to the sum of the bytes and
; the result will be 0. We initialize a temporary register with the expected
; checksum, and then add each byte to it as it's processed.
; If the value in the temp register is 0 after all 64 bytes have been copied
; to the write latches, proceed with the write.
	movfw	EXPECTED_CHECKSUM
	movwf	FSR1L			; use a temp for the running checksum
	ldfsr0d	EP1OUT_BUF		; set up read pointer
	movlw	(1<<LWLO)|(1<<WREN)	; write to latches only
	banksel	PMCON1
	movwf	PMCON1
; simultaneously compute the checksum of the 32 words and copy them to the
; write latches
	movlw	32			; number of words to write minus 1
	movwf	FSR1H			; used for loop count
_wloop	moviw	FSR0++			; load lower byte
	addwf	FSR1L,f			; add lower byte to checksum
	movwf	PMDATL			; copy to write latch
	moviw	FSR0++			; load upper byte
	addwf	FSR1L,f			; add upper byte to checksum
	movwf	PMDATH			; copy to write latch
; after writing the 32nd word to PMDATH:PMDATL, don't execute the unlock sequence
; or advance the address pointer!
	decf	FSR1H,f			; decrement loop count
	bz	_wcksum			; if 0, we're done writing to the latches
; still have more words to go
	call	flash_unlock		; execute unlock sequence
	incf	PMADRL,f		; increment write address
	goto	_wloop
; verify the checksum
_wcksum	clrf	PMCON1
	tstf	FSR1L
	skpz
	retlw	BSTAT_INVALID_CHECKSUM	; if there's a mismatch, abort the write
; checksum is valid, write the data
	bsf	PMCON1,WREN
	call	flash_unlock		; stalls until write finishes
	if VERIFY_WRITES==0
	goto	_wdone
	else
; verify the write: compare each byte in the buffer to its counterpart that
; was just written to flash.
; we do this backwards so we don't waste instructions resetting the pointers.
; (note: PMADRH:PMADRL is already pointing at the last written word, but FSR0
; is pointing to one byte past the end of the buffer)
	clrf	PMCON1			; clear write enable
	bsf	FSR1H,5			; set loop count to 32 (just need to set one bit because it's already 0)
_vloop	bsf	PMCON1,RD		; read word from flash
	nop				; 2 required nops
	nop
	moviw	--FSR0			; get high byte of expected word
	subwf	PMDATH,w		; compare with high byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	moviw	--FSR0			; get low byte of expected word
	subwf	PMDATL,w		; compare with low byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	decf	PMADRL,f		; decrement read address
	decfsz	FSR1H,f			; decrement loop count
	goto	_vloop
	retlw	BSTAT_OK
	endif


;;; Executes the flash unlock sequence, performing an erase or write.
;;; arguments:	PMCON1 bits CFGS, LWLO, FREE and WREN set appropriately
;;;		BSR=3
;;; returns:	none
;;; clobbers:	W
flash_unlock
	movlw	0x55
	movwf	PMCON2
	movlw	0xAA
	movwf	PMCON2
	bsf	PMCON1,WR
	nop
	nop
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

	if LOGGING_ENABLED
; Enable the UART
	banksel	SPBRGL
	movlw	low BAUDVAL	; set baud rate divisor
	movwf	SPBRGL
	bsf	TXSTA,BRGH	; high speed
	bsf	RCSTA,SPEN	; enable serial port
	bsf	TXSTA,TXEN	; enable transmission
; Print a power-on character
	call	log_init
	logch	'^',LOG_NEWLINE
	endif

; Initialize USB
	call	usb_init

; Attach to the bus (could be a subroutine, but inlining it saves 2 instructions)
_usb_attach
	logch	'A',0
	banksel	UCON		; reset UCON
	clrf	UCON
	banksel	PIE2
	bsf	PIE2,USBIE	; enable USB interrupts
	bsf	INTCON,PEIE
	banksel	UCON
_usben	bsf	UCON,USBEN	; enable USB module and wait until ready
	btfss	UCON,USBEN
	goto	_usben
	logch	'!',LOG_NEWLINE

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



;;; Gets the application's power config byte and stores it in APP_POWER_CONFIG.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0
get_app_power_config
	banksel	APP_POWER_CONFIG
	movlw	0x33			; default value: bus-powered, max current 100 mA
	movwf	APP_POWER_CONFIG
	call	app_is_present
	retz				; if Z flag is set, we have no application, just return
	if LOGGING_ENABLED
	pagesel	APP_CONFIG
	endif
	call	APP_CONFIG		; config value returned in W
	banksel	APP_POWER_CONFIG
	movwf	APP_POWER_CONFIG
	return



;;; Initializes the USB system and resets all associated registers.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_init
	logch	'R',LOG_NEWLINE
; disable USB interrupts
	banksel	PIE2
	bcf	PIE2,USBIE
; clear USB registers
	banksel	UEIR
	clrf	UEIR
	clrf	UIR
; disable endpoints we won't use
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
; get the app's power configuration (if it's present)
	call	get_app_power_config
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
; 0 for control
; 1 for CDC bulk data
; 2 for CDC notifications (though it's never actually used)
; my intuition was that I should wait until a SET_CONFIGURATION is received
; before setting up endpoints 1 and 2... but there seemed to be a timing issue
; when doing so, so I moved them here
_initep	movlw	(1<<EPHSHK)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP0
	movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP1
	movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPINEN)
	movwf	UEP2
; initialize endpoint buffers and counts
	banksel	BANKED_EP0OUT_ADRL
	movlw	low EP0OUT_BUF	; set endpoint 0 OUT address low
	movwf	BANKED_EP0OUT_ADRL
	movlw	low EP0IN_BUF	; set endpoint 0 IN address low
	movwf	BANKED_EP0IN_ADRL
	movlw	low EP1OUT_BUF	; set endpoint 1 OUT address low
	movwf	BANKED_EP1OUT_ADRL
	movlw	low EP1IN_BUF	; set endpoint 1 IN address low
	movwf	BANKED_EP1IN_ADRL
	movlw	EPBUF_ADRH	; set all ADRH values
	movwf	BANKED_EP0OUT_ADRH
	movwf	BANKED_EP0IN_ADRH
	movwf	BANKED_EP1OUT_ADRH
	movwf	BANKED_EP1IN_ADRH
	goto	arm_ep0_out



;;; Includes
	if LOGGING_ENABLED
	include "log.asm"
	endif



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
	dt	0x00, 0x02	; bcdUSB (USB 2.0)
	dt	0x02		; bDeviceClass (communication device)
	dt	0x00		; bDeviceSubclass
	dt	0x00		; bDeviceProtocol
	dt	0x08		; bMaxPacketSize0 (8 bytes)
	dt	0xd8, 0x04	; idVendor (Microchip)
	dt	0xdd, 0xdd	; idProduct (fake value)
	dt	0x01, 0x00	; bcdDevice (1)
	dt	0x00		; iManufacturer
	dt	0x00		; iProduct
	dt	0x01		; iSerialNumber
	dt	0x01		; bNumConfigurations

CONFIGURATION_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x02		; bDescriptorType
	dt	CONFIG_DESC_TOTAL_LEN, 0x00	; wTotalLength
	dt	0x02		; bNumInterfaces
	dt	0x01		; bConfigurationValue
	dt	0x00		; iConfiguration
	dt	0x80		; bmAttributes
	dt	0x11		; bMaxPower

INTERFACE_DESCRIPTOR_0
	dt	0x09		; bLength
	dt	0x04		; bDescriptorType (INTERFACE)
	dt	0x00		; bInterfaceNumber
CONFIGURATION_0_CONSTANT
	dt	0x00		; bAlternateSetting
CONFIGURATION_1_CONSTANT
	dt	0x01		; bNumEndpoints
	dt	0x02		; bInterfaceClass (communication)
	dt	0x02		; bInterfaceSubclass (abstract control model)
	dt	0x01		; bInterfaceProtocol (V.25ter, common AT commands)
	dt	0x00		; iInterface

	if (CONFIGURATION_0_CONSTANT>>8) != (CONFIGURATION_1_CONSTANT>>8)
	error "CONSTANT_0 and CONSTANT_1 must be in the same 256-word region"
	endif

HEADER_FUNCTIONAL_DESCRIPTOR
	dt	0x05		; bFunctionLength
	dt	0x24		; bDescriptorType (CS_INTERFACE)
	dt	0x00		; bDescriptorSubtype (header functional descriptor)
	dt	0x10,0x01	; bcdCDC (specification version, 1.1)

ABSTRACT_CONTROL_MANAGEMENT_FUNCTIONAL_DESCRIPTOR
	dt	0x04		; bFunctionLength
	dt	0x24		; bDescriptorType (CS_INTERFACE)
	dt	0x02		; bDescriptorSubtype (abstract control management functional descriptor)
	dt	0x02		; bmCapabilities

UNION_FUNCTIONAL_DESCRIPTOR
	dt	0x05		; bFunctionLength
	dt	0x24		; bDescriptorType (CS_INTERFACE)
	dt	0x06		; bDescriptorSubtype (union functional descriptor)
	dt	0x00		; bMasterInterface
	dt	0x01		; bSlaveInterface0

CALL_MANAGEMENT_FUNCTIONAL_DESCRIPTOR
	dt	0x05		; bFunctionLength
	dt	0x24		; bDescriptorType (CS_INTERFACE)
	dt	0x01		; bDescriptorSubtype (call management functional descriptor)
	dt	0x00		; bmCapabilities (doesn't handle call management)
	dt	0x01		; dDataInterface

ENDPOINT_DESCRIPTOR_2_IN
	dt	0x07		; bLength
	dt	0x05		; bDescriptorType (ENDPOINT)
	dt	0x82		; bEndpointAddress (2 IN)
	dt	0x03		; bmAttributes (transfer type: interrupt)
	dt	0x08, 0x00	; wMaxPacketSize (8)
	dt	0x7f		; bInterval

INTERFACE_DESCRIPTOR_1
	dt	0x09		; bLength
	dt	0x04		; bDescriptorType (INTERFACE)
	dt	0x01		; bInterfaceNumber
	dt	0x00		; bAlternateSetting
	dt	0x02		; bNumEndpoints
	dt	0x0a		; bInterfaceClass (data)
	dt	0x00		; bInterfaceSubclass
	dt	0x00		; bInterfaceProtocol
	dt	0x00		; iInterface

ENDPOINT_DESCRIPTOR_1_IN
	dt	0x07		; bLength
	dt	0x05		; bDescriptorType (ENDPOINT)
	dt	0x81		; bEndpointAddress (1 IN)
	dt	0x02		; bmAttributes (transfer type: bulk)
	dt	0x40, 0x00	; wMaxPacketSize (64)
	dt	0x00		; bInterval

ENDPOINT_DESCRIPTOR_1_OUT
	dt	0x07		; bLength
	dt	0x05		; bDescriptorType (ENDPOINT)
	dt	0x01		; bEndpointAddress (1 OUT)
	dt	0x02		; bmAttributes (transfer type: bulk)
	dt	0x40, 0x00	; wMaxPacketSize (64)
	dt	0x00		; bInterval

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
	error "Descriptors must be aligned with the end of the 512-word region"
	endif

	end
