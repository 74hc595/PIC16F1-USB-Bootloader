; vim:noet:sw=8:ts=8:ai:syn=pic
;
; USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
;
; Labels that do not begin with an underscore can be called as functions.
; Labels that begin with an underscore are not safe to call, they should only
; be reached via goto.

LOGGING_ENABLED		equ	1
USE_STRING_DESCRIPTORS	equ	0

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
	__config _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF
	__config _CONFIG2, _WRT_HALF & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_3x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LVP_OFF



;;; Constants
FOSC			equ	48000000
BAUD			equ	38400
BAUDVAL			equ	(FOSC/(16*BAUD))-1	; BRG16=0, BRGH=1

DEVICE_DESC_LEN		equ	18
CONFIG_DESC_TOTAL_LEN	equ	67

EP0_BUF_SIZE 		equ	8	; endpoint 0 buffer size
EP1_OUT_BUF_SIZE	equ	64	; endpoint 1 OUT (CDC data) buffer size
EP1_IN_BUF_SIZE		equ	1	; endpoint 1 IN (CDC data) buffer size (only need 1 byte to return status codes)

; Since we're only using 5 endpoints, use the BDT area for buffers,
; and use the 4 bytes normally occupied by the EP2 OUT buffer descriptor for variables.
USB_STATE		equ	BANKED_EP2OUT+0
EP0_DATA_IN_PTRL	equ	BANKED_EP2OUT+1	; pointer to block of data to be sent
EP0_DATA_IN_PTRH	equ	BANKED_EP2OUT+2	;   in the current EP0 IN transaction
EP0_DATA_IN_COUNT	equ	BANKED_EP2OUT+3	; remaining bytes to be sent

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

; USB_STATE bit flags
IS_CONTROL_WRITE	equ	0	; current endpoint 0 transaction is a control write
ADDRESS_PENDING		equ	1	; need to set address in next IN transaction
DEVICE_CONFIGURED	equ	2	; the device is configured



;;; Vectors
	org	0x0000
RESET_VECT
; configure the oscillator (48MHz from INTOSC using 3x PLL)
	banksel	OSCCON
	movlw	(1<<SPLLEN)|(1<<SPLLMULT)|(1<<IRCF3)|(1<<IRCF2)|(1<<IRCF1)|(1<<IRCF0)
	movwf	OSCCON
	goto	bootloader_start

	org	0x0004
INTERRUPT_VECT
	banksel	UIR
; reset?
	btfss	UIR,URSTIF
	goto	_uidle
	call	usb_init
	banksel	PIE2
	bsf	PIE2,USBIE	; reenable USB interrupts
	banksel	UIR
	bcf	UIR,URSTIF	; clear the flag
; idle? just clear the flag (TODO)
_uidle	btfsc	UIR,IDLEIF
	bcf	UIR,IDLEIF
; error?
	if LOGGING_ENABLED
	btfss	UIR,UERRIF
	goto	_utrans
	mlog
	mlogch	'E',0
	mloghex	1,LOG_NEWLINE
	mlogf	UEIR
	mlogend
	lbnksel	UEIR
	else
	btfsc	UIR,UERRIF
	endif
	clrf	UEIR		; clear error flags
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
	if USE_STRING_DESCRIPTORS
	movlw	DESC_STRING
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_string_descriptor
	endif
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
	movwf	EP0_DATA_IN_PTRL
	movlw	high DEVICE_DESCRIPTOR
	movwf	EP0_DATA_IN_PTRH
	movlw	DEVICE_DESC_LEN
	movwf	EP0_DATA_IN_COUNT
	goto	_adjust_data_in_count
_config_descriptor
	movlw	low CONFIGURATION_DESCRIPTOR
	movwf	EP0_DATA_IN_PTRL
	movlw	high CONFIGURATION_DESCRIPTOR
	movwf	EP0_DATA_IN_PTRH
	movlw	CONFIG_DESC_TOTAL_LEN	; length includes all subordinate descriptors
	movwf	EP0_DATA_IN_COUNT

	if USE_STRING_DESCRIPTORS
	goto	_adjust_data_in_count
_string_descriptor
	movlw	NUM_STRING_DESCRIPTORS		; ensure descriptor number is valid
	subwf	BANKED_EP0OUT_BUF+wValueL,w
	bc	_invalid_string_descriptor_index
	ldfsr0	STRING_DESCRIPTOR_OFFSETS	; index into offsets table
	movfw	BANKED_EP0OUT_BUF+wValueL
	addwf	FSR0L,f
	movlw	0
	addwfc	FSR0H,f
	moviw	FSR0				; get offset
	addwf	FSR0L,f				; add offset to current pointer
	movlw	0
	addwfc	FSR0H,f
	moviw	FSR0				; get descriptor length
	movwf	EP0_DATA_IN_COUNT
	movfw	FSR0L				; save current pointer location 
	movwf	EP0_DATA_IN_PTRL
	movfw	FSR0H
	movwf	EP0_DATA_IN_PTRH
	goto	_adjust_data_in_count
_invalid_string_descriptor_index
	mlog
	mlogch	'?',0
	mlogch	'S',0
	mloghex	1,LOG_NEWLINE
	mlogf	BANKED_EP0OUT_BUF+wValueH
	mlogend
	lbnksel	BANKED_EP0OUT_BUF
	goto	_usb_ctrl_invalid
	endif

; the count needs to be set to the minimum of the descriptor's length (in W)
; and the requested length
_adjust_data_in_count
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
	movwf	EP0_DATA_IN_PTRL
	movlw	high CONFIGURATION_0_CONSTANT
	movwf	EP0_DATA_IN_PTRH
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



;;; Reads data from EP0_DATA_IN_PTRL:EP0_DATA_IN_PTRH, copies it to the EP0 IN buffer,
;;; and decrements EP0_DATA_IN_COUNT.
;;; arguments:	BSR=0
;;; returns:	EP0_DATA_IN_PTRL:EP0_DATA_IN_PTRH advanced
;;;		EP0_DATA_IN_COUNT decremented
;;; clobbers:	W, FSR0, FSR1
ep0_read_in
	bcf	BANKED_EP0IN_STAT,UOWN	; make sure we have ownership of the buffer
	mloghex 2,0
	mlogf	EP0_DATA_IN_PTRH
	mlogf	EP0_DATA_IN_PTRL
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
	movfw	EP0_DATA_IN_PTRL	; set up source pointer
	movwf	FSR0L
	movfw	EP0_DATA_IN_PTRH
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
	movwf	EP0_DATA_IN_PTRL
	movfw	FSR0H
	movwf	EP0_DATA_IN_PTRH
; print the bytes that were copied
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
	banksel	UEP1
	movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP1
	movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPINEN)
	movwf	UEP2
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

	if 0
	movlw	1			; send a 1 character response
	movwf	BANKED_EP1IN_CNT
	movfw	BANKED_EP1OUT_BUF	; copy the first received character to the IN buffer
	andlw	b'00011111'		; but fix the upper the 3 bits to 010
	iorlw	b'01000000'
	movwf	BANKED_EP1IN_BUF	; so it will be echoed back
	mlogch	'%',0
	mloghex	2,LOG_SPACE|LOG_NEWLINE
	mlogf	BANKED_EP1OUT_CNT	; echo the character count
	mlogf	BANKED_EP1OUT_BUF	; echo the first character
	mlogend
	lbnksel	BANKED_EP1OUT_STAT
	goto	arm_ep1_out
	endif

	movf	BANKED_EP1OUT_CNT,f	; test for a zero-length packet
	bz	arm_ep1_out		; (just ignore them and rearm the OUT buffer)
	bcf	BANKED_EP1IN_STAT,UOWN
	call	bootloader_exec_cmd	; execute command; status returned in W
	banksel	BANKED_EP1IN_BUF
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	movlw	1
	movwf	BANKED_EP1IN_CNT	; output byte count is 1
	mlogch	'&',0
	mloghex	1,LOG_NEWLINE
	mlogf	BANKED_EP1IN_BUF
	lbnksel	BANKED_EP1IN_CNT
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

; erases the row of flash in PMADRH:PMADRL
; BSR=3
_bootloader_erase
	movlw	(1<<FREE)|(1<<WREN)	; enable write and erase to program memory
	movwf	PMCON1
	call	flash_unlock		; stalls until erase finishes
	bcf	PMCON1,WREN		; clear write enable flag
	retlw	BSTAT_OK

_bootloader_write
	retlw	1



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
	return



;;; Main function
bootloader_start
; Wait for the oscillator and PLL to stabilize
_wosc	movlw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	andwf	OSCSTAT,w
	sublw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	bnz	_wosc

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
	call	usb_attach
	bsf	INTCON,GIE	; enable interrupts

; Main loop
loop	
	if LOGGING_ENABLED
; Print any pending characters in the log
	call	log_service
	endif
	goto	loop



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
; disable all endpoints
	clrf	UEP0
	clrf	UEP1
	clrf	UEP2
	clrf	UEP3
	clrf	UEP4
	clrf	UEP5
	clrf	UEP6
	clrf	UEP7
; set configuration
	movlw	(1<<UPUEN)|(1<<FSEN)
	movwf	UCFG		; enable pullups, full speed, no ping-pong buffering
	movlw	(1<<BTSEE)|(1<<BTOEE)|(1<<DFN8EE)|(1<<CRC16EE)|(1<<CRC5EE)|(1<<PIDEE)
	movwf	UEIE		; enable all error interrupts
	movlw	(1<<IDLEIE)|(1<<TRNIE)|(1<<UERRIE)|(1<<URSTIE)
	movwf	UIE		; all interrupts except stall, SOF, and Bus Activity Detect
; clear all BDT entries, variables, and buffers
	ldfsr0d	BDT_START
	movlw	USED_RAM_LEN
	movwf	FSR1H		; loop count
	movlw	0
_ramclr	movwi	FSR0++
	decfsz	FSR1H,f
	goto	_ramclr
; reset ping-pong buffers and address
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
; initialize endpoint 0
_initep	movlw	(1<<EPHSHK)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP0
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



;;; Enables the USB module.
;;; Assumes all registers have been properly configured by calling usb_init.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0
usb_attach
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
ret	return



;;; Includes
	if LOGGING_ENABLED
	include "log.asm"
	endif



;;; Descriptors
DEVICE_DESCRIPTOR
	dt	0x12		; bLength
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
	dt	0x00		; iSerialNumber	(TODO)
	dt	0x01		; bNumConfigurations

CONFIGURATION_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x02		; bDescriptorType
	dt	0x43, 0x00	; wTotalLength
	dt	0x02		; bNumInterfaces
	dt	0x01		; bConfigurationValue
	dt	0x00		; iConfiguration
	dt	b'11000000'	; bmAttributes (self-powered)
	dt	0x19		; bMaxPower (25 -> 50 mA)

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


	if USE_STRING_DESCRIPTORS
NUM_STRING_DESCRIPTORS	equ	4
STRING_DESCRIPTOR_OFFSETS
	dt	STRING_DESCRIPTOR_LANGS-$
	dt	STRING_DESCRIPTOR_MANUFACTURER-$
	dt	STRING_DESCRIPTOR_PRODUCT-$
	dt	STRING_DESCRIPTOR_SERIAL_NUMBER-$

STRING_DESCRIPTOR_LANGS
	dt	0x04		; bLength
	dt	0x03		; bDescriptorTYpe
	dt	0x09, 0x04	; wLANGID[0] (American English)

STRING_DESCRIPTOR_MANUFACTURER
	dt	0x1A		; bLength
	dt	0x03		; bDescriptorType
	dt	'M', 0x00
	dt	'a', 0x00
	dt	't', 0x00
	dt	't', 0x00
	dt	' ', 0x00
	dt	'S', 0x00
	dt	'a', 0x00
	dt	'r', 0x00
	dt	'n', 0x00
	dt	'o', 0x00
	dt	'f', 0x00
	dt	'f', 0x00

STRING_DESCRIPTOR_PRODUCT
	dt	0x12		; bLength
	dt	0x03		; bDescriptorType
	dt	'U', 0x00
	dt	'S', 0x00
	dt	'B', 0x00
	dt	' ', 0x00
	dt	'T', 0x00
	dt	'e', 0x00
	dt	's', 0x00
	dt	't', 0x00

STRING_DESCRIPTOR_SERIAL_NUMBER
	dt	0x0E		; bLength
	dt	0x03		; bDescriptorType
	dt	'C', 0x00
	dt	'R', 0x00
	dt	'M', 0x00
	dt	'1', 0x00
	dt	'1', 0x00
	dt	'4', 0x00
	endif
	
	end	
