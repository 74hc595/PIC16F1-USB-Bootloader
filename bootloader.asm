; vim:noet:sw=8:ts=8:ai:syn=pic
;
; USB Mass Storage Bootloader for PIC16(L)F1454/5/9
;
; Notes on function calls:
; FSR0L, FSR0H, FSR1L, and FSR1H are used to pass additional arguments
; to functions, and may be used as scratch registers inside of functions.

	include "p16f1454.inc"
	include "bdt.inc"
	include "usb.inc"
	radix dec
	list n=0,st=off
	errorlevel -302


;;; Configuration
	__config _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF
	__config _CONFIG2, _WRT_OFF & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_3x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LVP_OFF



;;; Constants
FOSC		equ	48000000
BAUD		equ	38400
BAUDVAL		equ	(FOSC/(16*BAUD))-1	; BRG16=0, BRGH=1

EP0_BUF_SIZE 	equ	8	; endpoint 0 buffer size
RESERVED_RAM_SZ	equ	2	; amount of RAM reserved by the bootloader
EP0OUT_EVEN_BUF	equ	BUF_START+RESERVED_RAM_SZ
EP0OUT_ODD_BUF	equ	EP0OUT_EVEN_BUF+EP0_BUF_SIZE
EP0IN_BUF	equ	EP0OUT_ODD_BUF+EP0_BUF_SIZE



;;; Variables
RESERVED_RAM	equ	BANKED_BUF_START
USB_STATE	equ	RESERVED_RAM+0
NEXT_EP0_OUT_BD	equ	RESERVED_RAM+1	; either 0x00 (even) or 0x04 (odd)

;;; USB_STATE bit flags
LAST_EP0_WRITE	equ	0	; last endpoint 0 transaction is a control write
EP0_HANDLED	equ	1	; last endpoint 0 transaction was handled; will stall if 0



;;; Macros
	nolist
;;; Loads the address of the given symbol into FSR0.
ldfsr0	macro 	x
	movlw	low x
	movwf	FSR0L
	movlw	high x
	movwf	FSR0H
	endm

;;; Loads the given address in data space into FSR0.
;;; (This ensures that the high bit is not set, which the high directive may
;;; do implicitly)
ldfsr0d	macro	x
	movlw	low x
	movwf	FSR0L
	movlw	(high x) & 0x7F
	movwf	FSR0H
	endm

;;; Loads the address of the given symbol into FSR1.
ldfsr1	macro 	x
	movlw	low x
	movwf	FSR1L
	movlw	high x
	movwf	FSR1H
	endm

;;; Loads the given address in data space into FSR1.
;;; (This ensures that the high bit is not set, which the high directive may
;;; do implicitly)
ldfsr1d	macro	x
	movlw	low x
	movwf	FSR1L
	movlw	(high x) & 0x7F
	movwf	FSR1H
	endm

;;; Loads the address of the given symbol into PMADRH:PMADRL.
ldpmadr	macro	x
	banksel	PMADRL
	movlw	low x
	movwf	PMADRL
	movlw	high x
	movwf	PMADRH
	endm

;;; Waits until the bit in the specified register is set.
waitfs	macro 	reg,bit
	btfss	reg,bit
	goto	$-1
	endm

;;; Waits until the bit in the specified register is cleared.
waitfc	macro 	reg,bit
	btfsc	reg,bit
	goto	$-1
	endm

;;; Returns if the Z flag is set.
retz	macro
	skpnz
	return
	endm

;;; Returns if the Z flag is not set.
retnz	macro
	skpz
	return
	endm

;;; Subtracts the literal from W. (opposite of 'sublw')
subwl	macro	x
	addlw	256-x
	endm

;;; Increments W.
incw	macro
	addlw	1
	endm

;;; Decrements W.
decw	macro
	addlw	255
	endm

	
	
;;; Vectors
	list
	org	0x0000
RESET_VECT
	goto	bootloader_start
	org	0x0004
INTERRUPT_VECT
	call	usb_service
	retfie



;;; Main function
	org	0x0006
bootloader_start
; Configure the oscillator (48MHz from INTOSC using 3x PLL)
	banksel	OSCCON
	movlw	(1<<SPLLEN)|(1<<SPLLMULT)|(1<<IRCF3)|(1<<IRCF2)|(1<<IRCF1)|(1<<IRCF0)
	movwf	OSCCON

; Wait for the oscillator and PLL to stabilize
_wait_osc_ready
	movlw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	andwf	OSCSTAT,w
	sublw	(1<<PLLRDY)|(1<<HFIOFR)|(1<<HFIOFS)
	bnz	_wait_osc_ready

; Enable active clock tuning
	banksel	ACTCON
	movlw	(1<<ACTSRC)|(1<<ACTEN)
	movwf	ACTCON		; source = USB

; Turn on the LED
	banksel TRISA
	bcf	TRISA,TRISA4	; RA4 as output
	banksel	LATA
	bsf	LATA,LATA4	; set RA4 high

; Enable the UART
	banksel	SPBRGL
	movlw	low BAUDVAL	; set baud rate divisor
	movwf	SPBRGL
	bsf	TXSTA,BRGH	; high speed
	bsf	RCSTA,SPEN	; enable serial port
	bsf	TXSTA,TXEN	; enable transmission

; Print a string
	ldfsr0	STR_ON
	call	uart_print_str

; Initialize USB
	call	usb_init
	call	usb_attach
	bsf	INTCON,GIE	; enable interrupts

; Main loop
loop	
; Blink the LED
	banksel	LATA
	movlw	(1<<LATA4)
	xorwf	LATA,f
; Perform USB tasks
	goto	loop



;;; Initializes the USB system and resets all associated registers.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_init
	ldfsr0	STR_USB_INIT
	call	uart_print_str
; clear our state
	banksel	USB_STATE
	clrf	USB_STATE
	clrf	NEXT_EP0_OUT_BD
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
	movlw	(1<<UPUEN)|(1<<FSEN)|(1<<PPB0)
	movwf	UCFG		; enable pullups, full speed, ping-pong buffer for EP0 OUT
	movlw	(1<<BTSEE)|(1<<BTOEE)|(1<<DFN8EE)|(1<<CRC16EE)|(1<<CRC5EE)|(1<<PIDEE)
	movwf	UEIE		; enable all error interrupts
	movlw	(1<<IDLEIE)|(1<<TRNIE)|(1<<UERRIE)|(1<<URSTIE)
	movwf	UIE		; all interrupts except stall, SOF, and Bus Activity Detect
; clear all BDT entries
	ldfsr0d	BDT_START
	movlw	BDT_LEN
	movwf	FSR1H		; loop count
	movlw	0
_bdtclr	movwi	FSR0++
	decfsz	FSR1H,f
	goto	_bdtclr
; reset ping-pong buffers and address
	bsf	UCON,PPBRST
	clrf	UADDR
	bcf	UCON,PKTDIS	; enable packet processing
	bcf	UCON,PPBRST	; clear ping-pong buffer reset flag
; flush pending transactions
_tflush	btfss	UIR,TRNIF
	goto	_initep
	bcf	UIR,TRNIF
	call	_ret		; need at least 6 cycles before checking TRNIF again
	goto	_tflush
; initialize endpoint 0
_initep	movlw	(1<<EPHSHK)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP0
	ldfsr0d	EP0OUT_EVEN
	movlw	EP0_BUF_SIZE
	movwi	1[FSR0]		; set CNT for even
	movwi	5[FSR0]		; set CNT for odd
	movlw	low EP0OUT_EVEN_BUF
	movwi	2[FSR0]		; set ADRL for even
	movlw	low EP0OUT_ODD_BUF
	movwi	6[FSR0]		; set ADRL for odd
	movlw	(EP0OUT_EVEN_BUF>>8)
	movwi	3[FSR0]		; set ADRH for even
	movwi	7[FSR0]		; set ADRH for odd
	movlw	_DAT0|_BSTALL
	movwi	0[FSR0]		; set STAT
	bsf	INDF0,UOWN	; give ownership to SIE
_ret	return	



;;; Enables the USB module.
;;; Assumes all registers have been properly configured by calling usb_init.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0
usb_attach
	ldfsr0	STR_USB_ATTACH
	call	uart_print_str
	banksel	UCON		; reset UCON
	clrf	UCON
	banksel	PIE2
	bsf	PIE2,USBIE	; enable USB interrupts
	bsf	INTCON,PEIE
	banksel	UCON
_usben	bsf	UCON,USBEN	; enable USB module and wait until ready
	btfss	UCON,USBEN
	goto	_usben
	ldfsr0	STR_DONE
	goto	uart_print_str



;;; Services the USB bus.
;;; Should be called from the interrupt handler, or at least every 1ms.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_service
	ldfsr0	STR_UEIR
	call	uart_print_str
	banksel	UEIR
	movfw	UEIR
	call	uart_print_hex
	call	uart_print_str
	banksel	UIR
	movfw	UIR
	call	uart_print_hex
	call	uart_print_str
	banksel	UCON
	movfw	UCON
	call	uart_print_hex
	call	uart_print_nl
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
	btfss	UIR,UERRIF
	goto	_utrans
	clrf	UEIR		; clear error flags
	ldfsr0	STR_ERROR
	call	uart_print_str
; service transactions
_utrans	banksel	UIR
	btfss	UIR,TRNIF
	goto	_usdone
	movfw	USTAT		; stash the status in a temp register
	movwf	FSR1H
	bcf	UIR,TRNIF	; clear flag and advance USTAT fifo
	andlw	b'01111000'	; check endpoint number
	bnz	_utrans		; if not endpoint 0, loop (TODO)
	movfw	FSR1H		; bring original USTAT value back to W
	call	usb_service_ep0	; handle the control message
	goto	_utrans
; clear USB interrupt
_usdone	banksel	PIR2
	bcf	PIR2,USBIF
	return



;;; Handles a control transfer on endpoint 0.
;;; arguments:	USTAT value in W
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_service_ep0
	movwf	FSR1H		; save status in a temp register
	banksel	USB_STATE
	movlw	low EP0OUT_EVEN	; get the address of the next BDT entry
	btfss	FSR1H,PPBI	; if not using the odd buffer, next buffer will be odd
	addlw	BDT_ENTRY_SIZE
	movwf	NEXT_EP0_OUT_BD
	ldfsr0	STR_CTRL_EP0
	call	uart_print_str
	movfw	FSR1H
	call	uart_print_hex
	call	uart_print_nl
	btfsc	FSR1H,DIR	; is it an IN transfer or an OUT/SETUP?
	goto	usb_ctrl_in
; it's an OUT or SETUP transfer
	ldfsr0d	EP0OUT_EVEN	; load the current buffer pointer (EVEN or ODD)
	btfsc	FSR1H,PPBI	; add 4 if last transaction wrote to the ODD buffer
	addfsr	FSR0,BDT_ENTRY_SIZE
	moviw	0[FSR0]		; get BD0STAT
	andlw	b'00111100'	; isolate PID bits
	sublw	PID_SETUP	; is it a SETUP packet?
	bnz	usb_ctrl_out	; if not, it's a regular OUT
	; it's a SETUP packet--fall through



;;; Handles a SETUP control transfer on endpoint 0.
;;; arguments:	pointer to current BDT entry in FSR0
;;; returns:	none
;;; clobbers:	W, FSR0, FSR1
usb_ctrl_setup
; ensure none of the OUT endpoints are armed
	bcf	INDF0,UOWN	; FSR0 currently points to BD0STAT (even or odd)
	movlw	BDT_ENTRY_SIZE
	xorwf	FSR0L,f		; clear UOWN in other buffer
	bcf	INDF0,UOWN
	xorwf	FSR0L,f		; bring pointer back to current buffer
; get the address of the buffer
	moviw	2[FSR0]		; get ADRL
	movwf	FSR1H		; temporary
	moviw	3[FSR0]		; get ADRH
	movwf	FSR0H
	movfw	FSR1H		; bring ADRL out of temporary
	movwf	FSR0L
; check if this is a standard request (first byte)
	moviw	0[FSR0]
	andlw	b'01100000'	; bits 5 and 6 should be 0 for a standard request
	bnz	_usb_ctrl_complete
; get the request number (second byte)
	moviw	1[FSR0]
	movwf	FSR1H		; save in a temporary
; is it GET_DESCRIPTOR?
	movlw	GET_DESCRIPTOR
	subwf	FSR1H,w
	bz	_usb_get_descriptor
; unhandled request
	ldfsr0	STR_UNHANDLED_REQUEST
	call	uart_print_str
	movfw	FSR1H
	call	uart_print_hex
	call	uart_print_nl

; Finishes a SETUP transaction.
_usb_ctrl_complete
	banksel	UCON
	bcf	UCON,PKTDIS	; reenable packet processing
; if the request wasn't handled, stall
	banksel	USB_STATE
	btfsc	USB_STATE,EP0_HANDLED
	goto	_derp
	ldfsr0	STR_STALL
	call	uart_print_str
	ldfsr0d	EP0IN		; stall the IN endpoint
	movlw	_DAT0|_DTSEN|_BSTALL
	movwi	FSR0
	bsf	INDF0,UOWN	; and arm it
	banksel	USB_STATE
	movfw	NEXT_EP0_OUT_BD	; get pointer to next OUT buffer
	movwf	FSR0L
	movlw	_BSTALL
	movwi	FSR0		; stall the OUT endpoint
	bsf	INDF0,UOWN	; and arm it
	return

_derp	banksel	USB_STATE
	bcf	USB_STATE,EP0_HANDLED	; clear for next transaction
	return

; Handles a GET_DESCRIPTOR request.
; Pointer to request buffer is in FSR0.
_usb_get_descriptor
	moviw	3[FSR0]
	movwf	FSR1H		; save in a temporary
; device descriptor?
	movlw	DESC_DEVICE
	subwf	FSR1H,w
	bnz	_other_descriptor
	;banksel	USB_STATE
	;bsf	USB_STATE,EP0_HANDLED

_other_descriptor
	ldfsr0	STR_UNHANDLED_DESCRIPTOR
	call	uart_print_str
	movfw	FSR1H
	call	uart_print_hex
	call	uart_print_nl
	goto	_usb_ctrl_complete



;;; Handles an OUT control transfer on endpoint 0.
;;; arguments:	pointer to current BDT entry in FSR0
;;; returns:	none
;;; clobbers:
usb_ctrl_out
	ldfsr0	STR_CTRL_OUT
	goto	uart_print_str



;;; Handles an IN control transfer on endpoint 0.
;;; arguments:	pointer to current BDT entry in FSR0
;;; returns:	none
;;; clobbers:	
usb_ctrl_in
	ldfsr0	STR_CTRL_IN
	goto	uart_print_str



;;; Transmits a newline over the UART.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR
uart_print_nl
	movlw	'\n'		; fall through to uart_print_char



;;; Transmits a character over the UART and returns when complete.
;;; arguments:	character in W
;;; returns:	none
;;; clobbers:	BSR
uart_print_char
	banksel	TXREG
	movwf	TXREG		; transmit the character
	banksel	PIR1		; need 1 cycle delay before checking TXIF
	waitfs	PIR1,TXIF	; loop until character is sent
	return



;;; Transmits a null-terminated string over the UART.
;;; arguments:	pointer to string in FSR0
;;; returns:	none
;;; clobbers:	FSR0, W, BSR
uart_print_str
	moviw	FSR0++		; get a character and advance
	retz			; return if zero
	call	uart_print_char
	goto	uart_print_str	; next character



;;; Transmits a null-terminated packed (2 characters per word) ASCII string.
;;; from program memory over the UART.
;;; arguments:	pointer to string in PMADRH:PMADRL
;;		BSR=3
;;; returns:	none
;;; clobbers:	W, BSR, PMADRH:PMADRL
uart_print_packed_str
	bcf	PMCON1,CFGS	; don't read from configuration space
_l1	bsf	PMCON1,RD	; initiate read
	nop
	nop
	lslf	PMDATL,f	; shift lsb of high byte into PMDATH
	rlf	PMDATH,w
	tstf	WREG		; needed because rlf doesn't affect the Z flag
	retz			; return if high byte is 0
	call	uart_print_char	; print high byte
	banksel	PMDATL
	lsrf	PMDATL,w	; readjust PMDATL (this *does* affect the Z flag)
	retz			; return if low byte is 0
	call	uart_print_char	; print low byte
	banksel	PMADRL		; advance to next word
	incf	PMADRL,f
	skpnc
	incf	PMADRH,f
	goto	_l1



;;; Converts the lower nibble of W to its ASCII hexadecimal representation.
w_to_hex_ascii macro
	andlw	b'00001111'
	subwl	10		; is W >= 10?
	skpnc
	addlw	7		; if so, shift to letters
	addlw	'A'-7		; shift to printable ASCII
	endm



;;; Prints a 16-bit word over the UART.
;;; arguments:	msb in W
;;;		lsb in FSR0L
;;; returns:	none
;;; clobbers:	W, BSR, FSR1H
uart_print_hex16
	call	uart_print_hex	; print W (msb)
	movfw	FSR0L		; fall through and print lsb



;;; Transmits a hexadecimal byte over the UART.
;;; arguments:	byte in W
;;; returns:	none
;;; clobbers:	W, BSR, FSR1H
uart_print_hex
	movwf	FSR1H		; save byte
	swapf	WREG,w		; get high nibble
	w_to_hex_ascii
	call	uart_print_char	; print high nibble
	movfw	FSR1H		; bring back original byte
	w_to_hex_ascii
	goto	uart_print_char



;;; Descriptors
DEVICE_DESCRIPTOR
	dt	0x12		; bLength
	dt	0x01		; bDescriptorType
	dt	0x00, 0x02	; bcdUSB USB 2.0
	dt	0xFF		; bDeviceClass (vendor-defined)
	dt	0x00		; bDeviceSubclass
	dt	0x00		; bDeviceProtocol
	dt	0x08		; bMaxPacketSize0 (8 bytes)
	dt	0xd8, 0x04	; idVendor (Microchip)
	dt	0xdd, 0xdd	; idProduct (fake value)
	dt	0x01, 0x00	; bcdDevice (1)
	dt	0x00		; iManufacturer (TODO)
	dt	0x00		; iProduct (TODO)
	dt	0x00		; iSerialNumber (TODO)
	dt	0x01		; bNumConfigurations


;;; Strings
STR_ON
	dt	"Power on\n\0"
STR_USB_INIT
	dt	"USB init\n\0"
STR_USB_ATTACH
	dt	"USB attach...\0"
STR_UEIR
	dt	"UEIR=\0"
STR_UIR
	dt	" UIR=\0"
STR_UCON
	dt	" UCON=\0"
STR_ERROR
	dt	"error\n\0"
STR_CTRL_EP0
	dt	"servicing endpoint 0, USTAT=\0"
STR_CTRL_SETUP
	dt	"control SETUP\n\0"
STR_CTRL_OUT
	dt	"control OUT\n\0"
STR_CTRL_IN
	dt	"control IN\n\0"
STR_UNHANDLED_REQUEST
	dt	"unhandled request \0"
STR_GET_DESCRIPTOR
	dt	"GET_DESCRIPTOR\n\0"
STR_UNHANDLED_DESCRIPTOR
	dt	"unhandled descriptor \0"
STR_STALL
	dt	"sending STALL\n\0"
STR_DONE
	dt	"done\n\0"
	end	
