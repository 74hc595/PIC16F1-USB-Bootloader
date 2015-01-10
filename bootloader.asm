; vim:noet:sw=8:ts=8:ai
;
; USB Mass Storage Bootloader for PIC16(L)F1454/5/9
;
; Notes on function calls:
; FSR0L, FSR0H, FSR1L, and FSR1H are used to pass additional arguments
; to functions, and may be used as scratch registers inside of functions.

	include "p16f1454.inc"
	radix dec
	list n=0,st=off
	errorlevel -302
	
;;; Macros
	nolist
;;; Loads the address of the given symbol into FSR0.
ldfsr0	macro 	x
	movlw	low x
	movwf	FSR0L
	movlw	high x
	movwf	FSR0H
	endm

;;; Loads the address of the given symbol into FSR1.
ldfsr1	macro 	x
	movlw	low x
	movwf	FSR1L
	movlw	high x
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
	bra	$-1
	endm

;;; Waits until the bit in the specified register is cleared.
waitfc	macro 	reg,bit
	btfsc	reg,bit
	bra	$-1
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

	

;;; Configuration
	list
	__config _CONFIG1, _FCMEN_OFF & _IESO_OFF & _BOREN_OFF & _WDTE_OFF & _FOSC_INTOSC
	__config _CONFIG2, _PLLEN_ENABLED & _PLLMULT_3x & _USBLSCLK_48MHz & _CPUDIV_NOCLKDIV



;;; Constants
FOSC	equ	48000000
BAUD	equ	38400
BAUDVAL	equ	(FOSC/(16*BAUD))-1	; BRG16=0, BRGH=1




;;; Vectors
	org	0x0000
RESET_VECT
	goto	bootloader_start



;;; Main function
	org	0x0005
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
loop	ldfsr0	STR_HELLO
	call	uart_print_str

; Print a packed string
	ldpmadr	STR_PACKED
	call	uart_print_packed_str

; Print device ID
	ldfsr0	STR_DEVID
	call	uart_print_str
	banksel	PMADRL
	ldpmadr	0x8006		; device ID
	bsf	PMCON1,CFGS	; select configuration space
	bsf	PMCON1,RD	; read word
	nop
	nop
	movfw	PMDATL		; move lsb to FSR0L
	movwf	FSR0L
	movfw	PMDATH		; move msb to W
	call	uart_print_hex16
	call	uart_print_nl
	
; Print revision ID
	ldfsr0	STR_REVID
	call	uart_print_str
	banksel PMADRL
	decf	PMADRL,f	; previous word is revision ID
	bsf	PMCON1,RD	; read word
	nop
	nop
	movfw	PMDATL		; move lsb to FSR0L
	movwf	FSR0L
	movfw	PMDATH		; move msb to W
	call	uart_print_hex16
	call	uart_print_nl

; Toggle the LED
	banksel	LATA
	movlw	(1<<LATA4)
	xorwf	LATA,f
; Loop
	bra	loop



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
	bra	uart_print_str	; next character



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
	bra	_l1



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


;;; Strings
STR_HELLO
	dt	"hello, world\n\0"
STR_DEVID
	dt	"device id = \0"
STR_REVID
	dt	"revision id = \0"
STR_PACKED
	da	"abcdef\n\0"
	end	
