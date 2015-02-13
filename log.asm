; vim:noet:sw=8:ts=8:ai:syn=pic
;
; USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; Minimal, low-latency UART logging system.
; Uses a 256-byte circular buffer aligned to a 256-byte boundary.
;
; To write to the log, use the macros in log_macros.inc.
; To print the contents of the log over the UART, call log_service
; periodically from your main loop.
; The write macros are safe to use in an interrupt handler.

; Constants
FOSC		equ	48000000
BAUD		equ	38400
BAUDVAL		equ	(FOSC/(16*BAUD))-1	; BRG16=0, BRGH=1


; Linear address of the 256-byte buffer.
; Must be aligned to a 256-byte boundary.
LOG_BUFFER	equ	0x2200

; Banked RAM locations used by logging system
; (directly before the log buffer)
LOG_WREG_SAVE	equ	0x4d0		; bank 0x0A
LOG_FSR0L_SAVE	equ	0x4d1
LOG_FSR0H_SAVE	equ	0x4d2
LOG_HEAD	equ	0x4d3
LOG_TAIL	equ	0x4d4
LOG_FMT_FLAGS	equ	0x4d5		; hex count/newline/space flags
LOG_CURR_BYTE	equ	0x4d6

uart_init
	banksel	SPBRGL
	movlw	low BAUDVAL	; set baud rate divisor
	movwf	SPBRGL
	bsf	TXSTA,BRGH	; high speed
	bsf	RCSTA,SPEN	; enable serial port
	bsf	TXSTA,TXEN	; enable transmission
	return

log_init
	banksel	LOG_HEAD
	clrf	LOG_HEAD
	clrf	LOG_TAIL
	clrf	LOG_FMT_FLAGS
	return

log_service
	banksel	LOG_HEAD
_lsloop	movfw	LOG_TAIL	; if head == tail, buffer is empty
	subwf	LOG_HEAD,w
	skpnz
	return
; dequeue a byte
	movlw	LOG_BUFFER>>8
	movwf	FSR0H
	movfw	LOG_HEAD
	movwf	FSR0L
	moviw	FSR0++
	movwf	LOG_CURR_BYTE
; save advanced head pointer
	movfw	FSR0L
	movwf	LOG_HEAD
; are we in hex mode?
	btfsc	LOG_FMT_FLAGS,7
	goto	_lsphex
; process the byte
	btfsc	LOG_CURR_BYTE,7	; is this a hex marker?
	goto	_lsshex
; we're just in ASCII mode
	movfw	LOG_CURR_BYTE
	andlw	b'00111111'	; mask off high bits
	addlw	32		; upper bits need to be set properly
	call	_uart_print_ch
	btfsc	LOG_CURR_BYTE,6	; need to print a trailing newline?
	call	_uart_print_nl
	goto	_lsloop
_lsshex	movfw	LOG_CURR_BYTE	; starting hex mode? write flags and loop
	movwf	LOG_FMT_FLAGS
	goto	_lsloop	
_lsphex	btfsc	LOG_FMT_FLAGS,5	; need to print a leading space?
	call	_uart_print_space
	call	_uart_print_hex	; prints byte in LOG_CURR_BYTE
	movfw	LOG_FMT_FLAGS	; decrement hex byte count
	andlw	b'00011111'	; isolate count bits
	decfsz	WREG,w		; subtract 1
	goto	_nxthex
; hex counter reached 0; print newline if needed and clear format flags
	btfsc	LOG_FMT_FLAGS,6	; need to print a trailing newline?
	call	_uart_print_nl
	clrf	LOG_FMT_FLAGS
	goto	_lsloop
; hex counter > 0; write back new count
_nxthex	xorwf	LOG_FMT_FLAGS,w	; xor-swap new count and old flags
	xorwf	LOG_FMT_FLAGS,f
	xorwf	LOG_FMT_FLAGS,w	; LOG_FMT_FLAGS contains new count, but NOT old flags
	andlw	b'11100000'	; isolate old flags
	iorwf	LOG_FMT_FLAGS,f	; write them back to LOG_FMT_FLAGS
	goto	_lsloop



log_single_byte
	banksel	LOG_WREG_SAVE	; need to save W and FSR0
	movwf	LOG_WREG_SAVE
	movfw	FSR0L
	movwf	LOG_FSR0L_SAVE
	movfw	FSR0H
	movwf	LOG_FSR0H_SAVE
	call	_log_byte_inner	; and fall through
log_multi_byte_end
	banksel	LOG_FSR0L_SAVE	; (redundant when falling through)
; restore registers
	movfw	LOG_FSR0L_SAVE
	movwf	FSR0L
	movfw	LOG_FSR0H_SAVE
	movwf	FSR0H
	movfw	LOG_WREG_SAVE
	return

log_multi_byte_start
; save FSR0
	banksel	LOG_WREG_SAVE
	movfw	FSR0L
	movwf	LOG_FSR0L_SAVE
	movfw	FSR0H
	movwf	LOG_FSR0H_SAVE
	return

log_byte
	banksel	LOG_WREG_SAVE
	movwf	LOG_WREG_SAVE	;and fall through
; load tail pointer into FSR0
_log_byte_inner
	movlw	LOG_BUFFER>>8
	movwf	FSR0H
	movfw	LOG_TAIL
	movwf	FSR0L
	movfw	LOG_WREG_SAVE	; write byte into log buffer
	movwi	FSR0++		; advance tail pointer
; save new tail pointer
	movfw	FSR0L		; high byte is ignored for 256-byte wraparound
	movwf	LOG_TAIL
; always keep one slot open; if (tail+1)%256 == head, advance head
	subwf	LOG_HEAD,w
	skpnz
	incf	LOG_HEAD,f
	return


_uart_print_space
	movlw	' '
	goto	_uart_print_ch
_uart_print_nl
	movlw	'\n'
; prints char in W
_uart_print_ch
	banksel	TXREG
	movwf	TXREG		; transmit the character
	banksel	PIR1		; need 1 cycle delay before checking TXIF
	btfss	PIR1,TXIF	; loop until character is sent
	goto	$-1
	banksel	LOG_FMT_FLAGS	; preserve BSR
	return



;;; Converts the lower nibble of W to its ASCII hexadecimal representation.
_w2hd	macro			; 'w to hex digit'
	andlw	b'00001111'
	addlw	(256-10)	; is W >= 10?
	skpnc
	addlw	7		; if so, shift to letters
	addlw	'A'-7		; shift to printable ASCII
	endm
_uart_print_hex
	movfw	LOG_CURR_BYTE
	swapf	WREG,w		; get high nibble
	_w2hd
	call	_uart_print_ch	; print high nibble
	movfw	LOG_CURR_BYTE	; bring back original byte
	_w2hd
	goto	_uart_print_ch	; print low nibble

