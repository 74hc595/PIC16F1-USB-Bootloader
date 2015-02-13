; vim:noet:sw=8:ts=8:ai:syn=pic
;
; USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; Macros for log.asm.

; Begins a hex byte sequence; the lower 5 bits specify how many of the
; following bytes should be printed in hexadecimal.
LOG_HEX		equ	b'10000000'
; The given character or hex byte sequence should be followed by a newline.
LOG_NEWLINE	equ	b'01000000'
; The current hex byte sequence should print a space before each byte.
LOG_SPACE	equ	b'00100000'

; Logs a single ASCII character in the range 32-127.
logch	macro	ch,opts
	if	LOGGING_ENABLED
	movlw	((((ch-32) & b'00111111') | opts) & b'01111111')
	call	log_single_byte
	endif
	endm

; Logs a marker indicating that the next `count` bytes should be
; printed in hexadecimal.
loghex	macro	count,opts
	if	LOGGING_ENABLED
	movlw	(count & b'00011111') | (opts & b'01100000') | LOG_HEX
	call	log_single_byte
	endif
	endm

; Logs a literal by first copying it to W.
; This will *not* work properly for ASCII characters.
logl	macro	x
	if	LOGGING_ENABLED
	movlw	x
	call	log_single_byte
	endif
	endm

; Logs the byte in W.
logw	macro
	if	LOGGING_ENABLED
	call	log_single_byte
	endif
	endm

; Logs the byte in the given file register.
; The appropriate bank is selected and the byte is copied to W.
logf	macro	x
	if	LOGGING_ENABLED
	banksel	x
	movfw	x
	call	log_single_byte
	endif
	endm



; Begins a multi-character logging sequence.
; Saves FSR0; restore it with logend.
mlog	macro
	if	LOGGING_ENABLED
	call	log_multi_byte_start
	endif
	endm

; Logs the given ASCII character in the range 32-127 as part of a
; multi-character logging sequence.
; Must be called between mlog/mlogend, or FSR0 will be clobbered.
mlogch	macro	ch,opts
	if	LOGGING_ENABLED
	movlw	((((ch-32) & b'00111111') | opts) & b'01111111')	
	call	log_byte
	endif
	endm

; Logs a marker indicating that the next `count` bytes should be
; printed in hexadecimal.
; Must be called between mlog/logend. or FSR0 will be clobbered.
mloghex	macro	count,opts
	if	LOGGING_ENABLED
	movlw	(count & b'00011111') | (opts & b'01100000') | LOG_HEX
	call	log_byte
	endif
	endm

; Logs a literal as part of a multi-character logging sequence.
; The literal is copied to W.
; This will *not* work properly for ASCII characters.
; Must be called between mlog/logend. or FSR0 will be clobbered.
mlogl	macro	x
	if	LOGGING_ENABLED
	movlw	x
	call	log_single_byte
	endif
	endm

; Logs the byte in W as part of a multi-character logging sequence.
; Must be called between mlog/logend. or FSR0 will be clobbered.
mlogw	macro
	if	LOGGING_ENABLED
	call	log_byte
	endif
	endm

; Logs the byte in the given file register as part of a multi-character logging sequence.
; The appropriate bank is selected and the byte is copied to W.
; Must be called between mlog/logend, or FSR0 will be clobbered.
mlogf	macro	x
	if	LOGGING_ENABLED
	banksel	x
	movfw	x
	call	log_byte
	endif
	endm

; Ends a multi-character logging sequence.
; Restores FSR0 to the value it had when mlog was called.
mlogend	macro
	if	LOGGING_ENABLED
	call	log_multi_byte_end
	endif
	endm

; Bank-select macro that's not assembled if logging is disabled.
lbnksel	macro	x
	if	LOGGING_ENABLED
	banksel	x
	endif
	endm
