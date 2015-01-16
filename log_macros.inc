; vim:noet:sw=8:ts=8:ai:syn=pic
;
; Logging macros

LOGGING_ENABLED	equ	1


; Begins a hex byte sequence; the lower 5 bits specify how many of the
; following bytes should be printed in hexadecimal.
LOG_HEX		equ	b'10000000'
; The given character or hex byte sequence should be followed by a newline.
LOG_NEWLINE	equ	b'01000000'
; The current hex byte sequence should print a space before each byte.
LOG_DELIM	equ	LOG_HEX|b'00100000'


; Logs a single 6-bit character.
logch	macro	ch,opts
	if	LOGGING_ENABLED
	movlw	(((ch & b'00111111') | opts) & b'01111111')
	call	log_single_byte
	endif
	endm

; Begins a multi-character logging sequence.
logseq	macro	x
	if	LOGGING_ENABLED
	movlw	x
	call	log_multi_byte_start
	endif
	endm

; Logs the byte in W as part of a multi-character logging sequence.
; Must be called between logseq/logend.
logw	macro
	if	LOGGING_ENABLED
	call	log_byte
	endif
	endm

; Logs a marker indicating that the next `count` bytes should be
; printed in hexadecimal.
loghex	macro	count,opts
	if	LOGGING_ENABLED
	movlw	(count & b'00011111') | (opts & b'01100000') | LOG_HEX
	endif
	endm

; Logs the given literal as part of a multi-character logging sequence.
; Must be called between logseq/logend.
logl	macro	x
	if	LOGGING_ENABLED
	movlw	x
	call	log_byte
	endif
	endm

; Logs the byte in the given file register as part of a multi-character logging sequence.
; The appropriate bank is selected and the byte is copied to W.
; Must be called between logseq/logend.
logf	macro	x
	if	LOGGING_ENABLED
	banksel	x
	movfw	x
	call	log_byte
	endif
	endm

; Ends a multi-character logging sequence.
logend	macro
	if	LOGGING_ENABLED
	call	log_multi_byte_end
	endif
	endm

