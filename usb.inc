; vim:noet:sw=8:ts=8:ai:syn=pic

; USB PID values
PID_OUT		equ	(0x1<<PID0)
PID_ACK		equ	(0x2<<PID0)
PID_DATA0	equ	(0x3<<PID0)
PID_PING	equ	(0x4<<PID0)
PID_SOF		equ	(0x5<<PID0)
PID_NYET	equ	(0x6<<PID0)
PID_DATA2	equ	(0x7<<PID0)
PID_SPLIT	equ	(0x8<<PID0)
PID_IN		equ	(0x9<<PID0)
PID_NAK		equ	(0xA<<PID0)
PID_DATA1	equ	(0xB<<PID0)
PID_PRE		equ	(0xC<<PID0)
PID_ERR		equ	(0xC<<PID0) ; same as PRE
PID_SETUP	equ	(0xD<<PID0)
PID_STALL	equ	(0xE<<PID0)
PID_MDATA	equ	(0xF<<PID0)

