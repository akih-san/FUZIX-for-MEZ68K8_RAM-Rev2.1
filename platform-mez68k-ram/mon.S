/*
;;;
;;; Universal Monitor 68000
;;;   Copyright (C) 2021 Haruo Asano
;;;
;
;	page 0
;	CPU	68000
;	SUPMODE	ON
*/

	.globl	INIVEC
	.globl	UREQ_COM
	.globl	UNI_CHR
	.globl	STR_addr
	.globl	CREQ_COM
	.globl	CBI_CHR
	.globl	disk_drive
	.globl	disk_track
	.globl	disk_sector
	.globl	data_dma
	.globl	crd_cnt
	.globl	crd_rp
	.globl	crd_wp
	.globl	contmp_buf
	.globl	crd_buf
	.globl	INIT
	.globl	CSTART
	.globl	WSTART
	.globl	DUMP
	.globl	GO
	.globl	SETM
	.globl	LOADH
	.globl	SAVEH
	.globl	REG
	.globl	STROUT
	.globl	HEXOUT8
	.globl	HEXOUT4
	.globl	HEXOUT2
	.globl	HEXOUT1
	.globl	CRLF
	.globl	GETLIN
	.globl	IRQ5_H
	.globl	DUMMY_H
	.globl	NMI_H

.mri 1
;;;
;;; Memory
;;;

VECT_T:	EQU	$00000000
PROG_B:	EQU	$00000400
SHAD_M: EQU	$100		;PIC <-> 68008 Shared Memory

BUFLEN:	EQU	24		; Input buffer size
VECSIZ:	EQU	256		; Number of vectors to be initialized

MPU_SPEC	equ	0	; MPU Spec. (Effective when USE_IDENT=0) 0:MC68000/8 1:MC68010

;;;
;;; Common header file
;;;

;;; Constants
CR:	EQU	$0D
LF:	EQU	$0A
BS:	EQU	$08
DEL:	EQU	$7F

START_FUZIX	equ	$2004

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; define Vecter table
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	ORG	VECT_T

;sys_base	equ	$10008
sys_base	equ	$2008

spurious	equ	sys_base
unexpected	equ	sys_base+6
bus		equ	sys_base+12
addr		equ	sys_base+18
illegal		equ	sys_base+24
divzero		equ	sys_base+30
chk		equ	sys_base+36
trapv		equ	sys_base+42
priv		equ	sys_base+48
trace		equ	sys_base+54
unimpa		equ	sys_base+60
unimpf		equ	sys_base+66
uninit		equ	sys_base+72
interrupt	equ	sys_base+78
trap0		equ	sys_base+84
trap1		equ	sys_base+90
trap2		equ	sys_base+96
trap3		equ	sys_base+102
trap4		equ	sys_base+108
trap5		equ	sys_base+114
trap6		equ	sys_base+120
trap7		equ	sys_base+126
trap8		equ	sys_base+132
trap9		equ	sys_base+138
trap10		equ	sys_base+144
trap11		equ	sys_base+150
trap12		equ	sys_base+156
trap13		equ	sys_base+162
trap14		equ	sys_base+168
trap15		equ	sys_base+174

INIVEC:
	;; 0-7
	DC.L	STACK		; Reset: Initial SSP
	DC.L	CSTART		; Reset: Initial PC

	long	bus
	long	addr
	long	illegal
	long	divzero
	long	chk
	long	trapv
	long	priv
	long	trace
	long	unimpa
	long	unimpf
	long	unexpected
	long	unexpected
	long	unexpected
	long	uninit
	; 16-23 unused
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	; 24
	long	spurious
	long	unexpected
	long	unexpected
	long	unexpected
	long	unexpected
	long	IRQ5_H		; Level 5 Interrupt Autovector
	long	unexpected
	long	NMI_H		; Level 7 Interrupt Autovector
	; 32 - TRAP #n
	long	trap0
	long	trap1
	long	trap2
	long	trap3
	long	trap4
	long	trap5
	long	trap6
	long	trap7
	long	trap8
	long	trap9
	long	trap10
	long	trap11
	long	trap12			; System call
	long	trap13
	long	trap14
	long	trap15

INIVECE:
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; define shared memory area
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;	ORG	SHAD_M

	ds.b	$40

;----------------------------------------------------------------------------------
; request command to PIC
;----------------------------------------------------------------------------------
REQ_CONIN:	equ	1		; return char in UNI_CHR
REQ_CONOUT	equ	2		; UNI_CHR = output char
REQ_CONST:	equ	3		; return status in UNI_CHR( 0: no key, 1 : key exist )
REQ_STROUT	equ	4		; string output to console
					; UNI_CHR or CBI_CHR = string size.
					; STR_addr or data_dma = string address
REQ_CONIN1:	equ	9		; return char,(null char is no key)
REQ_CONOUT1:	equ	10		; return status,(0:output OK, 1:console busy)
REQ_STRIN	equ	11
REQ_CLR_IRQ:	equ	12		; clear IRQ interrupt request

PIC_REQ:	equ	$80000		;A19 = 1

;			   ; offset:
UREQ_COM:	DS.B	1	; 0: monitor CONIN/CONOUT request command
UNI_CHR:	DS.B	1	; 1: charcter (CONIN/CONOUT) or number of strings
STR_addr	DS.L	1	; 2: string address
;--------------------------------------------------------------------------------
CREQ_COM:	DS.B	1	; 6: PIC function CONIN/CONOUT request command
CBI_CHR:	DS.B	1	; 7: charcter or return status
disk_drive:	DS.B	1	; 8
blocks		DS.B	1	; 9
lba:		DS.L	1	; A,B,C,D
dma:		DS.L	1	; E,F,10,11

crd_cnt:	ds.b	1
crd_rp:		ds.b	1
crd_wp:		ds.b	1
irq5_flg:	ds.b	1
req_e:

;	org	(req_e+$80) & $ff80
	ds.b	((req_e+$80)&$ff80)-req_e

con_size:	equ	128
contmp_buf:	ds.b	con_size	; tmp string buffer
crd_buf:	ds.b	con_size	; console input buffer

shm_end:

;
; tty input output entry
;
ttyin_st:	jmp	con_st
tty_in:		jmp	con_in
tty_out:	jmp	con_out

entp_end:
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; unimon program
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;	ORG	$400
;	ORG	PROG_B
;	ds.b	$180
	ds.b	((entp_end+$180)&$ff80)-entp_end

CSTART:
	BSR	INIT

	MOVE.L	#DAT_E,D0

	MOVE.L	D0,DSADDR
	MOVE.L	D0,GADDR
	MOVE.L	D0,SADDR
	MOVE.B	#'S',HEXMOD
	MOVE.B	#MPU_SPEC,PSPEC

	;; Initialize register save area
	LEA	REG_B,A0
	MOVE	#(REG_E-REG_B)-1,D0
INIR0:
	CLR.B	(A0)+
	DBF	D0,INIR0
	MOVE.L	#STACK,REGSSP
	MOVE.L	#DAT_E,REGPC
;	MOVE.L	#USTACK,REGA7
;	move.l	#USTACK,a0
	lea	USTACK,a0
	MOVE.L	a0,USP
	
	jmp	START_FUZIX
	
WSTART:

	; set interrupt level 4

	lea	STACK,a7
	move.w	#$2400,sr
	
	move.b	#1,irq5_flg	;set FUZIX bypass flag
	LEA	PROMPT,A0
	BSR	STROUT
	BSR	GETLIN
	LEA	INBUF,A0
	BSR	SKIPSP
	MOVE.B	(A0),D0
	BSR	UPPER
	TST.B	D0
	BEQ	WSTART

	CMP.B	#'D',D0
	BEQ	DUMP
	CMP.B	#'G',D0
	BEQ	GO
	CMP.B	#'S',D0
	BEQ	SETM

	CMP.B	#'L',D0
	BEQ	LOADH
	CMP.B	#'P',D0
	BEQ	SAVEH

	CMP.B	#'R',D0
	BEQ	REG
ERR:
	LEA	ERRMSG,A0
	BSR	STROUT
	BRA	WSTART

;;;
;;; Dump memory
;;;

DUMP:
	ADDQ	#1,A0
	BSR	SKIPSP
	BSR	RDHEX		; 1st arg.
	TST	D2
	BNE	DP0
	;; No arg.
	BSR	SKIPSP
	TST.B	(A0)
	BNE	ERR
	MOVE.L	DSADDR,D0
	ADD.L	#128,D0
	MOVE.L	D0,DEADDR
	BRA	DPM

	;; 1st arg. found
DP0:
	MOVE.L	D1,DSADDR
	BSR	SKIPSP
	CMP.B	#',',(A0)
	BEQ	DP1
	TST.B	(A0)
	BNE	ERR
	;; No 2nd arg.
	ADD.L	#128,D1
	MOVE.L	D1,DEADDR
	BRA	DPM

DP1:
	ADDQ	#1,A0
	BSR	SKIPSP
	BSR	RDHEX
	BSR	SKIPSP
	TST	D2
	BEQ	ERR
	TST.B	(A0)
	BNE	ERR
	ADDQ.L	#1,D1
	MOVE.L	D1,DEADDR

	;; DUMP main
DPM:
	MOVE.L	DSADDR,D0
	AND.B	#$F0,D0
	MOVE.L	D0,A1
	CLR	D2		; DSTATE
DPM0:
	BSR	DPL
	BSR	con_st
	TST.B	D0
	BNE	DPM1
	CMP	#2,D2		; DSTATE
	BCS	DPM0
	MOVE.L	DEADDR,DSADDR
	BRA	WSTART
DPM1:
	MOVE.L	A1,DSADDR
	BSR	con_in
	BRA	WSTART

	;; DUMP line
DPL:
	MOVE.L	A1,D0
	BSR	HEXOUT8
	LEA	DSEP0,A0
	BSR	STROUT
	LEA	INBUF,A2
	MOVE	#15,D3
DPL0:
	BSR	DPB
	DBF	D3,DPL0

	LEA	DSEP1,A0
	BSR	STROUT

	LEA	INBUF,A2
	MOVE	#15,D3
DPL1:
	MOVE.B	(A2)+,D0
	CMP.B	#' ',D0
	BCS	DPL2
	CMP.B	#$7F,D0
	BCC	DPL2
	BSR	con_out
	BRA	DPL3
DPL2:
	MOVE.B	#'.',D0
	BSR	con_out
DPL3:
	DBF	D3,DPL1
	BRA	CRLF

	;; DUMP byte
DPB:
	MOVE.B	#' ',D0
	BSR	con_out
	TST	D2		; DSTATE
	BNE	DPB2
	;; Dump state 0
	CMP.L	DSADDR,A1
	BEQ	DPB1
	;; Still 0 or 2
DPB0:
	MOVE.B	#' ',D0
	BSR	con_out
	BSR	con_out
	MOVE.B	D0,(A2)+
	ADDQ	#1,A1
	RTS
	;; Found start address
DPB1:
	MOVE	#1,D2		; DSTATE
DPB2:
	CMP	#1,D2		; DSTATE
	BNE	DPB0
	;; Dump state 1
	MOVE.B	(A1)+,D0
	MOVE.B	D0,(A2)+
	BSR	HEXOUT2
	CMP.L	DEADDR,A1
	BEQ	DPB3
	RTS
	;; Found end address
DPB3:	
	MOVE	#2,D2		; DSTATE
	RTS

;;;
;;; GO address
;;;

GO:
	ADDQ	#1,A0
	BSR	SKIPSP
	BSR	RDHEX
	TST.B	(A0)
	BNE	ERR
	TST	D2
	BEQ	G0

	MOVE.L	D1,REGPC
G0:
	MOVE.L	REGSSP,D0
	AND.L	#$FFFFFFFE,D0
	MOVE.L	D0,A7

	MOVE.L	REGPC,-(A7)
	MOVE	REGSR,-(A7)

;	MOVE.L	REGA7,A0
;	MOVE	A0,USP

	MOVEM.L	REGD0,D0-D7/A0-A6
	clr.b	irq5_flg

	RTE
	
;;;
;;; SET memory
;;;

SETM:
	ADDQ	#1,A0
	BSR	SKIPSP
	BSR	RDHEX
	BSR	SKIPSP
	TST.B	(A0)
	BNE	ERR
	MOVE.L	D1,A1
	TST	D2
	BNE	SM0
	MOVE.L	SADDR,A1
SM0:
SM1:
	MOVE.L	A1,D0
	BSR	HEXOUT8
	LEA	DSEP1,A0
	BSR	STROUT
	MOVE.B	(A1),D0
	BSR	HEXOUT2
	MOVE.B	#' ',D0
	BSR	con_out
	BSR	GETLIN
	LEA	INBUF,A0
	BSR	SKIPSP
	MOVE.B	(A0),D0
	BNE	SM2
	;; Empty (Increment address)
	ADDQ	#1,A1
	MOVE.L	A1,SADDR
	BRA	SM1
SM2:
	CMP.B	#'-',D0
	BNE	SM3
	;; '-' (Decrement address)
	SUBQ	#1,A1
	MOVE.L	A1,SADDR
	BRA	SM1
SM3:
	CMP.B	#'.',D0
	BNE	SM4
	;; '.' (Quit)
	MOVE.L	A1,SADDR
	BRA	WSTART
SM4:
	BSR	RDHEX
	TST	D2
	BEQ	ERR
	MOVE.B	D1,(A1)+
	MOVE.L	A1,SADDR
	BRA	SM1

;;;
;;; LOAD HEX file
;;;

LOADH:
	ADDQ	#1,A0
	BSR	SKIPSP
	BSR	RDHEX
	BSR	SKIPSP
	TST.B	(A0)
	BNE	ERR

	TST	D2
	BNE	LH0

	CLR.L	D1		; Offset
LH0:
	BSR	con_in
	BSR	UPPER
	CMP.B	#'S',D0
	BEQ	LHS0
LH1:
	CMP.B	#':',D0
	BEQ	LHI0
	;; Skip to EOL
LH2:
	CMP.B	#CR,D0
	BEQ	LH0
	CMP.B	#LF,D0
	BEQ	LH0
LH3:
	BSR	con_in
	BRA	LH2

	;; Intel HEX
LHI0:
	BSR	HEXIN
	CLR	D2
	MOVE.B	D0,D2		; Length
	MOVE.B	D0,D3		; Checksum

	BSR	HEXIN
	CLR.L	D4
	MOVE.B	D0,D4		; Address H
	ADD.B	D0,D3		; Checksum

	BSR	HEXIN
	LSL	#8,D4
	MOVE.B	D0,D4		; Address L
	ADD.B	D0,D3		; Checksum

	;; Add offset
	ADD.L	D1,D4
	MOVE.L	D4,A1

	BSR	HEXIN
	MOVE.B	D0,RECTYP
	ADD.B	D0,D3		; Checksum

	TST	D2
	BEQ	LHI3
	SUBQ	#1,D2
LHI1:
	BSR	HEXIN
	ADD.B	D0,D3		; Checksum

	TST.B	RECTYP
	BNE	LHI2

	MOVE.B	D0,(A1)+
LHI2:
	DBF	D2,LHI1
LHI3:
	BSR	HEXIN
	ADD.B	D0,D3		; Checksum
	BNE	LHIE		; Checksum error
	TST.B	RECTYP
	BEQ	LH3
	BRA	WSTART
LHIE:
	LEA	IHEMSG,A0
	BSR	STROUT
	BRA	WSTART

	;; Motorola S record
LHS0:
	BSR	con_in
	MOVE.B	D0,RECTYP

	BSR	HEXIN
	CLR	D2
	MOVE.B	D0,D2		; Length+3
	MOVE.B	D0,D3		; Checksum

	BSR	HEXIN
	CLR.L	D4
	MOVE.B	D0,D4		; Address H
	ADD.B	D0,D3		; Checksum

	BSR	HEXIN
	LSL.L	#8,D4
	MOVE.B	D0,D4		; Address L
	ADD.B	D0,D3		; Checksum

	;; Add offset
	ADD.L	D1,D4
	MOVE.L	D4,A1

	SUBQ	#3,D2
	BEQ	LHS3
	SUBQ	#1,D2
LHS1:
	BSR	HEXIN
	ADD.B	D0,D3		; Checksum

	CMP.B	#'1',RECTYP
	BNE	LHS2

	MOVE.B	D0,(A1)+
LHS2:
	DBF	D2,LHS1
LHS3:
	BSR	HEXIN
	ADD.B	D0,D3		; Checksum
	CMP.B	#$FF,D3
	BNE	LHSE		; Checksum error

	CMP.B	#'9',RECTYP
	BEQ	LHSR
	BRA	LH3
LHSE:
	LEA	SHEMSG,A0
	BSR	STROUT
LHSR:
	BRA	WSTART

;;;
;;;  SAVE HEX file
;;;

SAVEH:
	ADDQ	#1,A0
	MOVE.B	(A0),D0
	BSR	UPPER
	CMP.B	#'I',D0
	BEQ	SH0
	CMP.B	#'S',D0
	BNE	SH1
SH0:
	ADDQ	#1,A0
	MOVE.B	D0,HEXMOD
SH1:
	BSR	SKIPSP
	BSR	RDHEX
	TST	D2
	BEQ	ERR
	MOVE.L	D1,A1		; Start address
	BSR	SKIPSP
	CMP.B	#',',(A0)+
	BNE	ERR
	BSR	SKIPSP
	BSR	RDHEX		; D1 = End address
	TST	D2
	BEQ	ERR
	BSR	SKIPSP
	TST.B	(A0)
	BNE	ERR

SH2:
	SUB.L	A1,D1
	ADDQ	#1,D1		; D1 = Length
SH3:
	BSR	SHL
	TST.L	D1
	BNE	SH3

	CMP.B	#'I',HEXMOD
	BNE	SH4
	;; End record for Intel HEX
	LEA	IHEXER,A0
	BSR	STROUT
	BRA	WSTART
	;; End record for Motorola S record
SH4:
	LEA	SRECER,A0
	BSR	STROUT
	BRA	WSTART

SHL:
	MOVEQ.L	#16,D2
	CMP.L	D2,D1
	BCC	SHL0
	MOVE.L	D1,D2
SHL0:
	SUB.L	D2,D1		; D1 = remain

	CMP.B	#'I',HEXMOD
	BNE	SHLS

	;; Intel HEX
	MOVE.B	#':',D0
	BSR	con_out

	MOVE.B	D2,D0
	MOVE.B	D2,D3		; Checksum
	BSR	HEXOUT2		; Length

	MOVE.L	A1,D0
	LSR	#8,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Address H

	MOVE.L	A1,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Address L

	CLR.B	D0
	BSR	HEXOUT2		; Record type

	SUBQ	#1,D2
SHLI0:
	MOVE.B	(A1)+,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2

	DBF	D2,SHLI0

	MOVE.B	D3,D0
	NEG.B	D0
	BSR	HEXOUT2		; Checksum
	BRA	CRLF

	;; Motorola S record
SHLS:	
	MOVE.B	#'S',D0
	BSR	con_out
	MOVE.B	#'1',D0
	BSR	con_out

	MOVE.B	D2,D0
	ADDQ.B	#2+1,D0		; DataLength + 2(Addr) + 1(Sum)
	MOVE.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Length

	MOVE.L	A1,D0
	LSR.L	#8,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Address H

	MOVE.L	A1,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Address L

	SUBQ	#1,D2
SHLS0:
	MOVE.B	(A1)+,D0
	ADD.B	D0,D3		; Checksum
	BSR	HEXOUT2		; Data

	DBF	D2,SHLS0

	MOVE.B	D3,D0
	NOT.B	D0
	BSR	HEXOUT2
	BRA	CRLF

;;;
;;; Register
;;;
REG:
	ADDQ	#1,A0
	BSR	SKIPSP
	MOVE.B	(A0),D0
	BSR	UPPER
	TST.B	D0
	BNE	RG0
	BSR	RDUMP
	BRA	WSTART
RG0:
	LEA	RNTAB,A1
RG1:
	CMP.B	(A1),D0
	BEQ	RG2		; Character match
	TST.B	1(A1)
	BEQ	RGE
	ADD.L	#10,A1
	BRA	RG1
RG2:
	CMP.B	#$0F,1(A1)
	BNE	RG3
	;; Next table
	MOVE.L	2(A1),A1
	ADDQ	#1,A0
	MOVE.B	(A0),D0
	BSR	UPPER
	BRA	RG1
RG3:	
	MOVE.B	1(A1),D3
	BEQ	RGE		; Found end mark
	BPL	RG30
	;; Check MC68010
	TST.B	PSPEC
	BEQ	RGE
RG30:
	MOVE.L	6(A1),A0
	BSR	STROUT
	MOVE.B	#'=',D0
	BSR	con_out
	MOVE.L	2(A1),A2
	AND.B	#$07,D3
	CMP.B	#1,D3
	BNE	RG4
	;; 8 bit register
	MOVE.B	(A2),D0
	BSR	HEXOUT2
	BRA	RG6
RG4:
	CMP.B	#2,D3
	BNE	RG5
	;; 16 bit register
	MOVE	(A2),D0
	BSR	HEXOUT4
	BRA	RG6
RG5:
	;; 32 bit register
	MOVE.L	(A2),D0
	BSR	HEXOUT8
RG6:
	MOVE.B	#' ',D0
	BSR	con_out
	BSR	GETLIN
	LEA	INBUF,A0
	BSR	SKIPSP
	BSR	RDHEX
	TST	D2
	BEQ	RGR
	CMP.B	#1,D3
	BNE	RG7
	;; 8 bit register
	MOVE.B	D1,(A2)
	BRA	RG9
RG7:
	CMP.B	#2,D3
	BNE	RG8
	;; 16 bit register
	MOVE	D1,(A2)
	BRA	RG9
RG8:
	;; 32 bit register
	MOVE.L	D1,(A2)
RG9:
RGR:
	BRA	WSTART
RGE:
	BRA	ERR
	
RDUMP:
	LEA	RDTAB,A1
RD0:
	MOVE	(A1)+,D1	; Flag
	BEQ	CRLF		; Found END mark => CR,LF and return
	BPL	RD00
	TST.B	PSPEC
	BEQ	CRLF		; 
RD00:	
	MOVE.L	(A1)+,A0	; String address
	BSR	STROUT
	MOVE.L	(A1)+,A0	; Register save area
	AND	#$0003,D1
	CMP	#1,D1
	BNE	RD1
	;; BYTE size
	MOVE.B	(A0),D0
	BSR	HEXOUT2
	BRA	RD0
RD1:
	CMP	#2,D1
	BNE	RD2
	;; WORD size
	MOVE	(A0),D0
	BSR	HEXOUT4
	BRA	RD0
RD2:
	;; LONG size
	MOVE.L	(A0),D0
	BSR	HEXOUT8
	BRA	RD0

;;;
;;; Other support routines
;;;

STROUT:
	MOVE.B	(A0)+,D0
	TST.B	D0
	BEQ	STROE
	BSR	con_out
	BRA	STROUT
STROE:
	RTS

HEXOUT8:
	MOVE.L	D0,-(A7)
	SWAP	D0
	BSR	HEXOUT4
	MOVE.L	(A7)+,D0
HEXOUT4:
	MOVE.L	D0,-(A7)
	ROR	#8,D0
	BSR	HEXOUT2
	MOVE.L	(A7)+,D0
HEXOUT2:
	MOVE.L	D0,-(A7)
	ROR.B	#4,D0
	BSR	HEXOUT1
	MOVE.L	(A7)+,D0
HEXOUT1:
	AND.B	#$0F,D0
	ADD.B	#'0',D0
	CMP.B	#'9'+1,D0
	BCS	con_out
	ADD	#'A'-'9'-1,D0
	BRA	con_out

HEXIN:
	CLR.B	D0
	BSR	HI0
	LSL.B	#4,D0
HI0:
	MOVE	D0,-(A7)
	BSR	con_in
	BSR	UPPER
	CMP.B	#'0',D0
	BCS	HIR
	CMP.B	#'9'+1,D0
	BCS	HI1
	CMP.B	#'A',D0
	BCS	HIR
	CMP.B	#'F'+1,D0
	BCC	HIR
	SUB.B	#'A'-'9'-1,D0
HI1:
	SUB.B	#'0',D0
	OR	(A7),D0
HIR:
	ADDQ	#2,A7
	RTS

CRLF:
	MOVE.B	#CR,D0
	BSR	con_out
	MOVE.B	#LF,D0
	BRA	con_out

GETLIN:
	LEA	INBUF,A0
	CLR.B	D1
GL0:
	BSR	con_in
	CMP.B	#CR,D0
	BEQ	GLE
	CMP.B	#LF,D0
	BEQ	GLE
	CMP.B	#BS,D0
	BEQ	GLB
	CMP.B	#DEL,D0
	BEQ	GLB
	CMP.B	#' ',D0
	BCS	GL0
	CMP.B	#$80,D0
	BCC	GL0
	CMP.B	#BUFLEN-1,D1
	BCC	GL0		; Too long
	ADDQ.B	#1,D1
	MOVE.B	D0,(A0)+
	BSR	con_out
	BRA	GL0
GLB:
	TST.B	D1
	BEQ	GL0
	SUBQ.L	#1,A0
	SUBQ.B	#1,D1
	MOVE.B	#BS,D0
	BSR	con_out
	MOVE.B	#' ',D0
	BSR	con_out
	MOVE.B	#BS,D0
	BSR	con_out
	BRA	GL0
GLE:
	BSR	CRLF
	MOVE.B	#$00,(A0)
	RTS

SKIPSP:
	CMP.B	#' ',(A0)+
	BEQ	SKIPSP
	SUBQ.L	#1,A0
	RTS

UPPER:
	CMP.B	#'a',D0
	BCS	UPR
	CMP.B	#'z'+1,D0
	BCC	UPR
	ADD.B	#'A'-'a',D0
UPR:
	RTS

RDHEX:
	CLR	D2		; Count
	CLR.L	D1		; Value
RH0:
	MOVE.B	(A0),D0
	BSR	UPPER
	CMP.B	#'0',D0
	BCS	RHE
	CMP.B	#'9'+1,D0
	BCS	RH1
	CMP.B	#'A',D0
	BCS	RHE
	CMP.B	#'F'+1,D0
	BCC	RHE
	SUB.B	#'A'-'9'-1,D0
RH1:
	SUB.B	#'0',D0
	LSL.L	#4,D1
	OR.B	D0,D1
	ADDQ	#1,A0
	ADDQ	#1,D2
	BRA	RH0
RHE:
	RTS

;;;
;;; Exception Handler
;;;

;
; timer interrupt handler
;contmp_buf	ds.b	con_size	; tmp string buffer

IRQ5_H:
	movem.l	d0-d3/a0-a1,-(a7)	;push d0,d1,a0,a1
	cmp.b	#$80,crd_cnt
	beq	exit_cin	; buffer full, then ignore key data

	move.l	#contmp_buf,STR_addr	; save temp buffer address

	; change change big endian to little endian
	lea	STR_addr,a0
	move.b	(a0),d0		;hh
	move.b	1(a0),d1		;hl
	move.b	2(a0),d2		;lh
	move.b	3(a0),d3		;ll
	move.b	d3,(a0)
	move.b	d2,1(a0)
	move.b	d1,2(a0)
	move.b	d0,3(a0)

	move.b	#$80,d0
	sub.b	crd_cnt,d0		; d0 = #$80 - crd_cnt
	move.b	d0,UNI_CHR		; set read buffer size
	move.b	#REQ_STRIN,UREQ_COM	; set string input request
	move.b	PIC_REQ,d0		; request STRIN to PIC. Memory $80000 read
wa_pic:
	move.b	UREQ_COM,d0
	bne	wa_pic

	move.b	UNI_CHR,d0		; get string count
	beq	exit_cin		; exit, if no data

	lea	crd_buf,a0		; a0 = crd_buf
	lea	contmp_buf,a1		; a1 = contmp_buf
	clr.l	d1
	move.b	crd_wp,d1		; d1 = crd_wp

mv_str:
	move.b	(a1),0(a0,d1)	; crd_buf[d1] <- (a1)++
	addq	#1,a1
	addq.b	#1,d1			; crd_wp++
	andi.b	#$7f,d1
	addq.b	#1,crd_cnt		; crd_cnt++
	subq.b	#1,d0			; decriment string count
	bne	mv_str
	move.b	d1,crd_wp		; update write point

exit_cin:
	moveq	#1,d0
	bsr	clr_int			; clear interrupt request
	movem.l	(a7)+,d0-d3/a0-a1	 ;pop a1,a0,d1,d0

;
; jump FIZIX interrupt program
;
	tst.b	irq5_flg
	bne		DUMMY_H

	jmp	interrupt

	;; Dummy
DUMMY_H:
	RTE
;
; NMI interrupt
;
NMI_H:
	MOVEM.L	D0-D7/A0-A6,REGD0
	move.w	(a7)+,d0		; get status register
	move.w	d0,REGSR
	move.l	(a7)+,d0		; get PC
	move.l	d0,REGPC

	move.l	a7,REGSSP		; save SSP
	move.l	a7,REGA7

	LEA	NMI_MSG,A0
	BSR	STROUT
	moveq	#0,d0
	bsr	clr_int			; clear interrupt request
	bra	WSTART

	ALIGN	2
NMI_MSG:
	byte	CR,LF
	ascii	"NMI Interrupt!!"
	byte	CR,LF,$00

clr_int:
	move.b	d0,UNI_CHR		; set interrupt source(0: NMI 1:IRQ5)
	move.b	#REQ_CLR_IRQ,UREQ_COM	; set clear interrupt request
	move.b	PIC_REQ,d0		; request conin to PIC. Memory $80000 read
w_pic:
	move.b	UREQ_COM,d0
	bne	w_pic
	rts

;;; DUMP group 0 (Address/Bus Error)
G0DUMP:
	;; Function Code
	MOVE	GR0BUF,D0
	AND	#$0007,D0
	LSL	#2,D0
	LEA	FCTAB,A0
	MOVE.L	0(A0,D3.W),A0
	BSR	STROUT

	;; Read / Write
	MOVE	GR0BUF,D0
	AND	#$0010,D0
	BNE	G0D00
	LEA	G0WR,A0
	BRA	G0D01
G0D00:
	LEA	G0RD,A0
G0D01:
	BSR	STROUT

	;; Address
	MOVE.L	GR0BUF+2,D0
	BSR	HEXOUT8		; Access address

	;; Instruction Register
	LEA	G0INS0,A0
	BSR	STROUT
	MOVE	GR0BUF+6,D0
	BSR	HEXOUT4		; Instruction register
	MOVE.B	#')',D0
	BSR	con_out

	BRA	CRLF

;;;
;;; Data area
;;; 
	
	ALIGN	2
PROMPT:	ascii	"] "
	byte	$00

	ALIGN	2
IHEMSG:	ascii	"Error ihex"
	byte	CR,LF,$00
	ALIGN	2
SHEMSG:	ascii	"Error srec"
	byte	CR,LF,$00
	ALIGN	2
ERRMSG:	ascii	"Error"
	byte	CR,LF,$00

	ALIGN	2
DSEP0:	ascii	" :"
	byte	$00
	ALIGN	2
DSEP1:	ascii	" : "
	byte	$00
	ALIGN	2
IHEXER:	ascii	":00000001FF"
	byte	CR,LF,$00
	ALIGN	2
SRECER: ascii	"S9030000FC"
	byte	CR,LF,$00

	ALIGN	2
RDTAB:	DC.W	$0003		; LONG
	DC.L	RDSD07,REGD0
	DC.W	$0003
	DC.L	RDSC,REGD1
	DC.W	$0003
	DC.L	RDSC,REGD2
	DC.W	$0003
	DC.L	RDSC,REGD3
	DC.W	$0003
	DC.L	RDSCS,REGD4
	DC.W	$0003
	DC.L	RDSC,REGD5
	DC.W	$0003
	DC.L	RDSC,REGD6
	DC.W	$0003
	DC.L	RDSC,REGD7

	DC.W	$0003
	DC.L	RDSA07,REGA0
	DC.W	$0003
	DC.L	RDSC,REGA1
	DC.W	$0003
	DC.L	RDSC,REGA2
	DC.W	$0003
	DC.L	RDSC,REGA3
	DC.W	$0003
	DC.L	RDSCS,REGA4
	DC.W	$0003
	DC.L	RDSC,REGA5
	DC.W	$0003
	DC.L	RDSC,REGA6
	DC.W	$0003
	DC.L	RDSC,REGA7

	DC.W	$0003
	DC.L	RDSPC,REGPC
	DC.W	$0003
	DC.L	RDSSSP,REGSSP
	DC.W	$0002		; WORD
	DC.L	RDSSR,REGSR

	DC.W	$8003		; LONG + (END flag for MC68000/8)
	DC.L	RDSVBR,REGVBR
	DC.W	$8001		; BYTE
	DC.L	RDSSFC,REGSFC
	DC.W	$8001		; BYTE
	DC.L	RDSDFC,REGDFC	
	
	DC.W	$0000		; END

	ALIGN	2
RDSD07:	ascii	"D0-D7="
	byte	$00
	ALIGN	2
RDSA07:	byte	CR,LF
	ascii	"A0-A7="
	byte	$00
	ALIGN	2
RDSPC:	byte	CR,LF
	ascii	"PC="
	byte	$00
	ALIGN	2
RDSSSP:	ascii	" SSP="
	byte	$00
	ALIGN	2
RDSSR:	ascii	" SR="
	byte	$00
	ALIGN	2
RDSVBR:	ascii	"  VBR="
	byte	$00
	ALIGN	2
RDSSFC:	ascii	" SFC="
	byte	$00
	ALIGN	2
RDSDFC:	ascii	" DFC="
	byte	$00
	ALIGN	2
RDSC:	ascii	","
	byte	$00
	ALIGN	2
RDSCS:	ascii	", "
	byte	$00

	ALIGN	2
RNTAB:
	DC.B	'A',$0F		; "A?"
	DC.L	RNTABA,0
	DC.B	'C',$0F		; "C?"
	DC.L	RNTABC,0
	DC.B	'D',$0F		; "D?"
	DC.L	RNTABD,0
	DC.B	'P',$0F		; "P?"
	DC.L	RNTABP,0
	DC.B	'S',$0F		; "S?"
	DC.L	RNTABS,0
	DC.B	'V',$0F		; "V?"
	DC.L	RNTABV,0

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABA:
	DC.B	'0',3		; "A0"
	DC.L	REGA0,RNA0
	DC.B	'1',3		; "A1"
	DC.L	REGA1,RNA1
	DC.B	'2',3		; "A2"
	DC.L	REGA2,RNA2
	DC.B	'3',3		; "A3"
	DC.L	REGA3,RNA3
	DC.B	'4',3		; "A4"
	DC.L	REGA4,RNA4
	DC.B	'5',3		; "A5"
	DC.L	REGA5,RNA5
	DC.B	'6',3		; "A6"
	DC.L	REGA6,RNA6
	DC.B	'7',3		; "A7"
	DC.L	REGA7,RNA7

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABC:
	DC.B	'C',$0F		; "CC"
	DC.L	RNTABCC,0
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABD:
	DC.B	'0',3		; "D0"
	DC.L	REGD0,RND0
	DC.B	'1',3		; "D1"
	DC.L	REGD1,RND1
	DC.B	'2',3		; "D2"
	DC.L	REGD2,RND2
	DC.B	'3',3		; "D3"
	DC.L	REGD3,RND3
	DC.B	'4',3		; "D4"
	DC.L	REGD4,RND4
	DC.B	'5',3		; "D5"
	DC.L	REGD5,RND5
	DC.B	'6',3		; "D6"
	DC.L	REGD6,RND6
	DC.B	'7',3		; "D7"
	DC.L	REGD7,RND7
	DC.B	'F',$0F		; "DF?"
	DC.L	RNTABDF,0

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABP:
	DC.B	'C',3		; "PC"
	DC.L	REGPC,RNPC

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABS:
	DC.B	'F',$0F		; "SF?"
	DC.L	RNTABSF,0
	DC.B	'R',2		; "SR"
	DC.L	REGSR,RNSR
	DC.B	'S',$0F		; "SS?"
	DC.L	RNTABSS,0

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABV:
	DC.B	'B',$0F		; "VB?"
	DC.L	RNTABVB,0

	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABCC:
	DC.B	'R',1		; "CCR"
	DC.L	REGSR+1,RNCCR
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABDF:
	DC.B	'C',$81		; "DFC"
	DC.L	REGDFC,RNDFC
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABSF:
	DC.B	'C',$81		; "SFC"
	DC.L	REGSFC,RNSFC
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABSS:
	DC.B	'P',3		; "SSP"
	DC.L	REGSSP,RNSSP
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNTABVB:
	DC.B	'R',$83		; "VBR"
	DC.L	REGVBR,RNVBR
	
	DC.B	$00,$00		; End mark
	DC.L	0,0

RNA0:	ascii	"A0"
	byte	$00
RNA1:	ascii	"A1"
	byte	$00
RNA2:	ascii	"A2"
	byte	$00
RNA3:	ascii	"A3"
	byte	$00
RNA4:	ascii	"A4"
	byte	$00
RNA5:	ascii	"A5"
	byte	$00
RNA6:	ascii	"A6"
	byte	$00
RNA7:	ascii	"A7"
	byte	$00
RNCCR:	ascii	"CCR"
	byte	$00
RND0:	ascii	"D0"
	byte	$00
RND1:	ascii	"D1"
	byte	$00
RND2:	ascii	"D2"
	byte	$00
RND3:	ascii	"D3"
	byte	$00
RND4:	ascii	"D4"
	byte	$00
RND5:	ascii	"D5"
	byte	$00
RND6:	ascii	"D6"
	byte	$00
RND7:	ascii	"D7"
	byte	$00
RNDFC:	ascii	"DFC"
	byte	$00
RNPC:	ascii	"PC"
	byte	$00
RNSFC:	ascii	"SFC"
	byte	$00
RNSR:	ascii	"SR"
	byte	$00
RNSSP:	ascii	"SSP"
	byte	$00
RNVBR:	ascii	"VBR"
	byte	$00

FCTAB:	DC.L	FCN0,FCN1,FCN2,FCN3
	DC.L	FCN4,FCN5,FCN6,FCN7
FCTAB2:	DC.L	FCN0,FCN1,FCN2,FCN3
	DC.L	FCN4,FCN5,FCN6,FCN7A
FCN0:	ascii	"FC=0 "
	byte	$00
FCN1:	ascii	"User Data "
	byte	$00
FCN2:	ascii	"User Program "
	byte	$00
FCN3:	ascii	"FC=3 "
	byte	$00
FCN4:	ascii	"FC=4 "
	byte	$00
FCN5:	ascii	"Supervisor Data "
	byte	$00
FCN6:	ascii	"Supervisor Program "
	byte	$00
FCN7:	ascii	"Interrupt Acknowledge "
	byte	$00
FCN7A:	ascii	"CPU "
	byte	$00
G0WR:	ascii	"Write "
	byte	$00
G0RD:	ascii	"Read "
	byte	$00
G0INS0:	ascii	"  (Inst="
	byte	$00
G0BYTE:	ascii	"Byte "
	byte	$00
G0WORD:	ascii	"Word "
	byte	$00
G0TO:	ascii	" to "
	byte	$00
G0FROM:	ascii	"from "
	byte	$00
G0SPC:	ascii	"Space "
	byte	$00
G0IF:	ascii	" (Instruction Fetch)"
	byte	$00
G0RMW:	ascii	" (Read Modify Write)"
	byte	$00

	;; Stack frame size (WORD) for each format
FMTLEN:
	DC.B	4		; 0000 (68010,68020,SCC68070)
	DC.B	4		; 0001 Throw Away (68020)
	DC.B	6		; 0010 (68020)
	DC.B	6		; 0011
	DC.B	8		; 0100
	DC.B	0		; 0101
	DC.B	0		; 0110
	DC.B	30		; 0111
	DC.B	29		; 1000 Bus Error, Address Error (68010)
	DC.B	10		; 1001 Coprocessor Midinstruction (68020)
	DC.B	16		; 1010 Short Bus Fault (68020,68030)
	DC.B	46		; 1011 Long Bus Fault (68020,68030)
	DC.B	12		; 1100 Bus Error (CPU32)
	DC.B	0		; 1101
	DC.B	0		; 1110
	DC.B	17		; 1111 (SCC68070)


	ALIGN	2


;;;
;;;	UART Console Driver
;;;

INIT:
	;; Initialize PIC command request table
	;; and clear console input buffer control variables
	LEA	SHAD_M,A0
	MOVE	#(req_e-SHAD_M)-1,D0
shad_0:
	CLR.B	(A0)+
	DBF	D0,shad_0
	RTS

con_st:
	move.b	crd_cnt,d0
	rts

; output d0 : conin character
con_in:
	tst.b	crd_cnt
	beq.s	con_in

	; disable interrupt
	move sr,-(a7)		; save status register
	or.w #$0700,sr		; disable interrupt

	movem.l	d1/a0,-(a7)	;push d1,a0

	lea	crd_buf,a0	; a0 = crd_buf
	clr.l	d1
	move.b	crd_rp,d1	; d1 = crd_rp
	move.b	0(a0,d1),d0	; d0 <- crd_buf[d1]

	addq.b	#1,d1		; crd_rp++
	andi.b	#$7f,d1
	move.b	d1,crd_rp

	subq.b	#1,crd_cnt

	movem.l	(a7)+,d1/a0	 ;pop a0,d1

	; enable interrupt
	move	(a7)+,sr
	rts

; input d0 : conout character
con_out:
	movem.l	d0/d1,-(a7)	;push d0,d1
	move.b	d0,d1
cout_1:
	move sr,-(a7)		; save status register
	or.w #$0700,sr		; disable interrupt

	bsr	CONOUT
	move	(a7)+,sr

	or.b	d0,d0
	beq.s	end_cout
	move.b	d1,d0
	bra	cout_1

end_cout:
	movem.l	(a7)+,d0/d1	;pop d1, d0
	rts

;----------------------------------------------------------------------------------
;	ORG	SHAD_M
;
;			   ; offset:
;UREQ_COM:	DS.B	1	; 0: monitor CONIN/CONOUT request command
;UNI_CHR:	DS.B	1	; 1: charcter (CONIN/CONOUT) or number of strings
;STR_addr	DS.L	1	; 2: string address
;--------------------------------------------------------------------------------
;CREQ_COM:	DS.B	1	; 6: PIC function CONIN/CONOUT request command
;CBI_CHR:	DS.B	1	; 7: charcter or return status
;disk_drive:	DS.B	1	; 8
;blocks		DS.B	1	; 9
;lba:		DS.L	1	; A,B,C,D
;dma:		DS.L	1	; E,F,10,11
;req_e:

CONIN:
	move.b	#REQ_CONIN1,UREQ_COM	; set CONIB request

cin0:
	move.b	PIC_REQ,d0		; request conin to PIC. Memory $80000 read
cin1:
	move.b	UREQ_COM,d0
;	or.b	d0,d0
	bne	cin1
	
	; get character or status
	move.b	UNI_CHR,d0
	RTS

CONOUT:
	move.b	#REQ_CONOUT1,UREQ_COM	; set CONOUT request
	move.b	d0,UNI_CHR		; set output character
	bra	cin0

ROM_E:

;;;
;;; Work area
;;;


;WORK_B	EQU	(ROM_E+16) & $fffffff0
;
;	ORG	WORK_B
	ds.b	((ROM_E+16)&$fff0)-ROM_E


INBUF:	DS.B	BUFLEN		; Line input buffer
DSADDR:	DS.L	1		; DUMP start address
DEADDR:	DS.L	1
DSTATE:	DS.B	1
	ALIGN	2
GADDR:	DS.L	1
SADDR:	DS.L	1		; SET address
HEXMOD:	DS.B	1		; HEX file mode
RECTYP:	DS.B	1		; Record type
PSPEC:	DS.B	1		; Processor spec.
	ALIGN	2

REG_B:
REGPC:	DS.L	1

REGD0:	DS.L	1
REGD1:	DS.L	1
REGD2:	DS.L	1
REGD3:	DS.L	1
REGD4:	DS.L	1
REGD5:	DS.L	1
REGD6:	DS.L	1
REGD7:	DS.L	1

REGA0:	DS.L	1
REGA1:	DS.L	1
REGA2:	DS.L	1
REGA3:	DS.L	1
REGA4:	DS.L	1
REGA5:	DS.L	1
REGA6:	DS.L	1
REGA7:	DS.L	1		; USP

REGSSP:	DS.L	1
REGSR:	DS.W	1
REGVBR:	DS.L	1
REGSFC:	DS.B	1
REGDFC:	DS.B	1

GR0BUF:	DS.W	46-4		; Group 0 exception
REGFV:	DS.W	1		; Format / Vector offset
REG_E:

;STACK:		EQU	(REG_E+$200) & $ffffff00
;USTACK:	EQU	(REG_E+$300) & $ffffff00
	ds.b	((REG_E+$200)&$ff00)-REG_E
STACK:
	ds.b	$100
USTACK:

DAT_E	equ	USTACK

	END
