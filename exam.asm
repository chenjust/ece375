;***********************************************************
;*	This is the final exam template for ECE375 Winter 2020
;***********************************************************
;*	 Author: Jason Chen
;*   Date: March 16, 2020
;***********************************************************
.include "m128def.inc"			; Include definition file
;***********************************************************
;*	Internal Register Definitions and Constants
;*	(feel free to edit these or add others)
;***********************************************************
.def	rlo = r0				; Low byte of MUL result
.def	rhi = r1				; High byte of MUL result
.def	zero = r2				; Zero register, set to zero in INIT, useful for calculations
.def	A = r3					; A variable
.def	B = r4					; Another variable
.def	dsumlo = r5
.def	dsumhi = r6
.def	mpr = r16				; Multipurpose register 
.def	oloop = r17				; Outer Loop Counter
.def	iloop = r18				; Inner Loop Counter
.def	dataptr = r19			; data ptr
.def	sumlo			= r21
.def	sumhi = r22
.def	smallest = r23
.def	samecnt	= r24

;***********************************************************
;*	Data segment variables
;*	(feel free to edit these or add others)
;***********************************************************
.dseg
.org	$0100						; data memory allocation for operands
operand1:		.byte 2				; allocate 2 bytes for a variable named op1


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment
;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt
.org	$0046					; End of Interrupt Vectors
;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:	; The initialization routine
		clr		zero
		clr		dsumlo
		clr		dsumhi
		clr		samecnt
		ser		smallest

; Compute square of treasure locations
		ldi YL, low(Result1)
		ldi YH, high(Result1)
		ldi	ZL,	low(Treasure1<<1)
		ldi ZH, high(Treasure1<<1)

		ldi oloop, 3									; Outer loop counter; 3 for 3 treasures
SquareOperandsOuter:
		clr A													; Clear the A register, which is used to hold the largest operand
																	; for each treasure
		clr sumlo
		clr sumhi
		ldi iloop, 2									; Inner loop counter; 2 operands per treasure
SquareOperandsInner:
		lpm mpr, Z+										; Load operand from program memory
		mov r20, mpr									; Copy mpr to r20
		cpi r20, 0										; Compare r20 to constant value 0
		brge GetLargestOperand				; If value in r20 is positive, branch to GetLargestOperand
		neg r20												; If the value is negative, get absolute value
GetLargestOperand:
		cp r20, A											; Compare the value in r20 to the value in A
		brlt StoreSquaredOperands			; If it's less, branch to StoreSquaredOperands
		mov A, r20										; If r20 >= A, replace the value in A with r20; A contains the
																	; larger of the two operands for each treasure.
StoreSquaredOperands:
		muls mpr, mpr									; Square the value of the operand
		st Y+, rlo										; Store low byte of the product
		st Y+, rhi										; Store high byte of the product
		add sumlo, rlo
		adc sumhi, rhi
		dec iloop											; Decrement the inner loop counter
		brne SquareOperandsInner			; If iloop is not 0, branch to SquareOperandsInner
		st Y+, sumlo
		st Y+, sumhi

SquareRootCalculate:
		mul A, A											; Square the value in A
		cp rlo, sumlo									; Compare low byte of squared value to sumlo
		cpc rhi, sumhi								; Compare high byte of squared value to sumhi
		brge SquareRootFound					; If product is >= sumhi:sumlo, branch to SquareRootFound
		inc A													; Otherwise, increment A
		rjmp SquareRootCalculate			; Jump to SquareRootCalculate
SquareRootFound:
		st Y+, A											; Store the square root value at memory location Y
		cp smallest, A								; Compare the value in smallest with A
		brlo AddSqrtToSum							; If smallest < A, branch to AddSqrtToSum
		breq SqrtEqualsSmallest				; If smallest == A, branch to SqrtEqualsSmallest
		mov smallest, A								; If A < smallest, replace smallest with value in A
		mov dataptr, oloop						; Copy oloop to dataptr
		sub dataptr, iloop						; Subtract iloop from dataptr; this gives the treasure number for
																	; the current iteration
		ldi samecnt, 1								; Load samecnt to 1, since only one treasure is at this distance so far
		rjmp AddSqrtToSum							; Jump to AddSqrtToSum
SqrtEqualsSmallest:
		inc samecnt										; Increment samecnt, indicating this treasure is at same distance as
																	; another treasure
AddSqrtToSum:
		add dsumlo, A
		adc dsumhi, zero
		dec oloop											; Decrement the outer loop counter
		brne SquareOperandsOuter			; If oloop is not 0, branch to SquareOperandsOuter

		cpi samecnt, 2								; Compare value of samecnt (number of treasures that share the closest
																	; distance) to 2
		breq BestDistanceSame					; If two treasures had the (same) closest distance, branch to
																	; BestDistanceSame
		brlo BestDistanceStore				; If only one treasure had the closest distance, branch to
																	; BestDistanceStore
		ldi dataptr, -3								; Otherwise, all 3 had the same distance; load -3 to dataptr
		rjmp BestDistanceStore				; BestDistanceStore
BestDistanceSame:
		ldi dataptr, -2								; Two treasures had the closest distance; load -2 to dataptr
BestDistanceStore:
		st Y+, dataptr								; Store treasure with closest distance at BestChoice address

		clr smallest									; Clear the smallest register
		ldi mpr, 3										; Load 3 to mpr; this is the number of treasures
AverageLoop:
		cp dsumlo, mpr								; Compare low byte of sum of all distances to mpr (3)
		cpc dsumhi, zero							; Compare high byte of distance sum with any carry from previous compare
		brlo AverageFound							; If the sum is < 3, branch to AverageFound
		sub dsumlo, mpr								; Subtract 3 from lower byte of sum
		sbc dsumhi, zero							; Subtract any carry from the previous sub instruction
		inc smallest									;	Increment the number of times we're subtracted
		rjmp AverageLoop							; Jump to AverageLoop
AverageFound:
		ldi mpr, 2										; Load 2 to mpr
		cp dsumlo, mpr								; Compare value of dsumlo to 2
		brlo AverageStore							; If sumlo < 2, jump to AverageStore; this means the remainder <= 1
		inc smallest									; Otherwise, round up, since the remainder is 2
		rjmp AverageStore							; Jump to AverageStore
AverageStore:
		st Y, smallest								; Store rounded average at Y pointer

		jmp	Grading

;***********************************************************
;*	Procedures and Subroutines
;***********************************************************

;***end of your code***end of your code***end of your code***end of your code***end of your code***
;******************************* Do not change below this point************************************
;******************************* Do not change below this point************************************
;******************************* Do not change below this point************************************

Grading:
		nop					; Check the results and number of cycles (The TA will set a breakpoint here)
rjmp Grading


;***********************************************************
;*	Stored Program Data
;***********************************************************

; Contents of program memory will be changed during testing
; The label names (Treasure1, Treasure2, etc) are not changed
Treasure1:	.DB	0xF9, 0xFD				; X, Y coordinate of treasure 1 (-7 in decimal), (-3 in decimal)
Treasure2:	.DB	0x03, 0x04				; X, Y coordinate of treasure 2 (+3 in decimal), (+4 in decimal)
Treasure3:	.DB	0x81, 0x76				; X, Y coordinate of treasure 3 (-127 in decimal), (+118 in decimal)

;***********************************************************
;*	Data Memory Allocation for Results
;***********************************************************
.dseg
.org	$0E00						; data memory allocation for results - Your grader only checks $0E00 - $0E16
Result1:		.byte 7				; x_squared, y_squared, x2_plus_y2, square_root (for treasure 1)
Result2:		.byte 7				; x_squared, y_squared, x2_plus_y2, square_root (for treasure 2)
Result3:		.byte 7				; x_squared, y_squared, x2_plus_y2, square_root (for treasure 3)
BestChoice:		.byte 1				; which treasure is closest? (indicate this with a value of 1, 2, or 3)
									; see the PDF for an explanation of the special case when 2 or more treasures
									; have an equal (rounded) distance
AvgDistance:	.byte 1				; the average distance to a treasure chest (rounded to the nearest integer)

;***********************************************************
;*	Additional Program Includes
;***********************************************************
; There are no additional file includes for this program
