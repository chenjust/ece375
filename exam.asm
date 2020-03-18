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
.def	rlo				= r0	; Low byte of MUL result
.def	rhi				= r1	; High byte of MUL result
.def	zero			= r2	; Zero register, set to zero in INIT, useful for calculations
.def	A					= r3	; A variable
.def	avg				= r4	; Average distance result
.def	dsumlo		= r5	; Low byte for running sum of all three distances
.def	dsumhi		= r6	; High byte for running sum of all three distances
.def	sumlo			= r7	; Low byte for sum on loop iterations
.def	sumhi			= r8	; High byte for sum on loop iterations
.def	mpr				= r16	; Multipurpose register 
.def	oloop			= r17	; Outer Loop Counter
.def	iloop			= r18	; Inner Loop Counter
.def	bestnum		= r19	; Number corresponding to the closest treasure
.def	tnum			= r20	; Current treasure number
.def	smallest	= r21	; Distance to the closest treasure
.def	samecnt		= r22	; Number of treasures that are the same (closest) distance away

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
		clr		zero										; Clear the zero register
		clr		dsumlo									; Clear the dsumlo register
		clr		dsumhi									; Clear the dsumhi register
		clr		samecnt									; Clear samecnt register
		ser		smallest								; Set all bits in the smallest register

		ldi YL, low(Result1)					; Load low byte of Result1 address to YL
		ldi YH, high(Result1)					; Load high byte of Result1 address to YH
		ldi	ZL,	low(Treasure1<<1)			; Load low byte of Treasure1<<1 program memory address to ZL
		ldi ZH, high(Treasure1<<1)		; Load high byte of Treasure1<<1 program memory address to ZH

		ldi oloop, 3									; Outer loop counter; 3 for 3 treasures
		ldi tnum, 1
SquareOperandsOuter:
		clr A													; Clear the A register, which is used to hold the largest operand
																	; for each treasure
		clr sumlo											; Clear the sumlo register, which is used to hold the low byte of
																	; the squared operands sum (x^2 + y^2)
		clr sumhi											; Clear sumhi register (for high byte of above result)
		ldi iloop, 2									; Inner loop counter; 2 operands per treasure
SquareOperandsInner:
		lpm mpr, Z+										; Load operand from program memory
		cpi mpr, 0										; Compare mpr to constant value 0
		brge GetLargestOperand				; If value in mpr is positive, branch to GetLargestOperand
		neg mpr												; If value is negative, get the absolute value
GetLargestOperand:
		cp mpr, A											; Compare the value in mpr to the value in A
		brlo StoreSquaredOperands			; If it's less, branch to StoreSquaredOperands
		mov A, mpr										; If mpr >= A, replace the value in A with mpr; A holds the
																	; larger of the two operands for each treasure.
StoreSquaredOperands:
		mul mpr, mpr									; Square the value of the operand
		st Y+, rlo										; Store low byte of the product
		st Y+, rhi										; Store high byte of the product
		add sumlo, rlo								; Add low byte of squared value to the low byte of sum (of x^2 + y^2)
		adc sumhi, rhi								;	Add high byte of product with carry to high byte of sum
		dec iloop											; Decrement the inner loop counter
		brne SquareOperandsInner			; If iloop is not 0, branch to SquareOperandsInner
		st Y+, sumlo									; Store low byte of sum of the squared operands at Y
		st Y+, sumhi									; Store high byte of sum of squared operands at Y

SquareRootCalculate:
		mul A, A											; Square the value in A
		cp rlo, sumlo									; Compare low byte of squared value to sumlo
		cpc rhi, sumhi								; Compare high byte of squared value to sumhi
		brsh SquareRootFound					; If product is >= sumhi:sumlo, branch to SquareRootFound
		inc A													; Otherwise, increment A
		rjmp SquareRootCalculate			; Jump to SquareRootCalculate
SquareRootFound:
		st Y+, A											; Store the square root value at memory location Y
		cp smallest, A								; Compare the value in smallest with A
		brlo AddSqrtToSum							; If smallest < A, branch to AddSqrtToSum
		breq SqrtEqualsSmallest				; If smallest == A, branch to SqrtEqualsSmallest
		mov smallest, A								; If A < smallest, replace smallest with value in A
		mov bestnum, tnum							; Copy tnum (current treasure number) to bestnum
		ldi samecnt, 1								; Load samecnt to 1, since only one treasure is at this distance so far
		rjmp AddSqrtToSum							; Jump to AddSqrtToSum
SqrtEqualsSmallest:
		inc samecnt										; Increment samecnt, indicating this treasure is at same distance as
																	; another treasure
AddSqrtToSum:
		add dsumlo, A									; Add sqrt to low byte of running distance sum
		adc dsumhi, zero							; Add any carry from previous instruction to high byte of running sum
		inc	tnum											; Increment current treasure number
		dec oloop											; Decrement the outer loop counter
		brne SquareOperandsOuter			; If oloop is not 0, branch to SquareOperandsOuter

		cpi samecnt, 2								; Compare value of samecnt (number of treasures that share the closest
																	; distance) to 2
		breq BestDistanceSame					; If two treasures had the (same) closest distance, branch to
																	; BestDistanceSame
		brlo BestDistanceStore				; If only one treasure had the closest distance, branch to
																	; BestDistanceStore
		ldi bestnum, -3								; Otherwise, all 3 had the same distance; load -3 to bestnum
		rjmp BestDistanceStore				; BestDistanceStore
BestDistanceSame:
		ldi bestnum, -2								; Two treasures had the closest distance; load -2 to bestnum
BestDistanceStore:
		st Y+, bestnum								; Store treasure with closest distance at BestChoice address

		clr avg												; Clear the avg register
		ldi mpr, 3										; Load 3 to mpr; this is the number of treasures
AverageLoop:
		cp dsumlo, mpr								; Compare low byte of sum of all distances to mpr (3)
		cpc dsumhi, zero							; Compare high byte of distance sum with any carry from previous compare
		brlo AverageFound							; If the sum is < 3, branch to AverageFound
		sub dsumlo, mpr								; Subtract 3 from lower byte of sum
		sbc dsumhi, zero							; Subtract any carry from the previous sub instruction
		inc avg												;	Increment the number of times we're subtracted
		rjmp AverageLoop							; Jump to AverageLoop
AverageFound:
		ldi mpr, 2										; Load 2 to mpr
		cp dsumlo, mpr								; Compare value of dsumlo to 2
		brlo AverageStore							; If sumlo < 2, jump to AverageStore; this means the remainder <= 1
		inc avg												; Otherwise, round up, since the remainder is 2
		rjmp AverageStore							; Jump to AverageStore
AverageStore:
		st Y, avg											; Store rounded average at Y pointer

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
