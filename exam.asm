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
opabs: .byte 3


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
		clr		sumlo
		clr		sumhi
		clr		samecnt
		ser		smallest

; Compute square of treasure locations
		ldi XL, low(opabs)
		ldi XH, high(opabs)
		ldi YL, low(Result1)
		ldi YH, high(Result1)
		ldi	ZL,	low(Treasure1<<1)
		ldi ZH, high(Treasure1<<1)
		ldi oloop, 3									; Outer loop counter; 3 for 3 treasures

SquareOperandsOuter:
		clr A													; Clear the A register, which is used to hold the largest operand
																	; for each treasure
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
		dec iloop											; Decrement the inner loop counter
		brne SquareOperandsInner			; If iloop is not 0, branch to SquareOperandsInner
		adiw YH:YL, 3									; Add 3 to the Y pointer for first memory location of next treasure
		st X+, A											; Store the larger of the two treasure operands at X pointer
		dec oloop											; Decrement the outer loop counter
		brne SquareOperandsOuter			; If oloop is not 0, branch to SquareOperandsOuter

		ldi YL, low(Result1)
		ldi YH, high(Result1)
		ldi oloop, 3									; Outer loop counter; 3 for 3 treasures
AddSquaredOperands:
		ld A, Y+											; Load low byte of first squared operand (x^2) into A
		ldd B, Y+1										; Load low byte of second squared operand (y^2) into B
		add A, B											; Add the low bytes of each squared operand and store into A
		std Y+3, A										; Store sum of lower bytes at memory location Y+3
		ld A, Y+											; Load high byte of first squared operand (x^2) into A
		ldd B, Y+1										; Load high byte of second squared operand (y^2) into B
		adc A, B											; Add high bytes of each operand with carry and store into A
		std Y+3, A										; Store sum of high bytes at memory location Y+3
		dec oloop											; Decrement outer loop counter
		breq SquareRootInit						; If oloop is 0, branch to SquareRootInit
		adiw YH:YL, 5									; Add 5 to the Y pointer for memory location of next treasure
		rjmp AddSquaredOperands				; Jump back to AddSquaredOperands

SquareRootInit:
		ldi XL, low(opabs+3)
		ldi XH, high(opabs+3)
		ldi YL, low(Result3+6)
		ldi YH, high(Result3+6)
		ldi oloop, 3
SquareRootLoop:
		ld B, -Y											; Load high byte of sum (x^2 + y^2) into B
		ld A, -Y											; Load low byte of sum (x^2 + y^2) into B

		ld mpr, -X										; Load the smaller of |x| and |y| into mpr; instead of starting
																	; from 0, we start from the larger of the two operands to find
																	; the square root
SquareRootCalculate:
		mul mpr, mpr									; Square the value in mpr
		cp rlo, A											; Compare low byte of squared value to A
		cpc rhi, B										; Compare high byte of squared value to B
		brge SquareRootFound					; If product is >= B:A, branch to SquareRootFound
		inc mpr												; Otherwise, increment mpr
		rjmp SquareRootCalculate			; Jump to SquareRootCalculate
SquareRootFound:
		std Y+2, mpr									; Store the square root value at memory location Y+2
		cp smallest, mpr							; Compare the value in smallest with mpr
		brlo AddSqrtToSum							; If smallest < mpr, branch to AddSqrtToSum
		breq SqrtEqualsSmallest				; If smallest == mpr, branch to SqrtEqualsSmallest
		mov smallest, mpr							; This mpr < smallest, so replace smallest with value in mpr
		mov dataptr, oloop						; Copy oloop to dataptr; this is the number of the closest treasure
		ldi samecnt, 1								; Load samecnt to 1, since only one treasure is at this distance so far
		rjmp AddSqrtToSum							; Jump to AddSqrtToSum
SqrtEqualsSmallest:
		inc samecnt										; Increment samecnt, indicating this treasure is at same distance as
																	; another treasure
AddSqrtToSum:
		add sumlo, mpr								; Add low byte of sum with mpr and store back into sumlo
		adc	sumhi, zero								; Add any carry from the previous instruction to sumhi
SRtCont:
		dec oloop											; Decrement the loop counter
		breq SquareRootDone						; If oloop is 0, branch to SquareRootDone
		sbiw YH:YL, 5									; Subtract 5 from Y pointer for memory location of previous treasure
		rjmp SquareRootLoop						; Jump to SquareRootLoop

SquareRootDone:
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
		ldi YL, low(BestChoice)
		ldi YH, high(BestChoice)
		st Y, dataptr									; Store treasure with closest distance at BestChoice address

		clr smallest									; Clear the smallest register
		ldi mpr, 3										; Load 3 to mpr; this is the number of treasures
AverageLoop:
		cp sumlo, mpr									; Compare low byte of sum of all distances to mpr (3)
		cpc sumhi, zero								; Compare high byte of distance sum with any carry from previous compare
		brlo AverageFound							; If the sum is < 3, branch to AverageFound
		sub sumlo, mpr								; Subtract 3 from lower byte of sum
		sbc sumhi, zero								; Subtract any carry from the previous sub instruction
		inc smallest									;	Increment the number of times we're subtracted
		rjmp AverageLoop							; Jump to AverageLoop
AverageFound:
		cpi sumlo, 2									; Compare value of sumlo to 2
		brlo AverageStore							; If sumlo < 2, jump to AverageStore; this means the remainder <= 1
		inc smallest									; Otherwise, round up, since the remainder is 2
		rjmp AverageStore							; Jump to AverageStore
AverageStore:
		ldi YL, low(AvgDistance)
		ldi YH, high(AvgDistance)
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
