; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab5_sourcecode.asm
;*
;*  This program adds two 16-bit numbers, subtracts two 16-
;*  bit numbers, multiplies two 24-bit numbers, and uses
;*  a combination of addition, subtraction, and
;*  multiplication to evaluate a compound expression.
;*
;***********************************************************
;*
;*   Author: Jason Chen
;*     Date: February 7, 2020
;*
;***********************************************************

.include "m128def.inc"      ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def  mpr = r16       ; Multipurpose register 
.def  rlo = r0        ; Low byte of MUL result
.def  rhi = r1        ; High byte of MUL result
.def  zero = r2       ; Zero register, set to zero in INIT, useful for calculations
.def  A = r3          ; A variable
.def  B = r4          ; Another variable

.def  oloop = r17     ; Outer Loop Counter
.def  iloop = r18     ; Inner Loop Counter
.def  oplen = r19     ; Operand length for LoadOperand


;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg             ; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org  $0000         ; Beginning of IVs
    rjmp  INIT      ; Reset interrupt

.org  $0046         ; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:             ; The initialization routine
    ; Initialize Stack Pointer
    ldi mpr, low(RAMEND)
    out SPL, mpr
    ldi mpr, high(RAMEND)
    out SPH, mpr

    clr   zero  ; Set the zero register to zero, maintain
                ; these semantics, meaning, don't
                ; load anything else into it.

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:             ; The Main program
    ldi oplen, 2                  ; Set operand length to 2 bytes for
                                  ; LoadOperand routine
    ; Setup the ADD16 function direct test
    ; Load beginning address of ADD16 operand 1 into Y
    ldi YL, low(ADD16_OP1)
    ldi YH, high(ADD16_OP1)
    ; Load beginning program memory address of ADD16 operand
    ; 1 into Z
    ldi ZL, low(AddOperand1<<1)
    ldi ZH, high(AddOperand1<<1)
    rcall LoadOperand             ; Load operand 1 into data memory pointed to by Y

    ; Load beginning address of ADD16 operand 2 into Y
    ldi YL, low(ADD16_OP2)
    ldi YH, high(ADD16_OP2)
    ; Load beginning program memory address of ADD16 operand
    ; 2 into Z
    ldi ZL, low(AddOperand2<<1)
    ldi ZH, high(AddOperand2<<1)
    rcall LoadOperand             ; Load operand 2 into data memory pointed to by Y

    nop ; Check load ADD16 operands (Set Break point here #1)  
    ; Call ADD16 function to test its correctness
    ; (calculate FCBA + FFFF)
    rcall ADD16

    nop ; Check ADD16 result (Set Break point here #2)
        ; Observe result in Memory window

    ; Setup the SUB16 function direct test
    ; Load beginning address of SUB16 operand 1 into Y
    ldi YL, low(SUB16_OP1)
    ldi YH, high(SUB16_OP1)
    ; Load beginning program memory address of SUB16 operand
    ; 1 into Z
    ldi ZL, low(SubOperand1<<1)
    ldi ZH, high(SubOperand1<<1)
    rcall LoadOperand             ; Load operand 1 into data memory pointed to by Y

    ; Setup the SUB16 function direct test
    ; Load beginning address of SUB16 operand 2 into Y
    ldi YL, low(SUB16_OP2)
    ldi YH, high(SUB16_OP2)
    ; Load beginning program memory address of SUB16 operand
    ; 2 into Z
    ldi ZL, low(SubOperand2<<1)
    ldi ZH, high(SubOperand2<<1)
    rcall LoadOperand             ; Load operand 2 into data memory pointed to by Y

    nop ; Check load SUB16 operands (Set Break point here #3)  
    ; Call SUB16 function to test its correctness
    ; (calculate FCB9 - E420)
    rcall SUB16

    nop ; Check SUB16 result (Set Break point here #4)
        ; Observe result in Memory window

    ldi oplen, 3                  ; Set operand length to 3 bytes for LoadOperand
                                  ; routine
    ; Setup the MUL24 function direct test
    ; Load beginning address of MUL24 operand 1 into Y
    ldi YL, low(MUL24_OP1)
    ldi YH, high(MUL24_OP1)
    ; Load beginning program memory address of MUL24 operand
    ; 1 into Z
    ldi ZL, low(MulOperand1<<1)
    ldi ZH, high(MulOperand1<<1)
    rcall LoadOperand             ; Load operand 1 into data memory pointed to by Y

    ; Load beginning address of MUL24 operand 2 into Y
    ldi YL, low(MUL24_OP2)
    ldi YH, high(MUL24_OP2)
    ; Load beginning program memory address of MUL24 operand
    ; 2 into Z
    ldi ZL, low(MulOperand2<<1)
    ldi ZH, high(MulOperand2<<1)
    rcall LoadOperand             ; Load operand 2 into data memory pointed to by Y

    nop ; Check load MUL24 operands (Set Break point here #5)  
    ; Call MUL24 function to test its correctness
    ; (calculate FFFFFF * FFFFFF)
    rcall MUL24

    nop ; Check MUL24 result (Set Break point here #6)
        ; Observe result in Memory window

    ldi oplen, 2                  ; Set operand length to 2 bytes for LoadOperand
    ; Load beginning address of COMPOUND operand D into Y
    ldi YL, low(COMPOUND_OPD)
    ldi YH, high(COMPOUND_OPD)
    ; Load beginning program memory address of COMPOUND operand
    ; D into Z
    ldi ZL, low(OperandD<<1)
    ldi ZH, high(OperandD<<1)
    rcall LoadOperand             ; Load operand D into data memory

    ; Load beginning address of COMPOUND operand E into Y
    ldi YL, low(COMPOUND_OPE)
    ldi YH, high(COMPOUND_OPE)
    ; Load beginning program memory address of COMPOUND operand
    ; E into Z
    ldi ZL, low(OperandE<<1)
    ldi ZH, high(OperandE<<1)
    rcall LoadOperand             ; Load operand E into data memory

    ; Load beginning address of COMPOUND operand F into Y
    ldi YL, low(COMPOUND_OPF)
    ldi YH, high(COMPOUND_OPF)
    ; Load beginning program memory address of COMPOUND operand
    ; F into Z
    ldi ZL, low(OperandF<<1)
    ldi ZH, high(OperandF<<1)
    rcall LoadOperand             ; Load operand F into data memory

    nop ; Check load COMPOUND operands (Set Break point here #7)  
    ; Call the COMPOUND function
    rcall COMPOUND

    nop ; Check COMPUND result (Set Break point here #8)
        ; Observe final result in Memory window
DONE:
    rjmp  DONE  ; Create an infinite while loop to signify the 
                ; end of the program.

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: ADD16
; Desc: Adds two 16-bit numbers and generates a 24-bit number
;   where the high byte of the result contains the carry
;   out bit.
;-----------------------------------------------------------
ADD16:
    push A        ; Save A register
    push B        ; Save B register
    push mpr      ; Save mpr register
    in mpr, SREG  ; Save status register
    push mpr      ;
    push XL       ; Save X pointer
    push XH       ;
    push YL       ; Save Y pointer
    push YH       ;
    push ZL       ; Save Z pointer
    push ZH       ;

    ; Load beginning address of first operand into X
    ldi XL, low(ADD16_OP1)  ; Load low byte of address
    ldi XH, high(ADD16_OP1) ; Load high byte of address

    ; Load beginning address of second operand into Y
    ldi YL, low(ADD16_OP2)
    ldi YH, high(ADD16_OP2)

    ; Load beginning address of result into Z
    ldi ZL, low(ADD16_Result)
    ldi ZH, high(ADD16_Result)

    ; Execute the function
    ld A, X+  ; Load lower byte of first operand
    ld B, Y+  ; Load lower byte of second operand
    add A, B  ; Add the two lower bytes together
    st Z+, A  ; Store the sum of the lower bytes as lower byte of sum
              ; and increment Z pointer to point to upper byte of sum
    ld A, X   ; Load upper byte of first operand
    ld B, Y   ; Load upper byte of second operand
    adc A, B  ; Add the upper bytes with the carry bit
    st Z+, A  ; Store the sum of upper bytes as upper byte of sum and
              ; increment Z pointer to store possible carry
    clr A     ; Clear A to prepare for instruction below
    adc A, zero ; Put the value of the carry bit into A
    st Z, A   ; Store the possible carry

    pop ZH          ; Restore Z pointer
    pop ZL          ;
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop XH          ; Restore X pointer
    pop XL          ;
    pop mpr         ; Restore status register
    out SREG, mpr   ;
    pop mpr         ; Restore mpr register
    pop B           ; Restore B register
    pop A           ; Restore A register
    ret             ; End a function with RET

;-----------------------------------------------------------
; Func: SUB16
; Desc: Subtracts two 16-bit numbers and generates a 16-bit
;   result.
;-----------------------------------------------------------
SUB16:
    push A        ; Save A register
    push B        ; Save B register
    push mpr      ; Save mpr register
    in mpr, SREG  ; Save status register
    push mpr      ;
    push XL       ; Save X pointer
    push XH       ;
    push YL       ; Save Y pointer
    push YH       ;
    push ZL       ; Save Z pointer
    push ZH       ;

    ; Load beginning address of first operand into X
    ; This is the value from which the second operand is subtracted
    ldi XL, low(SUB16_OP1)
    ldi XH, high(SUB16_OP1)
    ; Load beginning address of second operand into Y
    ; This is the value being subtracted from the first operand
    ldi YL, low(SUB16_OP2)
    ldi YH, high(SUB16_OP2)
    ; Load beginning address of result into Z
    ldi ZL, low(SUB16_RESULT)
    ldi ZH, high(SUB16_RESULT)

    ld A, X+    ; Load lower byte of first operand
    ld B, Y+    ; Load lower byte of second operand
    sub A, B    ; Subtract lower byte of second operand from lower
                ; byte of second operand
    st Z+, A    ; Store the difference of the lower bytes as lower
                ; byte of result
    ld A, X     ; Load upper byte of first operand
    ld B, Y     ; Load upper byte of second operand
    sbc A, B    ; Subtract upper byte of second operand, along with
                ; the carry bit, from the upper byte of first operand
    st Z, A     ; Store the difference of the upper bytes (and carry)
                ; as upper byte of result

    pop ZH          ; Restore Z pointer
    pop ZL          ;
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop XH          ; Restore X pointer
    pop XL          ;
    pop mpr         ; Restore status register
    out SREG, mpr   ;
    pop mpr         ; Restore mpr register
    pop B           ; Restore B register
    pop A           ; Restore A register
    ret             ; Return from function

;-----------------------------------------------------------
; Func: MUL24
; Desc: Multiplies two 24-bit numbers and generates a 48-bit 
;   result.
;-----------------------------------------------------------
MUL24:
    push mpr      ; Save mpr register
    in mpr, SREG  ; Save status register
    push mpr      ;
    push iloop    ; Save iloop register
    push oloop    ; Save oloop register
    push XL       ; Save X pointer
    push XH       ;
    push YL       ; Save Y pointer
    push YH       ;
    push ZL       ; Save Z pointer
    push ZH       ;

    ; Load beginning address of second operand (multiplier)
    ; into Y
    ldi YL, low(MUL24_OP2)
    ldi YH, high(MUL24_OP2)
    ; Load beginning address of result into Z
    ldi ZL, low(MUL24_RESULT)
    ldi ZH, high(MUL24_RESULT)

    ldi oloop, 3  ; Counter for LOAD_MULTIPLIER loop
; Load multiplier into lower 3 bytes of result
LOAD_MULTIPLIER:
    ld mpr, Y+    ; Load multiplier byte pointed to by Y into mpr
    st Z+, mpr    ; Store loaded multiplier byte into result pointer
    dec oloop     ; Decrement counter
    brne LOAD_MULTIPLIER  ; Iterate again if counter is not zero

    ; Zero out upper three bytes of the result
    ldi mpr, $00  ; Load zero into mpr
    st Z+, mpr    ; Store zero into the upper three bytes of result
    st Z+, mpr    ;
    st Z, mpr     ;

    ldi oloop, 24   ; Counter for ROTATE_OLOOP loop
; Outer rotate loop; performs a rotate right through carry
; from the most significant byte to the least significant byte
; and adds multiplicand to upper 3 bytes of result if rotate
; results in a carry
ROTATE_OLOOP:
    ; Load end address (one past the end) of result into Z
    ldi ZL, low(MUL24_RESULT+6)
    ldi ZH, high(MUL24_RESULT+6)
    ldi iloop, 6    ; Counter for ROTATE_ILOOP loop
; Inner rotate loop; performs rotate right through carry on all bytes
; from most significant to least significant.
ROTATE_ILOOP:
    ld mpr, -Z          ; Load byte pointed to by Z; starts at most
                        ; significant byte
    ror mpr             ; Rotate loaded byte to the right through carry
    st Z, mpr           ; Store rotated byte back into Z pointer
    dec iloop           ; Decrement inner loop counter
    brne ROTATE_ILOOP   ; Go back to top of loop and rotate again if six
                        ; rotations have not been done
    ; oloop is tested here because after 24 iterations of ROTATE_OLOOP have
    ; been completed, we want to rotate the result one last time and then
    ; return from the function, since the result is complete.
    tst oloop           ; Test if the oloop counter is zero
    breq MUL24_EXIT     ; If oloop counter is zero, return from function
    brcc SHIFT_DONE     ; If there is no carry, do not add
    clc                 ; Clear the carry bit; this is needed so we can use
                        ; adc instruction in CARRY_SET_ADD without carrying
                        ; in the carry bit from the rotation
    ldi iloop, 3                  ; Loop counter for CARRY_SET_ADD_LOOP
    ; Load address of the fourth result byte into Z since we will be adding
    ; the multiplicand to the upper 3 bytes
    ldi ZL, low(MUL24_RESULT+3)
    ldi ZH, high(MUL24_RESULT+3)
    ; Load beginning address of multiplicand into X
    ldi XL, low(MUL24_OP1)
    ldi XH, high(MUL24_OP1)
; This loop adds the multiplicand to the upper three bytes of the result when
; there is a carry from the rotate righ through carry
CARRY_SET_ADD:
    ld mpr, Z             ; Load byte pointed to be Z; starts at the fourth byte
                          ; of the result
    ld rhi, X+            ; Load multiplicand byte and increment to next byte
    adc mpr, rhi          ; Add the corresponding multiplicand and result byte
                          ; along with any carry from previous iteration of loop
    st Z+, mpr            ; Store sum back into Z
    dec iloop             ; Decrement iloop counter
    brne CARRY_SET_ADD    ; Loop until the three bytes have been added
; At this point, rotation (and possible addition of the multiplicand) is done
SHIFT_DONE:
    dec oloop           ; Decrement ROTATE_OLOOP loop counter
    rjmp ROTATE_OLOOP   ; Jump back to beginning of ROTATE_OLOOP; do this instead
                        ; branching when oloop is zero because if oloop IS zero,
                        ; we still want to rotate the result one last time
; Multiplication done
MUL24_EXIT:
    pop ZH          ; Restore Z pointer
    pop ZL          ;
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop XH          ; Restore X pointer
    pop XL          ;
    pop oloop       ; Restore oloop register
    pop iloop       ; Restore iloop register
    pop mpr         ; Restore status register
    out SREG, mpr   ;
    pop mpr         ; Restore mpr register
    ret             ; Return from function

;-----------------------------------------------------------
; Func: COMPOUND
; Desc: Computes the compound expression ((D - E) + F)^2
;   by making use of SUB16, ADD16, and MUL24.
;
;   D, E, and F are declared in program memory, and must
;   be moved into data memory for use as input operands.
;
;   All result bytes should be cleared before beginning.
;-----------------------------------------------------------
COMPOUND:
    push mpr  ; Save mpr register
    push XL   ; Save X pointer
    push XH   ;
    push YL   ; Save Y pointer
    push YH   ;
    push ZL   ; Save Z pointer
    push ZH   ;

    ; Setup SUB16 with operands D and E
    ; Load beginning address of first operand for SUB16 into Y
    ldi YL, low(SUB16_OP1)
    ldi YH, high(SUB16_OP1)
    ; Load beginning address of operand D into Z
    ldi ZL, low(COMPOUND_OPD)
    ldi ZH, high(COMPOUND_OPD)
    ; Copy operand D into the first operand for SUB16
    ld mpr, Z+
    st Y+, mpr
    ld mpr, Z
    st Y, mpr

    ; Load beginning address of second operand for SUB16 into Y
    ldi YL, low(SUB16_OP2)
    ldi YH, high(SUB16_OP2)
    ; Load beginning address of operand E into Z
    ldi ZL, low(COMPOUND_OPE)
    ldi ZH, high(COMPOUND_OPE)
    ; Copy operand E into second operand for SUB16
    ld mpr, Z+
    st Y+, mpr
    ld mpr, Z
    st Y, mpr
    ; Perform subtraction to calculate D - E
    rcall SUB16
    
    ; Setup the ADD16 function with SUB16 result and operand F
    ; Load beginning address of first operand for ADD16 into Y
    ldi YL, low(ADD16_OP1)
    ldi YH, high(ADD16_OP1)
    ; Load beginning address of SUB16 result into Z
    ldi ZL, low(SUB16_RESULT)
    ldi ZH, high(SUB16_RESULT)
    ; Copy SUB16 result into first operand for ADD16
    ld mpr, Z+
    st Y+, mpr
    ld mpr, Z+
    st Y, mpr

    ; Load beginning address of second operand for ADD16 into Y
    ldi YL, low(ADD16_OP2)
    ldi YH, high(ADD16_OP2)
    ; Load beginning address of operand F into Z
    ldi ZL, low(COMPOUND_OPF)
    ldi ZH, high(COMPOUND_OPF)
    ; Copy operand F into second operand for ADD16
    ld mpr, Z+
    st Y+, mpr
    ld mpr, Z
    st Y, mpr
    ; Perform addition next to calculate (D - E) + F
    rcall ADD16

    ; Setup the MUL24 function with ADD16 result as both operands
    ; Load beginning address of first operand for MUL24 into Y
    ldi YL, low(MUL24_OP1)
    ldi YH, high(MUL24_OP1)
    ; Load beginning address of second operand for MUL24 into Y
    ldi XL, low(MUL24_OP2)
    ldi XH, high(MUL24_OP2)
    ; Load beginning address of ADD16 result into Z
    ldi ZL, low(ADD16_RESULT)
    ldi ZH, high(ADD16_RESULT)
    ; Copy ADD16 result into both operands for MUL24
    ld mpr, Z+
    st X+, mpr
    st Y+, mpr
    ld mpr, Z+
    st X+, mpr
    st Y+, mpr
    ld mpr, Z
    st X, mpr
    st Y, mpr
    ; Perform multiplication to calculate ((D - E) + F)^2
    rcall MUL24

    pop ZH    ; Restore Z pointer
    pop ZL    ;
    pop YH    ; Restore Y pointer
    pop YL    ;
    pop XH    ; Restore X pointer
    pop XL    ;
    pop mpr   ; Restore mpr register
    ret       ; Return from function

;-----------------------------------------------------------
; Func: MUL16
; Desc: An example function that multiplies two 16-bit numbers
;     A - Operand A is gathered from address $0101:$0100
;     B - Operand B is gathered from address $0103:$0102
;     Res - Result is stored in address 
;         $0107:$0106:$0105:$0104
;   You will need to make sure that Res is cleared before
;   calling this function.
;-----------------------------------------------------------
MUL16:
    push  A       ; Save A register
    push  B       ; Save B register
    push  rhi     ; Save rhi register
    push  rlo     ; Save rlo register
    push  zero    ; Save zero register
    push  XH      ; Save X-ptr
    push  XL
    push  YH      ; Save Y-ptr
    push  YL      
    push  ZH      ; Save Z-ptr
    push  ZL
    push  oloop   ; Save counters
    push  iloop     

    clr   zero    ; Maintain zero semantics

    ; Set Y to beginning address of B
    ldi   YL, low(addrB)    ; Load low byte
    ldi   YH, high(addrB)   ; Load high byte

    ; Set Z to begginning address of resulting Product
    ldi   ZL, low(LAddrP)   ; Load low byte
    ldi   ZH, high(LAddrP)  ; Load high byte

    ; Begin outer for loop
    ldi   oloop, 2    ; Load counter
MUL16_OLOOP:
    ; Set X to beginning address of A
    ldi   XL, low(addrA)  ; Load low byte
    ldi   XH, high(addrA) ; Load high byte

    ; Begin inner for loop
    ldi   iloop, 2  ; Load counter
MUL16_ILOOP:
    ld    A, X+     ; Get byte of A operand
    ld    B, Y      ; Get byte of B operand
    mul   A,B       ; Multiply A and B
    ld    A, Z+     ; Get a result byte from memory
    ld    B, Z+     ; Get the next result byte from memory
    add   rlo, A    ; rlo <= rlo + A
    adc   rhi, B    ; rhi <= rhi + B + carry
    ld    A, Z      ; Get a third byte from the result
    adc   A, zero   ; Add carry to A
    st    Z, A      ; Store third byte to memory
    st    -Z, rhi   ; Store second byte to memory
    st    -Z, rlo   ; Store first byte to memory
    adiw  ZH:ZL, 1  ; Z <= Z + 1      
    dec   iloop     ; Decrement counter
    brne  MUL16_ILOOP   ; Loop if iLoop != 0
    ; End inner for loop

    sbiw  ZH:ZL, 1      ; Z <= Z - 1
    adiw  YH:YL, 1      ; Y <= Y + 1
    dec   oloop         ; Decrement counter
    brne  MUL16_OLOOP   ; Loop if oLoop != 0
    ; End outer for loop
        
    pop   iloop   ; Restore all registers in reverves order
    pop   oloop
    pop   ZL        
    pop   ZH
    pop   YL
    pop   YH
    pop   XL
    pop   XH
    pop   zero
    pop   rlo
    pop   rhi
    pop   B
    pop   A
    ret           ; End a function with RET

;-----------------------------------------------------------
; Func: LoadOperand
; Desc: Loads an operand pointed to by Z from program memory
;   into the location pointed to by Y. The number of bytes in
;   the operand is specified by the oplen register.
;-----------------------------------------------------------
LoadOperand:
    push mpr      ; Save mpr register
    in mpr, SREG  ; Save status register
    push mpr      ;
    push oplen    ; Save oplen register
    push YL       ; Save Y pointer
    push YH       ;
    push ZL       ; Save Z pointer
    push ZH       ;
; Loops and loads the number of bytes specified by the oplen
; register
LoadLoop:
    lpm mpr, Z+     ; Load program memory byte pointed to be Z and
                    ; increment to next byte
    st Y+, mpr      ; Store loaded program memory byte into location
                    ; pointed to by Y and increment Y
    dec oplen       ; Decrement number of bytes left to load
    brne LoadLoop   ; Return to top of loop if more bytes left

    pop ZH          ; Restore Z pointer
    pop ZL          ;
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop oplen       ; Restore oplen register
    pop mpr         ; Restore status register
    out SREG, mpr   ;
    pop mpr         ; Restore mpr register
    ret             ; Return from function

;***********************************************************
;*  Stored Program Data
;***********************************************************

; ADD16 operands
AddOperand1:
  .DW 0xFCBA        ; Operand 1 for ADD16
AddOperand2:
  .DW 0xFFFF        ; Operand 2 for ADD16

; SUB16 operands
SubOperand1:
  .DW 0xFCB9        ; Operand 1 for SUB16
SubOperand2:
  .DW 0xE420        ; Operand 2 for SUB16

; MUL24 operands
MulOperand1:
  .DW 0xFFFF        ; Operand 1 for MUL24
  .DW 0x00FF        ;
MulOperand2:
  .DW 0xFFFF        ; Operand 2 for MUL24
  .DW 0x00FF        ;

; Compoud operands
OperandD:
  .DW 0xFCBA        ; test value for operand D
OperandE:
  .DW 0x2019        ; test value for operand E
OperandF:
  .DW 0x21BB        ; test value for operand F

;***********************************************************
;*  Data Memory Allocation
;***********************************************************

.dseg
.org  $0100       ; data memory allocation for MUL16 example
addrA:  .byte 2
addrB:  .byte 2
LAddrP: .byte 4

; Data memory allocations for ADD16, SUB16, MUL24, and COMPOUND

.org  $0110     ; Data memory allocation for operands
ADD16_OP1:
  .byte 2       ; Allocate two bytes for first operand of ADD16
ADD16_OP2:
  .byte 2       ; Allocate two bytes for second operand of ADD16
SUB16_OP1:
  .byte 2       ; Allocate two bytes for first operand of SUB16
SUB16_OP2:
  .byte 2       ; Allocate two bytes for second operand of SUB16
MUL24_OP1:
  .byte 3       ; Allocate two bytes for first operand of MUL24
MUL24_OP2:
  .byte 3       ; Allocate two bytes for second operand of MUL24
COMPOUND_OPD:
  .byte 2       ; Allocate two bytes for operand D of COMPOUND
COMPOUND_OPE:
  .byte 2       ; Allocate two bytes for operand E of COMPOUND
COMPOUND_OPF:
  .byte 2       ; Allocate two bytes for operand F of COMPOUND

.org  $0130     ; Data memory allocation for results
ADD16_Result:
  .byte 3       ; Allocate three bytes for ADD16 result
SUB16_Result:
  .byte 2       ; Allocate two bytes for SUB16 result
MUL24_Result:
  .byte 6       ; Allocate six bytes for MUL24 result

;***********************************************************
;*  Additional Program Includes
;***********************************************************
; There are no additional file includes for this program
