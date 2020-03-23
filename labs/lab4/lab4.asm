; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab4_sourcecode.asm
;*
;*  This program displays two strings on the AVR's LCD. At
;*  the start, nothing is displayed on the LCD. When PD0 is
;*  pressed, the string "Jason Chen" is displayed on line 1
;*  of the display, and "Hello, World" is shown on line 2.
;*  When PD1 is pressed, the lines flip, so that "Jason
;*  Chen" is on line 2. When PD7 is pressed, the LCD's text
;*  is cleared.
;*
;***********************************************************
;*
;*   Author: Jason Chen
;*     Date: January 30, 2020
;*
;***********************************************************

.include "m128def.inc"      ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def  mpr = r16       ; Multipurpose register is
                      ; required for LCD Driver

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg               ; Beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org  $0000         ; Beginning of IVs
    rjmp INIT       ; Reset interrupt

.org  $0046         ; End of Interrupt Vectors

;***********************************************************
;*  Program Initialization
;***********************************************************
INIT:               ; The initialization routine
    ; Initialize Stack Pointer
    ldi mpr, low(RAMEND)
    out SPL, mpr    ; Set low byte of stack pointer to low byte of RAMEND
    ldi mpr, high(RAMEND)
    out SPH, mpr    ; Set high byte of stack pointer to high byte of RAMEND

    ; Initialize port D for input
    ldi mpr, $00
    out DDRD, mpr   ; Make all port D pins inputs
    ldi mpr,  $FF
    out PORTD, mpr  ; Enable pull-up resistor for all port D pins
    
    ; Initialize LCD Display
    rcall LCDInit
    
;***********************************************************
;*  Main Program
;***********************************************************
MAIN:               ; The Main program
    in  mpr, PIND   ; Get current state of port D pins
    andi mpr, (1<<PD0|1<<PD1|1<<PD5|1<<PD6|1<<PD7) ; Check for active low input
    cpi mpr, (1<<PD1|1<<PD5|1<<PD6|1<<PD7) ; Check if PD0 pressed
    brne CheckPD1   ; PD0 not pressed so check for PD1
    rcall HandlePD0 ; PD0 is hit; call HandlePD0 function
    rjmp MAIN       ; Infinite loop; continue polling input
CheckPD1:
    cpi mpr, (1<<PD0|1<<PD5|1<<PD6|1<<PD7) ; Check if PD1 pressed
    brne CheckPD7   ; PD1 not pressed so check for PD7
    rcall HandlePD1 ; PD1 is hit; call HandlePD1 function
    rjmp MAIN       ; Infinite loop; continue polling input
CheckPD7:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD5|1<<PD6) ; Check if PD7 pressed
    brne MAIN       ; PD7 not pressed so jump to main, beginning of loop
    rcall LCDClr    ; PD7 is hit; clear the data on the LCD
    rjmp  MAIN      ; Jump back to main to create infinite (polling) loop

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: LoadLine
; Desc: Load an entire string from program memory into data
;   memory. The Z register should point to the start of the
;   string in program memory. The X register should point
;   to the end of the string in program memory. The Y
;   register should point to the first address in the data
;   memory to which the string should be copied.
;
;   Copies characters from program memory into data memory
;   until the address stored in the X register is reached.
;-----------------------------------------------------------
LoadLine:
    push mpr        ; Save mpr
    in mpr, SREG    ; Save the status register
    push mpr        ;
    push ZL         ; Save the Z pointer
    push ZH         ;
    push YL         ; Save the Y pointer
    push YH         ;
LoadLineLoop:
    lpm mpr, Z+     ; Load byte from program memory into mpr then increment Z to go to next program memory byte
    st Y+, mpr      ; Store the loaded byte from program memory into data memory then increment the Y pointer
    cp ZL, XL       ; Check if lower byte of current program memory pointer matches lower byte of the pointer to the end of the string
    brne LoadLineLoop ; Not at end of string, so continue loading bytes from program memory
    cp ZH, XH       ; Check if upper byte of current program memory pointer matches upper byte of the pointer to the end of the string
    brne LoadLineLoop ; Not at end of string, so continue loading bytes from program memory

    pop YH          ; Restore the Y pointer
    pop YL          ;
    pop ZH          ; Restore the Z pointer
    pop ZL          ;
    pop mpr         ; Restore mpr
    out SREG, mpr   ; Restore the status register
    pop mpr         ;
    ret             ; Finished loading string; return from function

;-----------------------------------------------------------
; Func: HandlePD0
; Desc: This function is called when the PD0 button is hit.
;   It displays "Jason Chen" on line 1 of the LCD and
;   "Hello, World" on line 2 by copying each string from
;   program memory into the specific data memory locations
;   for each line.
;-----------------------------------------------------------
HandlePD0:
    push XL         ; Save the X register
    push XH         ;
    push YL         ; Save the Y register
    push YH         ;
    push ZL         ; Save the Z register
    push ZH         ;

    ; X register points to end of "Jason Chen" string in
    ; program memory
    ldi   XL, low(NAME_STR_END<<1)
    ldi   XH, high(NAME_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 1 of LCD
    ldi   YL, low(LCDLn1Addr)
    ldi   YH, high(LCDLn1Addr)
    ; Z register points to beginning of "Jason Chen" string
    ; in program memory
    ldi   ZL, low(NAME_STR_BEG<<1)
    ldi   ZH, high(NAME_STR_BEG<<1)
    ; Load the entire string into data memory for line 1
    rcall LoadLine

    ; X register points to end of "Hello, World" string in
    ; program memory
    ldi   XL, low(HW_STR_END<<1)
    ldi   XH, high(HW_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 2 of LCD
    ldi   YL, low(LCDLn2Addr)
    ldi   YH, high(LCDLn2Addr)
    ; Z register points to beginning of "Hello, World"
    ; string in program memory
    ldi   ZL, low(HW_STR_BEG<<1)
    ldi   ZH, high(HW_STR_BEG<<1)
    ; Load the entire string into data memory for line 2
    rcall LoadLine

    ; Write the loaded data to the two lines of the LCD
    rcall LCDWrite

    pop ZH          ; Restore the Z register
    pop ZL          ;
    pop YH          ; Restore the Y register
    pop YL          ;
    pop XH          ; Restore the X register
    pop XL          ;
    ret             ; Return from function

;-----------------------------------------------------------
; Func: HandlePD1
; Desc: This function is called when the PD1 button is hit.
;   It displays "Hello, World" on line 1 of the LCD and
;   "Jason Chen" on line 2 by copying each string from
;   program memory into the specific data memory locations
;   for each line.
;-----------------------------------------------------------
HandlePD1:
    push XL         ; Save the X register
    push XH         ;
    push YL         ; Save the Y register
    push YH         ;
    push ZL         ; Save the Z register
    push ZH         ;

    ; X register points to end of "Hello, World" string in
    ; program memory
    ldi   XL, low(HW_STR_END<<1)
    ldi   XH, high(HW_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 1 of LCD
    ldi   YL, low(LCDLn1Addr)
    ldi   YH, high(LCDLn1Addr)
    ; Z register points to beginning of "Hello, World"
    ; string in program memory
    ldi   ZL, low(HW_STR_BEG<<1)
    ldi   ZH, high(HW_STR_BEG<<1)
    ; Load the entire string into data memory for line 1
    rcall LoadLine

    ; X register points to end of "Jason Chen" string in
    ; program memory
    ldi XL, low(NAME_STR_END<<1)
    ldi XH, high(NAME_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 2 of LCD
    ldi YL, low(LCDLn2Addr)
    ldi YH, high(LCDLn2Addr)
    ; Z register points to beginning of "Jason Chen" string
    ; in program memory
    ldi ZL, low(NAME_STR_BEG<<1)
    ldi ZH, high(NAME_STR_BEG<<1)
    ; Load the entire string into data memory for line 2
    rcall LoadLine

    ; Write the loaded data to the two lines of the LCD
    rcall LCDWrite

    pop ZH          ; Restore the Z register
    pop ZL          ;
    pop YH          ; Restore the Y register
    pop YL          ;
    pop XH          ; Restore the X register
    pop XL          ;
    ret             ; Return from function

;***********************************************************
;*  Stored Program Data
;***********************************************************

;-----------------------------------------------------------
; String with my name; displayed on line 1 when PD0 is hit
; and on line 2 when PD1 is hit.
;-----------------------------------------------------------
NAME_STR_BEG:
.DB "Jason Chen      "  ; Extra spaces added to make 16 bits
NAME_STR_END:
;-----------------------------------------------------------
; Hello world string; displayed on line 1 when PD1 is hit
; and on line 2 when PD0 is hit.
;-----------------------------------------------------------
HW_STR_BEG:
.DB "Hello, World    "  ; Extra spaces added to make 16 bits
HW_STR_END:

;***********************************************************
;*  Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"    ; Include the LCD Driver


