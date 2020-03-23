; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab4_challengecode.asm
;*
;*  This program displays two strings on the AVR's LCD. At
;*  the start, nothing is displayed on the LCD. When PD0 is
;*  pressed, the string "Jason Chen" is displayed on line 1
;*  of the display, and "Hello, World" is shown on line 2.
;*  When PD1 is pressed, the lines flip, so that "Jason
;*  Chen" is on line 2. When PD7 is pressed, the LCD's text
;*  is cleared.
;*
;*  When PD5 is pressed, the text on the display (same as
;*  that displayed when PD0 is pressed) scrolls from left-
;*  to-right until every character returns to its original
;*  position. When PD6 is pressed, the same thing happens,
;*  except the text scrolls right-to-left.
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
.def  waitcnt = r23   ; Wait Loop Counter
.def  ilcnt = r24     ; Inner Loop Counter
.def  olcnt = r25     ; Outer Loop Counter

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
    brne CheckPD5   ; PD7 not pressed so check for PD5
    rcall LCDClr    ; PD7 is hit; clear the data on the LCD
    rjmp  MAIN      ; Jump back to main to create infinite (polling) loop
CheckPD5:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD6|1<<PD7) ; Check if PD5 pressed
    brne CheckPD6   ; PD5 not pressed, so check for PD6
    rcall HandlePD5 ; PD5 is hit; call HandlePD5 function
    rjmp MAIN       ; Jump back to mian to create infinite (polling) loop
CheckPD6:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD5|1<<PD7) ; Check if PD6 pressed
    brne MAIN       ; PD6 not pressed so jump to main, beginning of loop
    rcall HandlePD6 ; PD6 is hit; call HandlePD6 function
    rjmp MAIN       ; Jump back to main to keep polling input

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
    ldi XL, low(NAME_STR_END<<1)
    ldi XH, high(NAME_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 1 of LCD
    ldi YL, low(LCDLn1Addr)
    ldi YH, high(LCDLn1Addr)
    ; Z register points to beginning of "Jason Chen" string
    ; in program memory
    ldi ZL, low(NAME_STR_BEG<<1)
    ldi ZH, high(NAME_STR_BEG<<1)
    ; Load the entire string into data memory for line 1
    rcall LoadLine

    ; X register points to end of "Hello, World" string in
    ; program memory
    ldi XL, low(HW_STR_END<<1)
    ldi XH, high(HW_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 2 of LCD
    ldi YL, low(LCDLn2Addr)
    ldi YH, high(LCDLn2Addr)
    ; Z register points to beginning of "Hello, World"
    ; string in program memory
    ldi ZL, low(HW_STR_BEG<<1)
    ldi ZH, high(HW_STR_BEG<<1)
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
    ldi XL, low(HW_STR_END<<1)
    ldi XH, high(HW_STR_END<<1)
    ; Y register points to first address in data memory for
    ; line 1 of LCD
    ldi YL, low(LCDLn1Addr)
    ldi YH, high(LCDLn1Addr)
    ; Z register points to beginning of "Hello, World"
    ; string in program memory
    ldi ZL, low(HW_STR_BEG<<1)
    ldi ZH, high(HW_STR_BEG<<1)
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

;-----------------------------------------------------------
; Func: HandlePD5
; Desc: This function is called when the PD5 button is hit.
;   It displays "Hello, World" on line 1 of the LCD and
;   "Jason Chen" on line 2, and then every 0.25 seconds,
;   scrolls the text from left-to-right. It does this by
;   shifting all of the characters down one memory address
;   and wrapping back up to the first memory address once
;   the last address of line 2 is reached.
;-----------------------------------------------------------
HandlePD5:
    push mpr        ; Save mpr
    in mpr, SREG    ; Save the status register
    push mpr        ;
    push waitcnt    ; Save waitcnt
    push ilcnt      ; Save ilcnt
    push r17        ; Save r17
    push YL         ; Save the Y pointer
    push YH         ;

    rcall HandlePD0 ; Call HandlePD0 to display strings on LCD

    ; Delay to show the unscrolled strings on LCD
    ldi waitcnt, 25 ; 0.25 second delay
    rcall DelayWait

    ldi ilcnt, 0    ; ilcnt stores number of times text has scrolled

    ; Make Y point to first address in memory used to store text
    ; displayed on the LCD
    ldi YL, low(LCDLn1Addr)
    ldi YH, high(LCDLn1Addr)
    ld mpr, Y+      ; Load first character on the first line 
; On each loop, stores the character pointed to by Y, then replaces
; that character with the previous character.
ShiftDownLoop:
    ld r17, Y       ; Load character before it's replaced
    st Y+, mpr      ; Store previous character at Y pointer (moves
                    ; character down one address)
    mov mpr, r17    ; Copy the saved character to mpr

    ; Check if final address in data memory used to store LCD data
    ; has been reached
    cpi YL, low(LCDLn2Addr + $F + 1)  ; Address is one-past the last LCD
                                      ; data address
    brne ShiftDownLoop                ; Loop has not reached end yet
    cpi YH, high(LCDLn2Addr + $F + 1) ; Address is one-past the last LCD
                                      ; data address
    brne ShiftDownLoop                ; Loop has not reached end yet

    ; Loop has reached the final LCD data address
    ldi YL, low(LCDLn1Addr)   ; Move back to the first address (wrap
                              ; text up to the first line)
    ldi YH, high(LCDLn1Addr)  ;
    st Y+, mpr                ; Store the last character of line two as
                              ; as first character of line one
    rcall LCDWrite            ; Write shifted characters to LCD
    inc ilcnt                 ; Increment the number of scrolls so far
    cpi ilcnt, 32             ; Check if 32 scrolls have been completed
    breq ScrollRightDone      ; When 32 scrolls have been done, characters
                              ; have returned to original positions, so stop
    ldi waitcnt, 25           ; Wait 0.25 seconds before scrolling again
    rcall DelayWait           ;
    rjmp ShiftDownLoop        ; Scroll again
; Finished scrolling 32 times, so return from function
ScrollRightDone:
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop r17         ; Restore r17
    pop ilcnt       ; Restore ilcnt
    pop waitcnt     ; Restore waitcnt
    pop mpr         ; Restore mpr
    out SREG, mpr   ; Restore status register
    pop mpr         ;
    ret             ; Return from function

;-----------------------------------------------------------
; Func: HandlePD6
; Desc: This function is called when the PD6 button is hit.
;   It displays "Hello, World" on line 1 of the LCD and
;   "Jason Chen" on line 2, and then every 0.25 seconds,
;   scrolls the text from right-to-left. It does this by
;   shifting all of the characters up one memory address
;   and wrapping back down to the last memory address once
;   the first address of line 1 is reached.
;-----------------------------------------------------------
HandlePD6:
    push mpr        ; Save mpr
    in mpr, SREG    ; Save the status register
    push mpr        ;
    push waitcnt    ; Save waitcnt
    push ilcnt      ; Save ilcnt
    push r17        ; Save r17
    push YL         ; Save the Y pointer
    push YH         ;

    rcall HandlePD0 ; Call HandlePD0 to display strings on LCD

    ; Delay to show the unscrolled strings on LCD
    ldi waitcnt, 25 ; 0.25 second delay
    rcall DelayWait

    ldi ilcnt, 0    ; ilcnt stores number of times text has scrolled

    ; Make Y point to ONE-PAST the last address in memory used to
    ; store text displayed on the LCD
    ldi YL, low(LCDLn2Addr + $F + 1)
    ldi YH, high(LCDLn2Addr + $F + 1)
    ld mpr, -Y      ; Load last character on second line
; On each loop, stores the character pointed to by address above Y
; (or the last address if wrap around), then replaces that character
; with the character that comes after it
ShiftUpLoop:
    ld r17, -Y    ; Load character before it's replaced
    st Y, mpr     ; Store previous character at Y pointer (movies
                  ; character up one address)
    mov mpr, r17  ; Copy the saved character to mpr

    ; Check if first address in data memory used to store LCD data
    ; has been reached
    cpi YL, low(LCDLn1Addr)   ; Address is the first address of line 1
    brne ShiftUpLoop          ; Loop has not reached first address yet
    cpi YH, high(LCDLn1Addr)  ; Address is first address of line 1
    brne ShiftUpLoop          ; Loop has not reached first address yet

    ; Loop has reached the first LCD data address
    ldi YL, low(LCDLn2Addr + $F + 1)  ; Move back to one-past the last
                                      ; address (wrap text down to the
                                      ; second line)
    ldi YH, high(LCDLn2Addr + $F + 1) ;
    st -Y, mpr            ; Store first character of line one as last
                          ; character of line two
    ld mpr, Y             ; Store just-saved character in mpr so it
                          ; gets moved left with loop
    rcall LCDWrite        ; Write shfited characters to LCD
    inc ilcnt             ; Increment number of scrolls
    cpi ilcnt, 32         ; Check if 32 scrolls completed
    breq ScrollRightDone  ; When 32 scrolls are done, characters have
                          ; returned to original positions, so stop
    ldi waitcnt, 25       ; Wait 0.25 seconds before scrolling again
    rcall DelayWait       ;
    rjmp ShiftUpLoop      ; Scroll again
; Finished scrolling 32 times, so return from function
ScrollLeftDone:
    pop YH          ; Restore Y pointer
    pop YL          ;
    pop r17         ; Restore r17
    pop ilcnt       ; Restore ilcnt
    pop waitcnt     ; Restore waitcnt
    pop mpr         ; Restore mpr
    out SREG, mpr   ; Restore status register
    pop mpr         ;
    ret             ; Return from function

;----------------------------------------------------------------
; Sub:  DelayWait
; Desc: A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;   waitcnt*10ms.  Just initialize wait for the specific amount 
;   of time in 10ms intervals. Here is the general eqaution
;   for the number of clock cycles in the wait loop:
;     ((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;
;   (This function is taken from the BumpBot example program on
;   the lab website.)
;----------------------------------------------------------------
DelayWait:
    push  waitcnt     ; Save wait register
    push  ilcnt       ; Save ilcnt register
    push  olcnt       ; Save olcnt register

Loop: ldi   olcnt, 224    ; load olcnt register
OLoop:  ldi   ilcnt, 237  ; load ilcnt register
ILoop:  dec   ilcnt       ; decrement ilcnt
    brne  ILoop     ; Continue Inner Loop
    dec   olcnt     ; decrement olcnt
    brne  OLoop     ; Continue Outer Loop
    dec   waitcnt   ; Decrement wait 
    brne  Loop      ; Continue Wait loop  

    pop   olcnt     ; Restore olcnt register
    pop   ilcnt     ; Restore ilcnt register
    pop   waitcnt   ; Restore wait register
    ret             ; Return from subroutine

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


