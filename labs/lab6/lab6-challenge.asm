; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab6_challengecode.asm
;*
;*  This program uses interrupts to implement the BumpBot
;*  program. If the TekBot is not doing anything else (it
;*  has not received an interrupt), it moves forward. The
;*  number of times the right and left whiskers are hit
;*  is displayed on lines one and two of the LCD,
;*  respectively.
;* 
;*  When INT0 is triggered, indicating a right whisker hit,
;*  then TekBot reverses for one second, turns left for one
;*  second, and then resumes forward motion. When INT1 is
;*  triggered, the same thing happens, except the TekBot
;*  turns right instead of left. When INT2 or INT 3 are
;*  triggered, the right and left whisker hit counters,
;*  respectively, are cleared.
;*
;*  As part of the challenge, if alternating whiskers are
;*  hit five times (meaning INT0 and INT1 alternate five
;*  times), the TekBot will reverse for one second and turn
;*  to the respective direction for a few seconds rather
;*  than just one. Additionally, if the same whisker is
;*  hit twice in a row, the TekBot will reverse for two
;*  seconds and turn for another two.
;*
;***********************************************************
;*
;*   Author: Jason Chen
;*     Date: February 13, 2020
;*
;***********************************************************

.include "m128def.inc"      ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def  mpr = r16           ; Multipurpose register 
.def  waitcnt = r23       ; Wait Loop Counter
.def  ilcnt = r24         ; Inner Loop Counter
.def  olcnt = r25         ; Outer Loop Counter
.def  rightCounter = r4   ; Right whisker hit counter
.def  leftCounter = r5    ; Left whisker hit counter
.def  altCounter = r6     ; Alternating whisker hit counter
.def  prevMove = r7       ; Register storing previous turn command

.equ  WskrR = 0           ; Right Whisker Input Bit
.equ  WskrL = 1           ; Left Whisker Input Bit
.equ  EngineR = 5         ; Right engine bit
.equ  EngineL = 6         ; Left engine bit

.equ  Forward = (1<<EngineR|1<<EngineL)   ; Move forward command
.equ  Reverse = $00                       ; Reverse command
.equ  TurnRight = 1<<EngineL              ; Turn right command
.equ  TurnLeft  = 1<<EngineR              ; Turn left command

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg             ; Beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org  $0000           ; Beginning of IVs
    rjmp  INIT        ; Reset interrupt
.org $0002            ; IV for INT0
    rcall HitRight    ; Right whisker hit
    reti              ; Return from interrupt
.org $0004            ; IV for INT1
    rcall HitLeft     ; Left whisker hit
    reti              ; Return from interrupt
.org $0006            ; IV for INT2
    rcall ClearRight  ; Clear right whisker hit counter
    reti              ; Return from interrupt
.org $0008            ; IV for INT3
    rcall ClearLeft   ; Clear left whisker hit counter
    reti              ; Return from interrupt
.org  $0046           ; End of Interrupt Vectors

;***********************************************************
;*  Program Initialization
;***********************************************************
INIT:             ; The initialization routine
    ; Initialize Stack Pointer
    ldi mpr, low(RAMEND)      ; Set low byte of SP to low byte of RAMEND
    out SPL, mpr              ;
    ldi mpr, high(RAMEND)     ; Set high byte of SP to high byte of RAMEND
    out SPH, mpr              ;
    
    ; Initialize Port B for output
    ldi mpr, $FF              ; Make all port B pins outputs
    out DDRB, mpr             ;
    ldi mpr, $00              ; Turn off all LEDs initially
    out PORTB, mpr            ;
    
    ; Initialize Port D for input
    ldi mpr, $00              ; Make all port D pins inputs
    out DDRD, mpr             ;
    ldi mpr, $FF              ; Enable pull-up resistor for all port D pins
    out PORTD, mpr            ;

    ; Initialize external interrupts
    ldi mpr, $AA              ; Set falling-edge interrupts for INT0-3
    sts EICRA, mpr            ;

    ; Configure the External Interrupt Mask
    ldi mpr, $0F              ; Enable interrupts INT0-3
    out EIMSK, mpr            ;

    ; Initialize LCD display
    rcall LCDInit

    ; Initialize alternating whisker hit counter and register storing the previous
    ; turn command (for the challenge)
    clr altCounter            ; Clear alternating whisker hit counter
    clr prevMove              ; Clear register storing the previous turn command

    ; Initialize whisker hit counters
    clr rightCounter          ; Set right whisker hit counter to 0
    clr leftCounter           ; Set left whisker hit counter to 0
    rcall WriteRightCounter   ; Show initial value of right counter on LCD
    rcall WriteLeftCounter    ; Show initial value of left counter on LCD

    sei                       ; Set global interrupt bit to enable interrupts

;***********************************************************
;*  Main Program
;***********************************************************
MAIN:                 ; The Main program
    ldi mpr, Forward  ; Always move forward unless interrupted
    out PORTB, mpr    ;
    rjmp  MAIN        ; Create an infinite while loop to signify the 
                      ; end of the program.

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: HitRight
; Desc: Called when the right whisker (PD0/INT0) is hit;
;   increments the right whisker counter, reverses for one
;   second, turns left for one second, then resumes forward
;   motion. If HitRight is called twice in a row, the TekBot
;   will reverse and turn for two seconds each. If
;   alternating whiskers (right and left) are hit five
;   times, then the TekBot will reverse for one second and
;   turn for a few seconds.
;-----------------------------------------------------------
HitRight:
    push mpr                    ; Save mpr
    in mpr, SREG                ; Save the status register
    push mpr                    ;
    push waitcnt                ; Save waitcnt register
    push ilcnt                  ; Store ilcnt register
    push olcnt                  ; Restore olcnt register

    inc rightCounter            ; Increment right whisker counter
    rcall WriteRightCounter     ; Display incremented counter on LCD

    ldi olcnt, 100              ; Use olcnt to store reverse time
    ldi ilcnt, 100              ; Use ilcnt to store turn time

    mov mpr, prevMove           ; Copy prevMove to mpr to use cpi
    ; Check if same whisker hit twice in a row
    cpi mpr, TurnLeft           ; Was last executed command turn left?
    breq HitRightSameObject     ; If so, same object was hit twice in a row
    ; Check if alternating whisker hit
    cpi mpr, TurnRight          ; Was last executed command turn right?
    brne HitRightReset          ; If not, reset the alternative whisker counter
    inc altCounter              ; Otherwise, increment alt whisker counter
    mov mpr, altCounter         ; Copy alt whisker counter to mpr to use cpi
    cpi mpr, 5                  ; Have alt whiskers been hit 5 times in a row?
    brlo HitRightReverse        ; If not, do the normal BumpBot HitRight behavior
    brge HitRightAlternating    ; If so, turn for longer (180 degrees)
; Code jumps here when prevMove is neither TurnLeft nor TurnRight; should only
; be the case when program first starts
HitRightReset:
    clr altCounter              ; Clear the alt whisker hit counter
    rjmp HitRightReverse        ; Start reversing with default BumpBot behavior
; Code jumps here when same whisker is hit twice in a row
HitRightSameObject:
    clr altCounter              ; Clear alt whisker hit counter since same whisker
                                ; was hit twice in a row
    ldi olcnt, 200              ; Reverse for twice as long
    ldi ilcnt, 200              ; Turn for twice as long
    rjmp HitRightReverse        ; Start reversing
; Code jumps here when alternating whiskers have been hit five times
HitRightAlternating:
    clr altCounter              ; Clear alt whisker hit counter to start counting
                                ; another 5 alternative whisker hits
    ldi ilcnt, 255              ; Turn for a few seconds to get out of corner
HitRightReverse:
    ldi mpr, Reverse            ; Load reverse command to mpr
    out PORTB, mpr              ; Send reverse command to Port B
    mov waitcnt, olcnt          ; Copy reverse time from olcnt to waitcnt
    rcall DelayWait             ; Call wait function

    ldi mpr, TurnLeft           ; Load left turn command to mpr
    out PORTB, mpr              ; Send left turn command to Port B
    mov waitcnt, ilcnt          ; Copy turn time from ilcnt to waitcnt
    rcall DelayWait             ; Call wait function

    ldi mpr, Forward            ; Load move forward command to mpr
    out PORTB, mpr              ; Send move forward command to Port B

    ldi mpr, TurnLeft           ; Load left turn command to mpr
    mov prevMove, mpr           ; Copy left turn command from mpr to prevMove

    pop olcnt                   ; Restore olcnt register
    pop ilcnt                   ; Restore ilcnt register
    pop waitcnt                 ; Restore waitcnt register
    pop mpr                     ; Restore status register
    out SREG, mpr               ;

    ldi mpr, $0F                ; Clear any queued interrupts on INT0-3
    out EIFR, mpr               ;

    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: HitLeft
; Desc: Called when the left whisker (PD1/INT1) is hit;
;   increments the left whisker counter, reverses for one
;   second, turns right for one second, then resumes forward
;   motion. If HitLeft is called twice in a row, the TekBot
;   will reverse and turn for two seconds each. If
;   alternating whiskers (right and left) are hit five
;   times, then the TekBot will reverse for one second and
;   turn for a few seconds.
;-----------------------------------------------------------
HitLeft:
    push mpr                    ; Save mpr
    in mpr, SREG                ; Save the status register
    push mpr                    ;
    push waitcnt                ; Save waitcnt register
    push ilcnt                  ; Store ilcnt register
    push olcnt                  ; Restore olcnt register

    inc leftCounter             ; Increment left whisker counter
    rcall WriteLeftCounter      ; Display incremented counter on LCD

    ldi olcnt, 100              ; Use olcnt to store reverse time
    ldi ilcnt, 100              ; Use ilcnt to store turn time

    mov mpr, prevMove           ; Copy prevMove to mpr to use cpi
    ; Check if same whisker hit twice in a row
    cpi mpr, TurnRight          ; Was last executed command turn right?
    breq HitLeftSameObject      ; If so, same object was hit twice in a row
    ; Check if alternating whisker hit
    cpi mpr, TurnLeft           ; Was last executed command turn left?
    brne HitLeftReset           ; If not, reset the alternative whisker counter
    inc altCounter              ; Otherwise, increment alt whisker counter
    mov mpr, altCounter         ; Copy alt whisker counter to mpr to use cpi
    cpi mpr, 5                  ; Have alt whiskers been hit 5 times in a row?
    brlo HitLeftReverse         ; If not, do the normal BumpBot HitLeft behavior
    brge HitLeftAlternating     ; If so, turn for longer (180 degrees)
; Code jumps here when prevMove is neither TurnLeft nor TurnRight; should only
; be the case when program first starts
HitLeftReset:
    clr altCounter              ; Clear the alt whisker hit counter
    rjmp HitLeftReverse         ; Start reversing with default BumpBot behavior
; Code jumps here when same whisker is hit twice in a row
HitLeftSameObject:
    clr altCounter              ; Clear alt whisker hit counter since same whisker
                                ; was hit twice in a row
    ldi olcnt, 200              ; Reverse for twice as long
    ldi ilcnt, 200              ; Turn for twice as long
    rjmp HitLeftReverse         ; Start reversing
; Code jumps here when alternating whiskers have been hit five times
HitLeftAlternating:
    clr altCounter              ; Clear alt whisker hit counter to start counting
                                ; another 5 alternative whisker hits
    ldi ilcnt, 255              ; Turn for a few seconds to get out of corner
HitLeftReverse:
    ldi mpr, Reverse            ; Load reverse command to mpr
    out PORTB, mpr              ; Send reverse command to Port B
    mov waitcnt, olcnt          ; Copy reverse time from olcnt to waitcnt
    rcall DelayWait             ; Call wait function

    ldi mpr, TurnRight          ; Load right turn command to mpr
    out PORTB, mpr              ; Send right turn command to Port B
    mov waitcnt, ilcnt          ; Copy turn time from ilcnt to waitcnt
    rcall DelayWait             ; Call wait function

    ldi mpr, Forward            ; Load move forward command to mpr
    out PORTB, mpr              ; Send move forward command to Port B

    ldi mpr, TurnRight          ; Load right turn command to mpr
    mov prevMove, mpr           ; Copy right turn command from mpr to prevMove

    pop olcnt                   ; Restore olcnt register
    pop ilcnt                   ; Restore ilcnt register
    pop waitcnt                 ; Restore waitcnt register
    pop mpr                     ; Restore status register
    out SREG, mpr               ;

    ldi mpr, $0F                ; Clear any queued interrupts on INT0-3
    out EIFR, mpr               ;

    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: WriteRightCounter
; Desc: Displays the current value of the rightCounter
;   register on line one of the LCD.
;-----------------------------------------------------------
WriteRightCounter:
    push mpr                    ; Save mpr
    push XL                     ; Save X pointer
    push XH                     ;
    rcall LCDClrLn1             ; Clear line one of the LCD
    mov mpr, rightCounter       ; Copy current counter value to mpr
    ldi XL, low(LCDLn1Addr)     ; Load first memory address of LCD line 1 to X
    ldi XH, high(LCDLn1Addr)    ;
    rcall Bin2ASCII             ; Call Bin2ASCII to load ASCII values of each digit
                                ; into the LCD memory address location(s) pointed
                                ; to by X
    rcall LCDWrLn1              ; Call LCDWrLn1 to show counter one line 1
    pop XL                      ; Restore X pointer
    pop XH                      ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: WriteLeftCounter
; Desc: Displays the current value of the leftCounter
;   register on line two of the LCD.
;-----------------------------------------------------------
WriteLeftCounter:
    push mpr                    ; Save mpr
    push XL                     ; Save X pointer
    push XH                     ;
    rcall LCDClrLn2             ; Clear line two of the LCD
    mov mpr, leftCounter        ; Copy current counter value to mpr
    ldi XL, low(LCDLn2Addr)     ; Load first memory address of LCD line 2 to X
    ldi XH, high(LCDLn2Addr)    ;
    rcall Bin2ASCII             ; Call Bin2ASCII to load ASCII values of each digit
                                ; into the LCD memory address location(s) pointed
                                ; to by X
    rcall LCDWrLn2              ; Call LCDWrLn2 to show counter one line 2
    pop XL                      ; Restore X pointer
    pop XH                      ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: ClearRight
; Desc: Clears the right whisker counter by resetting
;   rightCounter to 0 and displaying it on line 1 of the
;   LCD.
;-----------------------------------------------------------
ClearRight:
    push mpr                  ; Save mpr
    in mpr, SREG              ; Save the status register
    push mpr                  ;
    clr rightCounter          ; Reset the rightCounter to 0
    rcall WriteRightCounter   ; Show reset value on LCD line 1
    pop mpr                   ; Restore status register
    out SREG, mpr             ;
    ldi mpr, $0F              ; Clear any queued interrupts on INT0-3
    out EIFR, mpr             ;
    pop mpr                   ; Restore mpr
    ret                       ; Return from function

;-----------------------------------------------------------
; Func: ClearLeft
; Desc: Clears the left whisker counter by resetting
;   leftCounter to 0 and displaying it on line 2 of the
;   LCD.
;-----------------------------------------------------------
ClearLeft:
    push mpr                  ; Save mpr
    in mpr, SREG              ; Save the status register
    push mpr                  ;
    clr leftCounter           ; Reset the leftCounter to 0
    rcall WriteLeftCounter    ; Show reset value on LCD line 2
    pop mpr                   ; Restore status register
    out SREG, mpr             ;
    ldi mpr, $0F              ; Clear any queued interrupts on INT0-3
    out EIFR, mpr             ;
    pop mpr                   ; Restore mpr
    ret                       ; Return from function

;----------------------------------------------------------------
; Sub: DelayWait
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
    push  waitcnt         ; Save wait register
    push  ilcnt           ; Save ilcnt register
    push  olcnt           ; Save olcnt register

Loop: ldi   olcnt, 224    ; load olcnt register
OLoop:  ldi   ilcnt, 237  ; load ilcnt register
ILoop:  dec   ilcnt       ; decrement ilcnt
    brne  ILoop           ; Continue Inner Loop
    dec   olcnt           ; decrement olcnt
    brne  OLoop           ; Continue Outer Loop
    dec   waitcnt         ; Decrement wait 
    brne  Loop            ; Continue Wait loop  

    pop   olcnt           ; Restore olcnt register
    pop   ilcnt           ; Restore ilcnt register
    pop   waitcnt         ; Restore wait register
    ret                   ; Return from subroutine


;***********************************************************
;*  Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"    ; Include the LCD Driver

