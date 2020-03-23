; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab7_sourcecode.asm
;*
;*  This program uses timer/counters to generate pulse-width
;*  modulation (PWM) signals to control the speed of a
;*  TekBot. The TekBot is always moving forward, but its
;*  speed, at any given time, is 1 of 16 different possible
;*  values. The speed is indicated by two LEDs. Initially,
;*  the TekBot moves at the maximum speed.
;*
;*  When INT0 (button PD0) is triggered, the TekBot's speed
;*  increases by one level if it is not already at max
;*  speed. When INT1 is triggered, its speed decreases by
;*  one level, unless it is already at minimum speed. When
;*  INT2 is hit, the TekBot's speed jumps to the highest
;*  speed unless it is already there. And when INT3 is hit,
;*  the TekBot's speed decreases to the lowest level unless
;*  it is already there.
;*
;***********************************************************
;*
;*   Author: Jason Chen
;*     Date: February 21, 2020
;*
;***********************************************************

.include "m128def.inc"      ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def  mpr = r16         ; Multipurpose register
.def  speed = r17       ; Register holding current speed
.def  waitcnt = r18     ; Wait Loop Counter
.def  ilcnt = r19       ; Inner Loop Counter
.def  olcnt = r20       ; Outer Loop Counter

.equ  EngDirR = 5       ; Right Engine Direction Bit
.equ  EngDirL = 6       ; Left Engine Direction Bit

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg                   ; beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org  $0000             ; Beginning of IVs
    rjmp  INIT          ; Reset interrupt
.org  $0002             ; IV for INT0
    rcall IncrSpeed     ; Increase speed by one level
    reti                ; Return from interrupt
.org  $0004             ; IV for INT1
    rcall DecrSpeed     ; Decrease speed by one level
    reti                ; Return from interrupt
.org  $0006             ; IV for INT2
    rcall SetMaxSpeed   ; Increase speed to highest level
    reti                ; Return from interrupt
.org  $0008             ; IV for INT3
    rcall SetMinSpeed   ; Decrease speed to lowest level
    reti                ; Return from interrupt
.org  $0046             ; End of interrupt vectors

;***********************************************************
;*  Program Initialization
;***********************************************************
INIT:                         ; Initialization routine
    ; Initialize the Stack Pointer
    ldi mpr, low(RAMEND)      ; Set low byte of SP to low byte of RAMEND
    out SPL, mpr              ;
    ldi mpr, high(RAMEND)     ; Set high byte of SP to high byte of RAMEND
    out SPH, mpr              ;

    ; Configure I/O ports
    ldi mpr, $FF              ; Make all port B pins outputs
    out DDRB, mpr             ;

    ; Configure External Interrupts, if needed
    ldi mpr, $AA              ; Set falling-edge interrupts for INT3-0
    sts EICRA, mpr            ;
    ldi mpr, $0F              ; Enable interrupts INT3-0
    out EIMSK, mpr            ;

    ; Configure 8-bit Timer/Counters
    ldi mpr, 0b01101001       ; Configure Timer/Counter0 and 2 with fast PWM
                              ; wave generation mode, non-prescaled clock,
                              ; and clear OC0 on compare match
    out TCCR0, mpr            ;
    out TCCR2, mpr            ;

    ; Set TekBot to Move Forward (1<<EngDirR|1<<EngDirL)
    ldi mpr, (1<<EngDirR|1<<EngDirL)    ; Make TekBot move forward
    out PORTB, mpr                      ;

    ; Set initial speed, display on Port B pins 3:0
    ldi speed, 15             ; Set initial speed to 15
    ldi mpr, $60              ; Move forward LEDs always on (1<<EngDirR|1<<EngDirL)
    eor mpr, speed            ; Indicate speed LED 3:0 (right 4 LEDs)
    out PORTB, mpr            ;
    ldi mpr, $00              ; Set initial duty cycle of 0% (for max speed)
    out OCR0, mpr             ;
    out OCR2, mpr             ;

    ; Enable global interrupts (if any are used)
    sei                       ; Set global interrupt bit to enable interrupts

;***********************************************************
;*  Main Program
;***********************************************************
MAIN:                 ; The Main program
    rjmp MAIN         ; return to top of MAIN

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: IncrSpeed
; Desc: Increments the current speed of the TekBot by one
;   level, if it is not already at the maximum speed. This
;   is called when the INT0 interrupt is triggered.
;-----------------------------------------------------------
IncrSpeed:
    push mpr                    ; Save mpr
    in mpr, SREG                ; Save status register
    push mpr                    ;

    rcall DebounceInput         ; Debounce button push since this function
                                ; executes so quickly that a single push can
                                ; end up triggering the interrupt multiple
                                ; times
    mov mpr, speed              ; Copy speed into mpr, since we can't cpi <= r15
    cpi mpr, 15                 ; Check if TekBot already at max speed
    breq IncrDone               ; If so, don't do anything
    inc speed                   ; Increment speed by one level
    ldi mpr, $60                ; Keep move forward bits set
    eor mpr, speed              ; Indicate speed on LEDs 3:0
    out PORTB, mpr              ;
    ldi mpr, 17                 ; To get 16 levels (including 0), 255/15 = 17
    mul mpr, speed              ; TekBot speed is the speed level times the increment
                                ; per level
    ldi mpr, 255                ; Load the maximum speed value into mpr
    sub mpr, r0                 ; Subtract difference of the maximum speed and
                                ; current speed to get duty cycle value (since maximum
                                ; speed is a 0% duty cycle)
    out OCR0, mpr               ; Generate signals with new duty cycle
    out OCR2, mpr               ;
IncrDone:
    pop mpr                     ; Restore status register
    out SREG, mpr               ;
    ldi mpr, $0F                ; Clear any queued interrupts on INT3-0
    out EIFR, mpr               ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: DecrSpeed
; Desc: Decrements the current speed of the TekBot by one
;   level, if it is not already at the minimum speed. This
;   is called when the INT1 interrupt is triggered.
;-----------------------------------------------------------
DecrSpeed:
    push mpr                    ; Save mpr
    in mpr, SREG                ; Save status register
    push mpr                    ;

    rcall DebounceInput         ; Debounce button push since this function
                                ; executes so quickly that a single push can
                                ; end up triggering the interrupt multiple
                                ; times
    mov mpr, speed              ; Copy speed into mpr, since we can't cpi <= r15
    cpi mpr, 0                  ; Check if TekBot is already at min speed
    breq DecrDone               ; If so, don't do anything
    dec speed                   ; Decrement speed by one level
    ldi mpr, $60                ; Keep move forward bits set
    eor mpr, speed              ; Indicate speed on LEDs 3:0
    out PORTB, mpr              ;
    ldi mpr, 17                 ; To get 16 levels (including 0), 255/15 = 17
    mul mpr, speed              ; TekBot speed is speed level (0-15) times the increment
                                ; per level
    ldi mpr, 255                ; Load maximum speed value into mpr
    sub mpr, r0                 ; Subtract difference of maximum speed and current speed
                                ; to get duty cycle value (since minimum speed is 100% duty
                                ; cycle)
    out OCR0, mpr               ; Generate signals with new duty cycle
    out OCR2, mpr               ;
DecrDone:
    pop mpr                     ; Restore status register
    out SREG, mpr               ;
    ldi mpr, $0F                ; Clear any queued interrupts on INT3-0
    out EIFR, mpr               ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: SetMaxSpeed
; Desc: Increases the TekBot's speed to the maximum level if
;   it is not already there. This is called when the INT2
;   interrupt is triggered.
;-----------------------------------------------------------
SetMaxSpeed:
    push mpr                    ; Save mpr
    rcall DebounceInput         ; Debounce button push to prevent unwanted duplicate calls
    ldi speed, 15               ; Load maximum speed level to speed
    ldi mpr, $6F                ; Move forward and show max speed on LEDs 3:0
    out PORTB, mpr              ;
    ldi mpr, $00                ; Load 0 to mpr for a 0% duty cycle
    out OCR0, mpr               ; Generate signals with 0% duty cycle
    out OCR2, mpr               ;
    ldi mpr, $0F                ; Clear any queued interrupts on INT3-0
    out EIFR, mpr               ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: SetMinSpeed
; Desc: Decreases the TekBot's speed to the minimum level if
;   it is not already there. This is called when the INT3
;   interrupt is triggered.
;-----------------------------------------------------------
SetMinSpeed:
    push mpr                    ; Save mpr
    rcall DebounceInput         ; Debounce button push to prevent unwanted duplicate calls
    ldi speed, 0                ; Load minimum speed level to speed
    ldi mpr, $60                ; Move forward and show min speed on LEDs 3:0
    out PORTB, mpr              ;
    ldi mpr, $FF                ; Load 0xFF to mpr to 100% duty cycle
    out OCR0, mpr               ; Generate signals with 100% duty cycle
    out OCR2, mpr               ;
    ldi mpr, $0F                ; Clear any queued interrupts on INT3-0
    out EIFR, mpr               ;
    pop mpr                     ; Restore mpr
    ret                         ; Return from function

;-----------------------------------------------------------
; Func: DebounceInput
; Desc: Debounces PDn button pushes by creating a delay of
;   200 ms.
;-----------------------------------------------------------
DebounceInput:
    push waitcnt                ; Save waitcnt register
    ldi waitcnt, 21             ; Load 20 to waitcnt (for 20*10ms=200ms delay)
    rcall DelayWait             ; Create 200 ms delay
    pop waitcnt                 ; Restore waitcnt register
    ret                         ; Return from function

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
ILoop:  dec   ilcnt       ; Decrement ilcnt
    brne  ILoop           ; Continue Inner Loop
    dec   olcnt           ; Decrement olcnt
    brne  OLoop           ; Continue Outer Loop
    dec   waitcnt         ; Decrement wait 
    brne  Loop            ; Continue Wait loop  

    pop   olcnt           ; Restore olcnt register
    pop   ilcnt           ; Restore ilcnt register
    pop   waitcnt         ; Restore wait register
    ret                   ; Return from subroutine

