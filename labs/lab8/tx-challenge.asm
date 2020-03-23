; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab8_Tx_challengecode.asm
;*
;*  This program remotely controls a robot by using the AVR
;*  board's USART1 transmitter (connected to an IR
;*  transmitter) to send commands to the robot.
;*  Communication with the robot occurs using a 16-bit
;*  packet structure in which the first byte is the address
;*  of the robot to which the packet is addressed and the
;*  second byte is the command that the robot should perform.
;*
;*  The remote can send six different commands to the robot:
;*  move forward, reverse, turn right, turn left, speed up,
;*  and speed down. These actions are mapped to the push buttons
;*  PD0, PD1, PD4, PD5, PD6, and PD7, respectively. Pushing a
;*  button will send the respective command to the robot.
;*
;***********************************************************
;*
;*   Author: Jason Chen
;*     Date: March 1, 2020
;*
;***********************************************************

.include "m128def.inc"      ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def  mpr = r16       ; Multi-Purpose Register
.def  action = r17    ; Action to transmit
.def  waitcnt = r18   ; Wait Loop Counter
.def  ilcnt = r19     ; Inner Loop Counter
.def  olcnt = r20     ; Outer Loop Counter

.equ  EngEnR = 4      ; Right Engine Enable Bit
.equ  EngEnL = 7      ; Left Engine Enable Bit
.equ  EngDirR = 5     ; Right Engine Direction Bit
.equ  EngDirL = 6     ; Left Engine Direction Bit

; Action codes for communication between remote and robot
; Control signals are shifted right by one and ORed with $80
.equ  MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1)) ;0b10110000 Move Forward Action Code
.equ  MovBck =  ($80|$00)                     ;0b10000000 Move Backward Action Code
.equ  TurnR =   ($80|1<<(EngDirL-1))          ;0b10100000 Turn Right Action Code
.equ  TurnL =   ($80|1<<(EngDirR-1))          ;0b10010000 Turn Left Action Code
.equ  SpeedUp =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))    ;0b11001000 SpeedUp Action Code
.equ  SpeedDown = 0b11111000  ; Action code for speed up

.equ BotAddress = $7C ; Address of robot paired with this remote

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg             ; Beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org  $0000         ; Beginning of IVs
    rjmp  INIT      ; Reset interrupt

.org  $0046         ; End of Interrupt Vectors

;***********************************************************
;*  Program Initialization
;***********************************************************
INIT:
    ; Initialize stack pointer
    ldi mpr, low(RAMEND)    ; Load low byte of RAMEND to SPL
    out SPL, mpr            ;
    ldi mpr, high(RAMEND)   ; Load high byte of RAMEND to SPH
    out SPH, mpr            ;

    ; I/O Ports
    ldi mpr, $00            ; Make all Port D pins inputs
    out DDRD, mpr           ;
    ldi mpr, $FF            ; Enable pull-up resistor for all Port D pins
    out PORTD, mpr          ;

    ; Configure USART1
    ; Set baud rate at 2400bps
    ldi mpr, high(832)    ; Load high nibble of 832 (UBRR value)
    sts UBRR1H, mpr       ; to UBRR1H
    ldi mpr, low(832)     ; Load low byte of 832 (UBRR value)
    sts UBRR1L, mpr       ; to UBRR1L

    ldi mpr, 1<<U2X1      ; Double USART transmission speed (for double data rate)
    sts UCSR1A, mpr       ;

    ; Enable transmitter
    ldi mpr, 1<<TXEN1     ; Set TXEN1 bit of UCSR1B
    sts UCSR1B, mpr       ;

    ;Set frame format: 8 data bits, 2 stop bits
    ldi mpr, 0b00001110   ; Configure USART1 with async operation, disabled
                          ; parity, 2 stop bits, 8-bit data
    sts UCSR1C, mpr       ;

    clr action            ; Clear action register

;***********************************************************
;*  Main Program
;***********************************************************
MAIN:                     ; The Main program
    in  mpr, PIND         ; Get current state of port D pins
    andi mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD6|1<<PD7) ; Check for active low input
    cpi mpr, (1<<PD1|1<<PD4|1<<PD5|1<<PD6|1<<PD7) ; Check if PD0 pressed
    brne CheckPD1         ; PD0 not pressed so check for PD1
    rcall HandlePD0       ; PD0 is hit; call HandlePD0 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input
CheckPD1:
    cpi mpr, (1<<PD0|1<<PD4|1<<PD5|1<<PD6|1<<PD7) ; Check if PD1 pressed
    brne CheckPD4         ; PD1 not pressed so check for PD4
    rcall HandlePD1       ; PD1 is hit; call HandlePD1 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input
CheckPD4:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD5|1<<PD6|1<<PD7) ; Check if PD4 pressed
    brne CheckPD5         ; PD4 not pressed so check for PD5
    rcall HandlePD4       ; PD4 is hit; call HandlePD4 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input
CheckPD5:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD6|1<<PD7) ; Check if PD5 pressed
    brne CheckPD6         ; PD5 not pressed so check for PD6
    rcall HandlePD5       ; PD5 is hit; call HandlePD5 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input
CheckPD6:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD7) ; Check if PD6 pressed
    brne CheckPD7         ; PD6 not pressed, so check for PD7
    rcall HandlePD6       ; PD6 is hit; call HandlePD6 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input
CheckPD7:
    cpi mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD6) ; Check if PD7 pressed
    brne MAIN             ; PD7 not pressed so jump to main, beginning of loop
    rcall HandlePD7       ; PD7 is hit; call HandlePD7 function
    rcall DebounceInput   ; Debounces the input buttons using a fixed delay
    rjmp MAIN             ; Infinite loop; continue polling input

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: TransmitAction
; Desc: Transmits the action stored in the action register
;   using the USART1 transmitter by sending two packets:
;   first the robot address and then the action.
;-----------------------------------------------------------
TransmitAction:
    push mpr              ; Save mpr
WaitToTransmitAddr:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, UDRE1       ; Check if data register is empty (ready for TX)
    rjmp WaitToTransmitAddr ; If not, loop and check again
    ldi mpr, BotAddress   ; Load the robot's address into mpr
    sts UDR1, mpr         ; Transmit the robot's address by writing to UDR1
WaitToTransmitAction:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, UDRE1       ; Check if data register is empty (ready for TX)
    rjmp WaitToTransmitAction ; If not, loop and check again
    sts UDR1, action      ; Transmit the action by writing to UDR1
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD0
; Desc: Called when the PD0 button is pushed; sends the
;   move forward command to the robot.
;-----------------------------------------------------------
HandlePD0:
    ldi action, MovFwd    ; Load move forward command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD1
; Desc: Called when the PD1 button is pushed; sends the
;   move back (reverse) command to the robot.
;-----------------------------------------------------------
HandlePD1:
    ldi action, MovBck    ; Load reverse command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD4
; Desc: Called when the PD4 button is pushed; sends the
;   turn right command to the robot.
;-----------------------------------------------------------
HandlePD4:
    ldi action, TurnR     ; Load turn right command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD5
; Desc: Called when the PD5 button is pushed; sends the
;   turn left command to the robot.
;-----------------------------------------------------------
HandlePD5:
    ldi action, TurnL     ; Load turn left command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD6
; Desc: Called when the PD0 button is pushed; sends the
;   speed up command to the robot.
;-----------------------------------------------------------
HandlePD6:
    ldi action, SpeedUp   ; Load speed up command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandlePD7
; Desc: Called when the PD7 button is pushed; sends the
;   speed down command to the robot.
;-----------------------------------------------------------
HandlePD7:
    ldi action, SpeedDown ; Load speed down command to action register
    rcall TransmitAction  ; Call TransmitAction to send the action
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: DebounceInput
; Desc: Debounces PDn button pushes by creating a delay of
;   200 ms.
;-----------------------------------------------------------
DebounceInput:
    push waitcnt          ; Save waitcnt register
    ldi waitcnt, 20       ; Load 20 to waitcnt
    rcall Wait            ; Call Wait to create 200 ms delay
    pop waitcnt           ; Restore waitcnt register
    ret                   ; Return from function

;----------------------------------------------------------------
; Sub: Wait
; Desc: A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;   waitcnt*10ms.  Just initialize wait for the specific amount 
;   of time in 10ms intervals. Here is the general eqaution
;   for the number of clock cycles in the wait loop:
;     ((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;
;   (This function is taken from the BumpBot example program on
;   the lab website.)
;----------------------------------------------------------------
Wait:
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

