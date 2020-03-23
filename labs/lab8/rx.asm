; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab8_Rx_sourecode.asm
;*
;*  This program controls a robot by using the AVR board's
;*  USART1 receiver (connected to an IR receiver) to receive
;*  commands from a second remote board. The remote can
;*  send six different commands: move forward, reverse,
;*  turn right, turn left, halt, and freeze. For the first
;*  five commands, the robot will perform the respective
;*  action. For the sixth command, freeze, the robot will
;*  enable the USART1 transmitter and transmit the freeze
;*  signal, which should freeze any robots in the
;*  surrounding area that pick up the IR transmission.
;*
;*  Communication between the robot and remote are done
;*  using a packet format in which the first 8 bits
;*  are the address of the robot the remote is
;*  communicating with and the second 8 bits are the
;*  action code of the command being sent. The only
;*  exception is when the robot receives a freeze signal,
;*  in which case, no robot address is required and the
;*  robot freezes for 5 seconds. If the robot receives the
;*  freeze signal a third time, it will freeze forever.
;*
;*  Additionally, the robot also responds to whisker hits
;*  on its left and right whiskers. When the left whisker is
;*  hit, the robot reverses for one second, turns right for
;*  one second, and resumes its previous action before the
;*  hit. If the right whisker is hit, the robot reverses for
;*  one second, turns left for one second, and resumes its
;*  previous action. In both cases, commands from the remote
;*  are ignored while the robot is reversing/turning.
;*
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
.def  action = r17    ; Robot's current action
.def  freezes = r21   ; Number of times robot has been frozen
.def  addrRXed = r22  ; Register indicating if robot address has been received
.def  waitcnt = r18   ; Wait Loop Counter
.def  ilcnt = r19     ; Inner Loop Counter
.def  olcnt = r20     ; Outer Loop Counter

.equ  WskrR = 0       ; Right Whisker Input Bit
.equ  WskrL = 1       ; Left Whisker Input Bit
.equ  EngEnR = 4      ; Right Engine Enable Bit
.equ  EngEnL = 7      ; Left Engine Enable Bit
.equ  EngDirR = 5     ; Right Engine Direction Bit
.equ  EngDirL = 6     ; Left Engine Direction Bit

.equ BotAddress = $7C ; Arbitrary address for the robot

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////
.equ  MovFwd =  (1<<EngDirR|1<<EngDirL) ;0b01100000 Move Forward Action Code
.equ  MovBck =  $00                     ;0b00000000 Move Backward Action Code
.equ  TurnR =   (1<<EngDirL)            ;0b01000000 Turn Right Action Code
.equ  TurnL =   (1<<EngDirR)            ;0b00100000 Turn Left Action Code
.equ  Halt =    (1<<EngEnR|1<<EngEnL)   ;0b10010000 Halt Action Code
.equ  Freeze =  0b11110000              ; Freeze action code

.equ  FreezeSignal = 0b01010101         ; Freeze signal

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg             ; Beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org  $0000         ; Beginning of IVs
    rjmp  INIT      ; Reset interrupt
.org  $0002         ; IV for INT0
    rcall HitRight  ; Right whisker hit
    reti            ; Return from interrupt
.org  $0004         ; IV for INT1
    rcall HitLeft   ; Left whisker hit
    reti            ; Return from interrupt
.org  $003C         ; IV for USART1, RX
    rcall DataRX    ; Handle received data
    reti            ; Return from interrupt

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
    ldi mpr, $FF            ; Make all Port B pins outputs
    out DDRB, mpr           ;

    ; Configure USART1
    ; Set baud rate at 2400bps
    ldi mpr, high(832)    ; Load high nibble of 832 (UBRR value)
    sts UBRR1H, mpr       ; to UBRR1H
    ldi mpr, low(832)     ; Load low byte of 832 (UBRR value)
    sts UBRR1L, mpr       ; to UBRR1L

    ldi mpr, 1<<U2X1      ; Double USART transmission speed (for double data rate)
    sts UCSR1A, mpr       ;

    ; Enable receiver and receive interrupts
    ldi mpr, (1<<RXCIE1|1<<RXEN1) ; Set RXCIE1 and RXEN1 bits of UCSR1B
    sts UCSR1B, mpr               ;

    ; Set frame format: 8 data bits, 2 stop bits
    ldi mpr, 0b00001110   ; Configure USART1 with async operation, disabled
                          ; parity, 2 stop bits, 8-bit data, and received data
                          ; sampled on falling edge
    sts UCSR1C, mpr       ;

    ; Configure External Interrupts
    ldi mpr, 0b00000011   ; Enable interrupts INT0 and INT1
    out EIMSK, mpr        ;
    ; Set the Interrupt Sense Control to falling edge detection
    ldi mpr, 0b00001010   ; Set falling-edge triggers for INT0 and INT1
    sts EICRA, mpr        ;

    ldi action, MovFwd    ; Make robot initially move forward
    out PORTB, action     ;
    clr addrRXed          ; Clear addrRXed register
    clr freezes           ; Clear freezes register

    sei                   ; Set global interrupt enable bit to enable interrupts

;***********************************************************
;*  Main Program
;***********************************************************
MAIN:                     ; The Main program
    rjmp  MAIN            ; Jump back to MAIN to loop forever

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: HitRight
; Desc: Called when the right whisker (PD0/INT0) is hit;
;   reverses for one second, turns left for one second,
;   then resumes whatever action it was doing before the
;   hit. Any commands from the remote are ignored while
;   the robot is reversing/turning.
;-----------------------------------------------------------
HitRight:
    push mpr              ; Save mpr

    ldi mpr, MovBck       ; Load reverse command to mpr
    out PORTB, mpr        ; Send reverse command to Port B
    ldi waitcnt, 100      ; Wait (reverse) for one second
    rcall Wait            ;

    ldi mpr, TurnL        ; Load turn left command to mpr
    out PORTB, mpr        ; Send turn left command to Port B
    ldi waitcnt, 100      ; Wait (turn) for one second
    rcall Wait            ;

    out PORTB, action     ; Resume previous action
; Flushes the USART1 receive buffer to prevent data received
; during this routine from triggering interrupt
HitRightFlushUSART:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, RXC1        ; Check if RXC1 bit is set
    rjmp HitRightFlushed    ; If not, don't need to flush anything
    lds mpr, UDR1         ; If so, read UDR1 (until RXC1 is set)
    rjmp HitRightFlushUSART ; Jump to HitRightFlushUSART and check again
HitRightFlushed:
    ldi mpr, 0b00000011   ; Clear queued interrupts for INT1 and INT0
    out EIFR, mpr         ;

    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HitLeft
; Desc: Called when the left whisker (PD0/INT0) is hit;
;   reverses for one second, turns right for one second,
;   then resumes whatever action it was doing before the
;   hit. Any commands from the remote are ignored while
;   the robot is reversing/turning.
;-----------------------------------------------------------
HitLeft:
    push mpr              ; Save mpr

    ldi mpr, MovBck       ; Load reverse command to mpr
    out PORTB, mpr        ; Send reverse command to Port B
    ldi waitcnt, 100      ; Wait (reverse) for one second
    rcall Wait            ;

    ldi mpr, TurnR        ; Load turn right command to mpr
    out PORTB, mpr        ; Send turn right command to Port B
    ldi waitcnt, 100      ; Wait (turn) for one second
    rcall Wait            ;

    out PORTB, action     ; Resume previous action
; Flushes the USART1 receive buffer to prevent data received
; during this routine from triggering interrupt
HitLeftFlushUSART:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, RXC1        ; Check if RXC1 bit is set
    rjmp HitLeftFlushed   ; If not, don't need to flush anything
    lds mpr, UDR1         ; If so, read UDR1 (until RXC1 is set)
    rjmp HitLeftFlushUSART  ; Jump to HitLeftFlushUSART and check again
HitLeftFlushed:
    ldi mpr, 0b00000011   ; Clear queued interrupts for INT1 and INT0
    out EIFR, mpr         ;

    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: TransmitFreeze
; Desc: Called when the freeze action code is received from
;   the remote. The USART1 transmitter is enabled, while the
;   receiver is disabled to prevent it from receiving its
;   own freeze signal, and the freeze signal is transmitted
;   (without any bot address). The transmitter is then
;   disabled again.
;-----------------------------------------------------------
TransmitFreeze:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;
    lds mpr, UCSR1B       ; Load UCSR1B to mpr
    sbr mpr, 1<<TXEN1     ; Set the TXEN1 bit to enable transmitter
    cbr mpr, (1<<RXCIE1|1<<RXEN1) ; Clear RXCIE1/RXEN1 bits to disable receiver
    sts UCSR1B, mpr       ; Write mpr to UCSR1B
WaitToTransmit:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, UDRE1       ; Check if the data register empty flag is set
    rjmp WaitToTransmit   ; If not, jump to WaitToTransmit and check again
    ldi mpr, FreezeSignal ; If so, load the freeze signal to mpr
    sts UDR1, mpr         ; Write the freeze signal to UDR1 to transmit it
WaitForTransmitComplete:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, TXC1        ; Check if transmit has completed (TXC1 is set)
    rjmp WaitForTransmitComplete  ; If not, jump to WaitForTransmitComplete and check again
    ldi mpr, (1<<TXC1|1<<U2X1)    ; Clear the TXC1 flag (by writing 1); keep U2X1 bit set
    sts UCSR1A, mpr               ; in UCSR1A register

    lds mpr, UCSR1B       ; Load UCSR1B to mpr
    sbr mpr, (1<<RXCIE1|1<<RXEN1) ; Set RXCIE1/RXEN1 bits to re-enable receiver
    cbr mpr, 1<<TXEN1             ; Clear TXEN1 bit to disable transmitter
    sts UCSR1B, mpr       ; Write mpr to UCSR1B
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: PerformFreeze
; Desc: Called when a freeze signal is received from another
;   robot. If this is the third time the robot has been
;   frozen, the robot will stay frozen forever. Otherwise,
;   the robot freezes (halts) for five seconds. Any data
;   received by the USART1 receiver and whisker hits are
;   ignored while the robot is frozen.
;-----------------------------------------------------------
PerformFreeze:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;
    push waitcnt          ; Save waitcnt register

    inc freezes           ; Increment the freezes counter
    ldi mpr, Halt         ; Load halt command to mpr
    out PORTB, mpr        ; Send halt command to Port B
    cpi freezes, 3        ; Check if this is the 3rd time robot has been frozen
    brlt FreezeFive       ; If not, jump to FreezeFive
FreezeForever:
    rjmp FreezeForever    ; If so, freeze forever by looping here forever
FreezeFive:
    ldi waitcnt, 250      ; Wait (halt) for 5 seconds
    rcall Wait            ; Waits for 2.5 seconds
    rcall Wait            ; Waits for another 2.5 seconds
    out PORTB, action     ; Resume previous action before freeze
; Flushes the USART1 receive buffer to prevent data received
; during this routine from triggering interrupt
FreezeFlushUSART:
    lds mpr, UCSR1A       ; Load UCSR1A to mpr
    sbrs mpr, RXC1        ; Check if RXC1 bit is set
    rjmp FreezeFlushed    ; If not, don't need to flush anything
    lds mpr, UDR1         ; If so, read UDR1 (until RXC1 is set)
    rjmp FreezeFlushUSART ; Jump to FreezeFlushUSART and check again
FreezeFlushed:
    ldi mpr, 0b00000011   ; Clear queued interrupts for INT1 and INT0
    out EIFR, mpr         ;

    pop waitcnt           ; Restore waitcnt register
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HandleAction
; Desc: Makes the robot perform the action stored in the
;   action register, provided it is valid. It does this by
;   sending the contents of the action register to Port B.
;-----------------------------------------------------------
HandleAction:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;

    cpi action, MovFwd    ; Check if action is move forward
    brne CheckReverse     ; If not, check for reverse
    rjmp PerformAction    ; Jump to PerformAction to output MovFwd action
CheckReverse:
    cpi action, MovBck    ; Check if action is reverse
    brne CheckRightTurn   ; If not, check for right turn
    rjmp PerformAction    ; Jump to PerformAction to output MovBck action
CheckRightTurn:
    cpi action, TurnR     ; Check if action is right turn
    brne CheckLeftTurn    ; If not, check for left turn
    rjmp PerformAction    ; Jump to PerformAction to output TurnR action
CheckLeftTurn:
    cpi action, TurnL     ; Check if action is left turn
    brne CheckHalt        ; If not, check for halt
    rjmp PerformAction    ; Jump to PerformAction to output TurnL action
CheckHalt:
    cpi action, Halt      ; Check if action is halt
    brne CheckFreeze      ; If not, check for freeze
    rjmp PerformAction    ; Jump to PerformAction to output Halt action
CheckFreeze:
    cpi action, Freeze    ; Check if action is freeze
    brne HandleDone       ; If not, the action is invalid
    rcall TransmitFreeze  ; Call TransmitFreeze to transmit freeze signal
    in action, PORTB      ; Store the robot's current action from Port B into action register
    rjmp HandleDone       ; Skip the out instruction below since it's unnecessary
PerformAction:
    out PORTB, action     ; Send the given action to Port B
HandleDone:
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: DataRX
; Desc: This function is called when the USART1, RX
;   interrupt is triggered, meaning data has been received
;   and is ready to be read. It first looks at the MSB; if
;   it's 0, it checks whether the data is the robot's
;   address or a freeze signal. In the first case, if the
;   received value matches the robot's address, the routine
;   sets the addrRXed register to indicate that the robot's
;   address has been received; in the second, it freezes.
;
;   When the MSB is 1, if the robot's address was received
;   as the last packet, the given action is performed.
;-----------------------------------------------------------
DataRX:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;
    push r23              ; Save r23

    lds r23, UDR1         ; Load UDR1 (received data) into r23
    mov mpr, r23          ; Copy r23 to mpr
    andi r23, 1<<7        ; Use mask to get MSB of r23
    breq MSBZeroReceived  ; If MSB is 0, branch to MSBZeroReceived
    brne ActionReceived   ; If MSB is 1, branch to ActionReceived
MSBZeroReceived:
    cpi mpr, BotAddress     ; Check if received data is the robot's address
    brne CheckFreezeSignal  ; If not, branch to CheckFreezeSignal
    breq AddressReceived    ; If so, branch to AddressReceived
CheckFreezeSignal:
    cpi mpr, FreezeSignal ; Check if the received data is the freeze signal
    brne ClearRXed        ; If not, jump to ClearRXed
    rcall PerformFreeze   ; If so, call PerformFreeze to freeze the robot
    rjmp ClearRXed        ; Jump to ClearRXed
AddressReceived:
    ser addrRXed          ; The received packet is the robot's address, so set
                          ; the addrRXed register
    rjmp RXDone           ; Jump to RXDone
ActionReceived:
    tst addrRXed          ; Check if the previous received packet was robot's address
    breq RXDone           ; If not, don't perform action (jump to RXDone)
    mov action, mpr       ; Otherwise, copy the received data to the action register
    lsl action            ; Left shift the data to get the command (action code)
    rcall HandleAction    ; Call HandleAction to perform the received action
ClearRXed:
    clr addrRXed          ; Clear the addrRXed register; code gets here if the received
                          ; packet is not the robot's address, which indicates for the next
                          ; received packet that the previous one was not the robot's address
RXDone:
    pop r23               ; Restore r23
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
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

