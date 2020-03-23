; vi: ft=avr8bit
;***********************************************************
;*
;*  Jason_Chen_Lab8_Rx_challengecode.asm
;*
;*  This program controls a robot by using the AVR board's
;*  USART1 receiver (connected to an IR receiver) to receive
;*  commands from a second remote board. The remote can
;*  send six different commands: move forward, reverse,
;*  turn right, turn left, speed up, and speed down. Each
;*  action will be performed by the robot.
;*
;*  Communication between the robot and remote are done
;*  using a packet format in which the first 8 bits
;*  are the address of the robot the remote is
;*  communicating with and the second 8 bits are the
;*  action code of the command being sent.
;*
;*  Additionally, the robot also responds to whisker hits
;*  on its left and right whiskers. When the left whisker is
;*  hit, the robot reverses for one second, turns right for
;*  one second, and resumes its previous action before the
;*  hit. If the right whisker is hit, the robot reverses for
;*  one second, turns left for one second, and resumes its
;*  previous action. In both cases, commands from the remote
;*  are ignored while the robot is reversing/turning. As
;*  part of the challenge, the whisker hit delays are
;*  performed using interrupts with Timer/Counter1.
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
.def  freezes = r18   ; Number of times robot has been frozen
.def  addrRXed = r19  ; Register indicating if robot address has been received
.def  speed = r20     ; Current speed level of the robot

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
.equ  SpeedUp =    (1<<EngEnR|1<<EngEnL)  ;0b10010000 Speed Up Action Code
.equ  SpeedDown =  0b11110000             ; Speed Down action code

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
.org  $0018         ; IV for Timer/Counter1 Compare Match A
    ijmp            ; Indirect jump to address in Z
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

    ; Configure Timer/Counter1
    ldi mpr, 0b00000000   ; Configure Timer/Counter1 with normal port operation and
    out TCCR1A, mpr       ; CTC mode
    ldi mpr, 0b00001100   ; Configure Timer/Counter1 with CTC mode, prescale of 256
    out TCCR1B, mpr       ;
    ldi mpr, high(62499)  ; Load high byte of 62499 to OCR1AH (TOP value needed for
    out OCR1AH, mpr       ; 1-second delay, found using (16MHz * 1s)/256 - 1
    ldi mpr, low(62499)   ; Load low byte of 62499 to OCR1AL
    out OCR1AL, mpr       ;

    ; Configure 8-bit Timer/Counters
    ldi mpr, 0b01101001       ; Configure Timer/Counter0 and 2 with fast PWM
                              ; wave generation mode, non-prescaled clock,
                              ; and clear OC0 on compare match
    out TCCR0, mpr            ;
    out TCCR2, mpr            ;

    ; Set initial speed, display on Port B pins 3:0
    ldi speed, 15             ; Set initial speed to 15
    ldi mpr, MovFwd           ; Make robot initially move foward
    eor mpr, speed            ; Indicate speed LED 3:0 (right 4 LEDs)
    out PORTB, mpr            ;
    ldi mpr, $00              ; Set initial duty cycle of 0% (for max speed)
    out OCR0, mpr             ;
    out OCR2, mpr             ;

    ldi action, MovFwd    ; Robot initially moves forward
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
;   loads the address to which the Timer/Counter1 interrupt
;   should jump to after a 1-second delay and then calls the
;   WhiskerHit routine to make the robot reverse. The loaded
;   address will cause the Timer/Counter1 interrupt to jump
;   to the HitRightTurn label after 1-second, which will
;   make the robot turn for one second. After that, the
;   robot will resume its previous action when the next
;   Timer/Counter1 interrupt jumps to the WhiskerHitResume
;   label.
;
;   While the robot is reversing or turning, button pushes
;   and any data from the remote are ignored by disabling
;   the respective interrupts.
;-----------------------------------------------------------
HitRight:
    ldi ZL, low(HitRightTurn)   ; Load low byte of HitRightTurn address to ZL; this
                                ; address will be used by the Timer/Counter1 Compare Match
                                ; A interrupt to jump back to the code
    ldi ZH, high(HitRightTurn)  ; Load high byte of HitRightTurn address to ZH
    rcall WhiskerHit            ; Call WhiskerHit to start reversing
    ret                         ;

;-----------------------------------------------------------
; Func: HitRightTurn
; Desc: The Timer/Counter1 Compare Match A interrupt jumps
;   here after the robot reverses for one second following
;   a right whisker hit. This routine makes the robot turn
;   left by calling WhiskerHitTurn, which will send the
;   command to the robot and also set up the Z-pointer for
;   the next 1-second interrupt.
;-----------------------------------------------------------
HitRightTurn:
    push mpr              ; Save mpr
    ldi mpr, TurnL        ; Load turn left command to mpr
    rcall WhiskerHitTurn  ; Call WhiskerHitTurn to make robot turn
    pop mpr               ; Restore mpr
    reti                  ; Return from interrupt

;-----------------------------------------------------------
; Func: HitLeft
; Desc: Called when the left whisker (PD0/INT0) is hit;
;   loads the address to which the Timer/Counter1 interrupt
;   should jump to after a 1-second delay and then calls the
;   WhiskerHit routine to make the robot reverse. The loaded
;   address will cause the Timer/Counter1 interrupt to jump
;   to the HitLeftTurn label after 1-second, which will
;   make the robot turn for one second. After that, the
;   robot will resume its previous action when the next
;   Timer/Counter1 interrupt jumps to the WhiskerHitResume
;   label.
;
;   While the robot is reversing or turning, button pushes
;   and any data from the remote are ignored by disabling
;   the respective interrupts.
;-----------------------------------------------------------
HitLeft:
    ldi ZL, low(HitLeftTurn)  ; Load low byte of HitLeftTurn address to ZL; this
                              ; address will be used by the Timer/Counter1 Compare Match
                              ; A interrupt to jump back to the code
    ldi ZH, high(HitLeftTurn) ; Load high byte of HitLeftTurn address to ZH
    rcall WhiskerHit      ; Call WhiskerHit to handle the whisker hit
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: HitLeftTurn
; Desc: The Timer/Counter1 Compare Match A interrupt jumps
;   here after the robot reverses for one second following
;   a left whisker hit. This routine makes the robot turn
;   right by calling WhiskerHitTurn, which will send the
;   command to the robot and also set up the Z-pointer for
;   the next 1-second interrupt.
;-----------------------------------------------------------
HitLeftTurn:
    push mpr              ; Save mpr
    ldi mpr, TurnR        ; Load turn right command to mpr
    rcall WhiskerHitTurn  ; Call WhiskerHitTurn to make robot turn
    pop mpr               ; Restore mpr
    reti                  ; Return from interrupt

;-----------------------------------------------------------
; Func: WhiskerHit
; Desc: This function is called by the HitRight and HitLeft
;   routines following a whisker hit. It disables the USART1
;   receiver, along with its receive complete interrupt. It
;   also disables external interrupts to prevent additional
;   whisker hits. It then makes the robot reverse and clears
;   the Timer/Counter1 TCNT1 register to reset the timer.
;   Following that, any existing Output Compare Match A
;   interrupt for the timer is cleared, and that interrupt
;   is then enabled so that it triggers after 1 second.
;-----------------------------------------------------------
WhiskerHit:
    push mpr              ; Save mpr
    push action           ; Save action
    ldi mpr, $00          ; Disable the USART1 receiver and receive complete interrupt
    sts UCSR1B, mpr       ;
    ldi mpr, $00          ; Disable interrupts from INT0 and INT1
    out EIMSK, mpr        ;

    ldi action, MovBck    ; Load reverse command to action register
    rcall UpdatePortB     ; Show new command on Port B
    ldi mpr, $00          ; Clear Timer/Counter1's TCNT1 register to count from 0 again
    out TCNT1H, mpr       ;
    out TCNT1L, mpr       ;
    ldi mpr, 1<<OCF1A     ; Write 1 to OCF1A to clear any existing Output Compare Match
                          ; A interrupt
    out TIFR, mpr         ;
    ldi mpr, 1<<OCIE1A    ; Write 1 to OCIE1A to enable Output Compare Match A interrupt
    out TIMSK, mpr        ;
    pop action            ; Restore action
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: WhiskerHitTurn
; Desc: This function makes the robot turn using the command
;   specified in mpr. It then resets Timer/Counter1 to
;   create another 1-second delay and loads the Z-pointer
;   with the address of WhiskerHitResume, so the Output
;   Compare Match A interrupt knows where to jump after the
;   next 1-second delay.
;-----------------------------------------------------------
WhiskerHitTurn:
    push mpr              ; Save mpr
    push action           ; Save action
    mov action, mpr       ; Copy turn command to action register
    rcall UpdatePortB     ; Show turn command on Port B
    ldi mpr, $00          ; Clear Timer/Counter1's TCNT1 register to count from 0 again
    out TCNT1H, mpr       ;
    out TCNT1L, mpr       ;
    ldi ZL, low(WhiskerHitResume)   ; Load low byte of WhiskerHitResume address to ZL; this
                                    ; address will be used by the Timer/Counter1 Compare Match
                                    ; A interrupt to jump back to the code
    ldi ZH, high(WhiskerHitResume)  ; Load high byte of WhiskerHitResume address to ZH
    pop action            ; Restore action
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: WhiskerHitResume
; Desc: This routine makes the robot function normally again
;   after it finishes reversing and turning. The Output
;   Compare Match A interrupt jumps here when it triggers
;   the second time (so after 2 seconds). The routine clears
;   any queued external interrupts and disables the
;   Timer/Counter1 Output Compare Match A interrupt. It then
;   re-enables the USART1 receiver and its interrupt, along
;   with external interrupts for whisker hits. That makes
;   the robot function as normal again.
;-----------------------------------------------------------
WhiskerHitResume:
    push mpr              ; Save mpr
    rcall UpdatePortB     ; Resume previous action
    ldi mpr, 0b00000011   ; Clear queued interrupts for INT1 and INT0
    out EIFR, mpr         ;
    ldi mpr, $00          ; Disable the Timer/Counter1 Output Compare Match A interrupt
    out TIMSK, mpr        ;
    ldi mpr, (1<<RXCIE1|1<<RXEN1) ; Re-enable USART1 receiver and receive interrupt
    sts UCSR1B, mpr               ;
    ldi mpr, 0b00000011   ; Re-enable interrupts for INT0 and INT1
    out EIMSK, mpr        ;
    pop mpr               ; Restore mpr
    reti                  ; Return from interrupt

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
    brne CheckSpeedUp     ; If not, check for halt
    rjmp PerformAction    ; Jump to PerformAction to output TurnL action
CheckSpeedUp:
    cpi action, SpeedUp   ; Check if action is speed up
    brne CheckSpeedDown   ; If not, check for speed down
    in action, PORTB      ; Restore robot's current action, since speed shouldn't
                          ; affect what the robot is currently doing
    cbr action, $0F       ; Clear the lower four bits that indicate speed
    rcall IncrSpeed       ; Call IncrSpeed to increment and display new speed
    rjmp HandleDone       ; Jump to HandleDone since IncrSpeed calls UpdatePortB
CheckSpeedDown:
    cpi action, SpeedDown ; Check if action is speed down
    brne HandleDone       ; If not, the action is invalid
    in action, PORTB      ; Restore robot's current action, since speed shouldn't
                          ; affect what the robot is currently doing
    cbr action, $0F       ; Clear the lower four bits that indicate speed
    rcall DecrSpeed       ; Call DecrSpeed to decrement and display new speed
    rjmp HandleDone       ; Jump to HandleDone since IncrSpeed calls UpdatePortB
PerformAction:
    rcall UpdatePortB     ; Send the given action to Port B
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
;   it's 0, it checks if the data is equal to the robot's
;   address. If so, the addrRXed register is set to indicate
;   that the robot's address has been received.
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
    cpi mpr, BotAddress   ; Check if received data is the robot's address
    brne ClearRXed        ; If not, branch to ClearRXed
    breq AddressReceived  ; If so, branch to AddressReceived
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

;-----------------------------------------------------------
; Func: UpdatePortB
; Desc: Updates the value of PORTB based on the values of
;   the action and speed registers. The action register is
;   uses the upper four bits of PORTB, while the speed
;   value takes the lower four bits.
;-----------------------------------------------------------
UpdatePortB:
    push mpr              ; Save mpr
    mov mpr, action       ; Copy action to mpr
    eor mpr, speed        ; XOR speed with mpr (modifies lower 4 bits)
    out PORTB, mpr        ; Send mpr to PORTB
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: SetDutyCycle
; Desc: Sets the duty cycle of the PWM wave generated by
;   Timer/Counters 1 and 2 based on the value of the speed
;   register. There are 16 speed levels (duty cycles), so
;   each increment/decrement of speed represents about a
;   6.7% change in duty cycle.
;-----------------------------------------------------------
SetDutyCycle:
    push mpr              ; Save mpr
    push r0               ; Save r0
    push r1               ; Save r1
    ldi mpr, 17           ; Load 17 (increment per speed level) to mpr
    mul mpr, speed        ; Multiply speed by 17
    ldi mpr, 255          ; Load 255 (max 8-bit integer) to mpr
    sub mpr, r0           ; Subtract 255 from speed by 17 to get active low duty cycle
    out OCR0, mpr         ; Send duty cycle (mpr) to OCR0
    out OCR2, mpr         ; Send duty cycle (mpr) to OCR2
    pop r1                ; Restore r1
    pop r0                ; Restore r0
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: IncrSpeed
; Desc: Increments the current speed of the TekBot by one
;   level, if it is not already at the maximum speed, and
;   then updates Port B based on the new speed.
;-----------------------------------------------------------
IncrSpeed:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;
    cpi speed, 15         ; Check if speed is max (15)
    breq IncrDone         ; If so, jump to IncrDone
    inc speed             ; Increment the speed level
    rcall UpdatePortB     ; Send new value based on speed to Port B
    rcall SetDutyCycle    ; Set new duty cycle based on incremented speed
IncrDone:
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
    ret                   ; Return from function

;-----------------------------------------------------------
; Func: DecrSpeed
; Desc: Decrements the current speed of the TekBot by one
;   level, if it is not already at the minimum speed, and
;   then updates Port B based on the new speed.
;-----------------------------------------------------------
DecrSpeed:
    push mpr              ; Save mpr
    in mpr, SREG          ; Save status register
    push mpr              ;
    cpi speed, 0          ; Check if speed is min (0)
    breq DecrDone         ; If so, jump to DecrDone
    dec speed             ; Decrement the speed level
    rcall UpdatePortB     ; Send new value based on speed to Port B
    rcall SetDutyCycle    ; Set new duty cycle based on decremented speed
DecrDone:
    pop mpr               ; Restore status register
    out SREG, mpr         ;
    pop mpr               ; Restore mpr
    ret                   ; Return from function

