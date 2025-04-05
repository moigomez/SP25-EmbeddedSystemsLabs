;
; RPG_Interrupts.asm
;
; Created: 4/1/2025
; Author : Moises Gomez
;
; ASM file to implement RPG counter-clockwise and clockwise
; detection using interrupts
;

; The below is ChatGPT code

.include "m328pdef.inc"

; Enable PCINT on PB4 and PB5 (PCINT4 & PCINT5)
cbi DDRB, 0         ; PB0 (RPG A) - Input
cbi DDRB, 1         ; PB1 (RPG B) - Input
sbi PORTB, 0        ; Enable pull-up
sbi PORTB, 1        ; Enable pull-up

sbi DDRD, 5         ; PD5 (Test) - Output

; Enable Pin Change Interrupts
ldi R24, (1 << PCIE0)  ; Enable PCINT[7:0]
sts PCICR, R24

; Enable PCINT0 and PCINT1
ldi R24, (1 << PCINT0) | (1 << PCINT1)
sts PCMSK0, R24

sei   ; Enable global interrupts

main_loop:
    rjmp main_loop

; Interrupt Service Routine for Rotary Encoder (PCINT0)
RPG_ISR:
    push R20
    push R21
    push R22

    rcall read_RPG_direction  ; Reuse your existing function

    ; Check if RPG is resting on detent and update count accordingly
    cpi R20, 3
    brne RPG_DONE
    cpi R22, 1
    breq clockwise
    cpi R22, 2
    breq counterclockwise

RPG_DONE:
    pop R22
    pop R21
    pop R20
    reti  ; Return from interrupt

read_RPG_direction:
    clr R21              ; Clear temporary register
    sbic PINB, 0         ; If PB0 (RPG A) is high, set bit 1
    ori R21, (1 << 1)
    sbic PINB, 1         ; If PB1 (RPG B) is high, set bit 0
    ori R21, (1 << 0)

    ; Compare current state (R21) with previous state (R20)
    cp R21, R20
    breq read_complete    ; If no change, exit

    ; Encode previous + current state into 4 bits (2-bit shift)
    lsl R20
    lsl R20
    or R20, R21           ; Combine with new state

    ; Check transition pattern using known quadrature sequences
    cpi R20, 0b0001
    breq RPG_rotated_CCW
    cpi R20, 0b0111
    breq RPG_rotated_CCW
    cpi R20, 0b1110
    breq RPG_rotated_CCW
    cpi R20, 0b1000
    breq RPG_rotated_CCW

    cpi R20, 0b0010
    breq RPG_rotated_CW
    cpi R20, 0b0100
    breq RPG_rotated_CW
    cpi R20, 0b1101
    breq RPG_rotated_CW
    cpi R20, 0b1011
    breq RPG_rotated_CW

    rjmp updateState     ; No valid movement, update last state
RPG_rotated_CW:
    ldi R22, 1           ; Clockwise rotation detected
    rjmp updateState

RPG_rotated_CCW:
    ldi R22, 2           ; Counterclockwise rotation detected
    rjmp updateState

updateState:
    mov R20, R21         ; Save current state as previous
read_complete:
    ret

clockwise:
    sbi PORTD, 5
    rjmp RPG_DONE
counterclockwise:
    cbi PORTD, 5
	rjmp RPG_DONE