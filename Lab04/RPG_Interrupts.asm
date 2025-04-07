;
; RPG_Interrupts.asm
;
; Created: 4/1/2025
; Author : Moises Gomez
;
; ASM file to implement RPG counter-clockwise and clockwise
; detection using interrupts
;

; The below is tweaked ChatGPT code

.include "m328pdef.inc"


.equ RPG_A = 0         ; PB0 (RPG A, PCINT0)
.equ RPG_B = 1         ; PB1 (RPG B, PCINT1)
.equ TEST_PIN = 5      ; PD5 (Test output)

.org 0x0000
rjmp RESET         ; Reset vector (or main init label)
.org 0x0006
rjmp RPG_ISR       ; Pin Change Interrupt 0 vector

RESET:
    ldi R16, low(RAMEND)
    out SPL, R16
    ldi R16, high(RAMEND)
    out SPH, R16
    ; Enable PCINT on PB0 and PB1 (PCINT0 & PCINT1)
    cbi DDRB, RPG_A         ; PB0 (RPG A) - Input
    cbi DDRB, RPG_B         ; PB1 (RPG B) - Input
    sbi PORTB, RPG_A        ; Enable pull-up
    sbi PORTB, RPG_B        ; Enable pull-up

    sbi DDRD, TEST_PIN         ; PD5 (Test) - Output

    ; Enable Pin Change Interrupts
    ldi R24, (1 << PCIE0)  ; Enable PCINT[7:0]
    sts PCICR, R24


    ; Initialize previous RPG state (R20)
    clr R21
    sbic PINB, RPG_A
    ori R21, (1 << 1)
    sbic PINB, RPG_B
    ori R21, (1 << 0)
    mov R20, R21

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
    sbic PINB, RPG_A         ; If PB0 (RPG A) is high, set bit 1
    ori R21, (1 << 1)
    sbic PINB, RPG_B         ; If PB1 (RPG B) is high, set bit 0
    ori R21, (1 << 0)

    ; Compare current state (R21) with previous state (R20)
    cp R21, R20
    breq read_complete

    ; Encode previous + current state into 4 bits (2-bit shift)
    lsl R20
    lsl R20
    or R20, R21

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

    rjmp updateState
RPG_rotated_CW:
    ldi R22, 1
    rjmp updateState

RPG_rotated_CCW:
    ldi R22, 2
    rjmp updateState

updateState:
    mov R20, R21         ; Save current state as previous
read_complete:
    ret

clockwise:
    sbi PORTD, TEST_PIN
    rjmp RPG_DONE
counterclockwise:
    cbi PORTD, TEST_PIN
	rjmp RPG_DONE