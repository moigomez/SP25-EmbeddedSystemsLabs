;
; RPG_Interrupts.asm
;
; Created: 4/1/2025
; Author : Moises Gomez
;
; ASM file to implement RPG counter-clockwise and clockwise
; detection using interrupts
;

.include "m328pdef.inc"

.equ RPG_A = 0        ; PB0 - PCINT0
.equ RPG_B = 1        ; PB1 - PCINT1
.equ TEST_PIN = 5     ; PD5 - Testing purposes

.def regPrevious = R19
.def regCurrent = R16

.org 0x0000
rjmp RESET         ; Reset vector (or main init label)
.org 0x0006
rjmp RPG_ISR       ; Pin Change Interrupt 0 vector

RESET:
    ldi R16, low(RAMEND)
    out SPL, R16
    ldi R16, high(RAMEND)
    out SPH, R16

    cbi DDRB, RPG_A            ; Set PB0 as input
    cbi DDRB, RPG_B            ; Set PB1 as input
    sbi PORTB, RPG_A           ; Enable pull-up on PB0
    sbi PORTB, RPG_B           ; Enable pull-up on PB1

    sbi DDRD, TEST_PIN         ; PD5 (Test) - Output

    ldi R16, (1 << PCINT0) | (1 << PCINT1)
    sts PCMSK0, R16
    ldi R16, (1 << PCIE0)
    sts PCICR, R16

    clr regPrevious
    clr regCurrent

    sbic PINB, RPG_A
    ori regPrevious, (1 << 1)   ; Save A to bit 1
    sbic PINB, RPG_B
    ori regPrevious, (1 << 0)   ; Save B to bit 0

    sei

main_loop:
    rjmp main_loop

RPG_ISR:
    ; Get current state
    in  regCurrent, PINB
    andi regCurrent, 3      ; Mask for PB0 and PB1 (bits 1:0)

    mov R21, R19
    lsl R21                 ; Shift left twice
    lsl R21
    or  R21, R16
    mov R20, R21

    cpi R20, 0b0001
    breq clockwise
    cpi R20, 0b0111
    breq clockwise
    cpi R20, 0b1110
    breq clockwise
    cpi R20, 0b1000
    breq clockwise

    cpi R20, 0b0010
    breq counterclockwise
    cpi R20, 0b1011
    breq counterclockwise
    cpi R20, 0b1101
    breq counterclockwise
    cpi R20, 0b0100
    breq counterclockwise

    rjmp save_state
clockwise:
    inc R23
    cpi R23, 4              ; if (R23 == 4) {
    brne save_state         ;
    clr R23                 ; 
    sbi PORTD, TEST_PIN     ;     PD5.high() 
    rjmp save_state         ; }

counterclockwise:
    inc R23              
    cpi R23, 4              ; if (R23 == 4) {
    brne save_state         ;
    clr R23                 ;
    cbi PORTD, TEST_PIN     ;     PD5.low()
                            ; }
save_state:
    mov regPrevious, regCurrent
    reti