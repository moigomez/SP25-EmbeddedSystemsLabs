;
; Initialization.asm
;
; Created: 3/27/2025
; Author : Moises Gomez
;
; ASM file to initialize PWM functionality for fan
;

.include "m328pdef.inc" 

.cseg
.org 0x00
rjmp start

.org 0x30
start:
    ; Set PD5 (OC0B) as output
    sbi DDRD, PD5  

    ; Configure Timer0
    ldi R16, (1 << WGM01) | (1 << WGM00) | (1 << COM0A1)  ; Set Fast PWM and non-inverting mode
    sts TCCR0A, R16

    ldi R16, (1 << WGM02) | (1 << CS00)  ; Prescaler = 1
    sts TCCR0B, R16

    ldi R17, 100
    out OC0RA, R17

    ldi R17, 50
    out OC0RB, R17

loop:
    rjmp loop  ; Infinite loop