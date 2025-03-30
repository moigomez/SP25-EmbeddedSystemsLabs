;
; Initialization.asm
;
; Created: 3/27/2025
; Author : Moises Gomez
;
; ASM file to initialize PWM functionality for fan
;

.include "m328pdef.inc"

; Define Constants
.equ F_CPU = 16000000    ; 16 MHz CPU clock
.equ PWM_FREQ = 80000    ; 80 kHz PWM frequency
.equ OCR0A_VAL = 199     ; OCR0A for 80kHz
.equ DUTY_CYCLE_MIN = 2  ; Minimum OCR0B value (~1%)
.equ DUTY_CYCLE_MAX = 199; Maximum OCR0B value (100%)

.cseg
.org 0x00
rjmp RESET           ; Reset vector

RESET:
    ; Set OC0B (PD5) as output
    sbi DDRD, PD5        ; Set PD5 (OC0B) as output

    ; Configure Timer0 for Fast PWM (Mode 7: WGM02:0 = 7)
    ldi R16, (1 << WGM02) ; Set WGM02 (select mode 7)
    sts TCCR0B, R16       ; Store to TCCR0B

    ldi R16, (1 << WGM00) | (1 << WGM01) | (1 << COM0B1)  ; Fast PWM, non-inverting
    sts TCCR0A, R16       ; Store to TCCR0A

    ; Set OCR0A for 80 kHz PWM frequency
    ldi R16, OCR0A_VAL
    sts OCR0A, R16

    ; Set initial duty cycle to 50% (OCR0B = OCR0A / 2)
    ldi R16, OCR0A_VAL / 2
    sts OCR0B, R16

    ; Start Timer0 with no prescaler (N = 1, CS02:0 = 001)
    ldi R16, (1 << CS00)
    sts TCCR0B, R16

MAIN_LOOP:
    rjmp MAIN_LOOP        ; Infinite loop (PWM runs automatically)