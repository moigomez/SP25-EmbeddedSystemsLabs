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

.equ RPG_A = 0        ; PB0 - PCINT0
.equ RPG_B = 1        ; PB1 - PCINT1
.equ TEST_PIN = 5        ; PD5 - for direction test output (e.g. LED)

.def rCurr = R16      ; Current state of A/B
.def rTemp = R17      ; Temp for shift/merge
.def rTrans = R18      ; 4-bit transition value
.def rPrev = R19      ; Previous state of A/B
.def rDir = R20      ; Direction/position counter

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
    sts PCMSK0, R16       ; Enable PCINT0 & PCINT1
    ldi R16, (1 << PCIE0)
    sts PCICR, R16        ; Enable Pin Change Interrupt group 0

    clr rCurr
    clr rPrev
    clr rDir

    sbic PINB, RPG_A
    ori rPrev, (1 << 1)   ; Save A to bit 1
    sbic PINB, RPG_B
    ori rPrev, (1 << 0)   ; Save B to bit 0

    sei   ; Enable global interrupts

main_loop:
    rjmp main_loop        ; Infinite loop

RPG_ISR:
    ; Get current state
    in  rCurr, PINB
    andi rCurr, 0x03          ; Mask for PB0 and PB1 (bits 1:0)

    ; Build transition: (rPrev << 2) | rCurr
    mov rTemp, rPrev
    lsl rTemp                 ; Shift left twice
    lsl rTemp
    or  rTemp, rCurr          ; Combine into transition
    mov rTrans, rTemp

    cpi rTrans, 0b0001
    breq clockwise
    cpi rTrans, 0b0111
    breq clockwise
    cpi rTrans, 0b1110
    breq clockwise
    cpi rTrans, 0b1000
    breq clockwise

    cpi rTrans, 0b0010
    breq counterclockwise
    cpi rTrans, 0b1011
    breq counterclockwise
    cpi rTrans, 0b1101
    breq counterclockwise
    cpi rTrans, 0b0100
    breq counterclockwise

    rjmp save_state          ; If no match, just save and exit
clockwise:
    inc rDir
    sbi PORTD, TEST_PIN      ; Turn on test pin
    rjmp save_state

counterclockwise:
    dec rDir
    cbi PORTD, TEST_PIN      ; Turn off test pin

save_state:
    mov rPrev, rCurr
    reti