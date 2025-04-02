;
; RPG_Detection.asm
;
; Created: 4/1/2025
; Author : moigomez
;
; Collection of previous code that implemented RPG
; direction detection correctly
;

.include "m328Pdef.inc"

.cseg
.org 0x00

;----------------------------------------
; Configure I/O Lines
;----------------------------------------
cbi DDRB, 4       ; PB4 (RPG A) - Input
cbi DDRB, 5       ; PB5 (RPG B) - Input

sbi PORTB, 4      ; Enable pull-up on PB4
sbi PORTB, 5      ; Enable pull-up on PB5

sbi DDRD, 5       ; PD5 (Test) - Output

clr R19

;----------------------------------------
; Main Program Loop
;----------------------------------------
main_loop:
	ldi R22, 0
    rcall read_RPG_direction 

    ; Check if RPG is resting on detent and update count accordignly 
	cpi R20, 3        ; if (R20 != 3) {
	brne main_loop    ; }
	cpi R22, 1        ; else if (R22 == 1) {
	breq increment    ;     R19++
	cpi R22, 2        ; } else if (R22 == 2) }
	breq decrement    ;     R19--
                      ; }
rjmp main_loop  


;----------------------------------------
; Helper Subroutines
;----------------------------------------
increment:
    sbi PORTD, 5

    ; Make sure counter has not reached max value
	cpi R19, 15       ; if (R19 >= 15) {
	brge main_loop    ; } else {
	inc R19           ;     R19++
    rjmp main_loop    ; }
decrement:
    cbi PORTD, 5

    ; Make sure counter has not reached min value
	cpi R19, 0        ; if (R19 == 0) {
	breq main_loop    ; } else {
	dec R19           ;     R19--
	rjmp main_loop    ; }


;----------------------------------------
; RPG Subroutines
;----------------------------------------
read_RPG_direction:
    ; Read A and B, shifting A into bit 1, B into bit 0 -> Current state
    clr R21              ; Clear temporary register
    sbic PINB, 4         ; If A is high, set bit 1
    ori R21, (1 << 1)
    sbic PINB, 5         ; If B is high, set bit 0
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
	ldi R22, 1
    rjmp updateState
RPG_rotated_CCW:
	ldi R22, 2
    rjmp updateState
updateState:
    mov R20, R21         ; Save current state as previous
read_complete:
    ret
