;
; LAB3.asm
;
; Created: 3/2/2025 3:01:26 PM
; Author : moigomez
;

.include "m328Pdef.inc"
.cseg
.org 0

/*----------------------------------------
R16 - Index; Contains the index of the lookup table for the chosen digit
R17 - Shift Counter; 8-bit counter that shifts out data from register
R18 - Digit Value; Contains the data representing the digit on the 7-segment display
R19 - Digit Counter; Contains the incremented/decremented value for 7-segment display
R23 - Mode; Not implemented

R20 - RPG Direction; Stores the direction of RPG encoder
R21 - RPG Temporary Register; Stores temporary values that aid in decoding RPG

----------------------------------------*/


;----------------------------------------
; 7-Segment Display Patterns
;----------------------------------------
digit_patterns:    ; Bit Representation:    0bGEFPDCBA
	.dw 0b01101111, 0b00000110, 0b11001011, 0b10001111, 0b10100110, 0b10101101, 0b11101101, 0b00000111  ; 0-7
	.dw 0b11101111, 0b10101111, 0b11100111, 0b11101100, 0b01101001, 0b11001110, 0b11101001, 0b11100001  ; 8-F


;----------------------------------------
; Configure I/O Lines
;----------------------------------------
sbi DDRB, 0       ; PB0 (SER) - Output
sbi DDRB, 1       ; PB1 (RCLK) - Output
sbi DDRB, 2       ; PB2 (SRCLK) - Output
cbi DDRB, 3       ; PB3 (BUTTON) - Input
cbi DDRB, 4       ; PB4 (RPG A) - Input
cbi DDRB, 5       ; PB5 (RPG B) - Input

sbi PORTB, 3      ; Enable pull-up resistor on PB3
sbi PORTB, 4      ; Enable pull-up on PB4
sbi PORTB, 5      ; Enable pull-up on PB5

clr R19
clr R18           ; Initialize counter to 0
clr R23           ; Initialize mode: 0 = increment, 1 = decrement

;----------------------------------------
; Main Program Loop
;----------------------------------------
main_loop:
    
	;sbic PINB, 3
	;rjmp main_loop
    ;rcall wait_for_button_release

/*
    ; Make sure counter has not reached max value
	cpi R19, 15     ; if (R19 >= 15) {
	brge main_loop	; } else {
	inc R19         ;   R19++
                    ; }
*/

    rcall read_RPG_direction 

	; Load patterns into FLASH
    ldi ZL, low(digit_patterns)
    ldi ZH, high(digit_patterns)
    
	; Set index of pattern to retrieve
    ;--ldi R16, 1
	mov R16, R19
    lsl R16
    
	; Add offset to Z register
    add ZL, R16
    adc ZH, R1

	; Load chosen pattern to R18
	lpm R18, Z


	; Push pattern to display
    rcall display_pattern

	; rcall read_RPG_direction

	rcall wait_for_button_press

	;wait:
	;sbis PINB, 3
	;rjmp wait

	;rcall read_RPG_direction
	
	rjmp main_loop



wait_for_button_press:
    sbic PINB, 3
	rjmp wait_for_button_press
	ret

wait_for_button_release:
    sbis PINB, 3
	rjmp wait_for_button_release
	ret


read_RPG_direction:
    ; Read A and B, shifting A into bit 1, B into bit 0 -> Current state
    clr R21              ; Clear temporary register
    sbic PINB, 4         ; If A is high, set bit 1
    ori R21, (1 << 1)
    sbic PINB, 5         ; If B is high, set bit 0
    ori R21, (1 << 0)

    ; Compare current state (R21) with previous state (R20)
    cp R21, R20
    breq read_complete        ; If no change, exit

    ; Encode previous + current state into 4 bits (2-bit shift)
    lsl R20               ; Shift previous state left by 2
    lsl R20
    or R20, R21           ; Combine with new state

    ; Check transition pattern using known quadrature sequences
    cpi R20, 0b0001       ; 00 -> 01 (CW)
    breq RPG_rotated_CW
    cpi R20, 0b0111       ; 01 -> 11 (CW)
    breq RPG_rotated_CW
    cpi R20, 0b1110       ; 11 -> 10 (CW)
    breq RPG_rotated_CW
    cpi R20, 0b1000       ; 10 -> 00 (CW)
    breq RPG_rotated_CW

    cpi R20, 0b0010       ; 00 -> 10 (CCW)
    breq RPG_rotated_CCW
    cpi R20, 0b0100       ; 10 -> 11 (CCW)
    breq RPG_rotated_CCW
    cpi R20, 0b1101       ; 11 -> 01 (CCW)
    breq RPG_rotated_CCW
    cpi R20, 0b1011       ; 01 -> 00 (CCW)
    breq RPG_rotated_CCW

    rjmp updateState     ; No valid movement, update last state

RPG_rotated_CW:
    inc R19              ; Increment counter for CW rotation
    rjmp updateState

RPG_rotated_CCW:
    dec R19              ; Decrement counter for CCW rotation
    rjmp updateState

updateState:
    mov R20, R21          ; Save current state as previous
read_complete:
    ret                  ; Return to main loop



/*
read_RPG_direction:
	sbic PINB, 4
	rjmp read_complete

	sbic PINB, 5
	rjmp RPG_rotated_CW

	rjmp RPG_rotated_CCW

RPG_rotated_CW:
    ldi R20, 1
	;inc R19
	rjmp read_complete

RPG_rotated_CCW:
    ldi R20, 0
	;dec R19
	rjmp read_complete

read_complete:
	ret
*/







display_pattern:
    ldi R17, 8             ; Bit counter (8 bits)
    
shift_loop:
    rol R18                ; Rotate left, MSB goes into Carry
    brcc skip_set           ; If Carry = 0, skip setting SER
    sbi PORTB, 0           ; Set SER (PB0) if Carry = 1
    rjmp clk_pulse
skip_set:
    cbi PORTB, 0           ; Clear SER (PB0) if Carry = 0
    
clk_pulse:
    sbi PORTB, 2           ; Pulse SRCLK (PB2)
    cbi PORTB, 2
    
    dec R17                ; Decrement bit counter
    brne shift_loop         ; Repeat until all 8 bits are sent
    
    sbi PORTB, 1           ; Pulse RCLK (PB1) to latch output
    cbi PORTB, 1
    
    ret



	/*
lookup_table: .dw 0, 1, 4, 9, 16

    ldi ZL, low(lookup_table)
    ldi ZH, high(lookup_table)
    
    ldi r16, 3
    lsl r16
    
    add ZL, r16
    adc ZH, r1
    
    lpm r18, Z
	*/


/*
.include "m328Pdef.inc"

.cseg
.org 0x00

;----------------------------------------
; Configure I/O Lines
;----------------------------------------
sbi DDRB, 0       ; PB0 (SER) - Output
sbi DDRB, 1       ; PB1 (RCLK) - Output
sbi DDRB, 2       ; PB2 (SRCLK) - Output
cbi DDRB, 3       ; PB3 (BUTTON) - Input
sbi PORTB, 3      ; Enable pull-up resistor on PB3

clr R18           ; Initialize counter to 0
clr R23           ; Initialize mode: 0 = increment, 1 = decrement
;rcall update_display  ; Display '0' at startup

;----------------------------------------
; Main Program Loop
;----------------------------------------
mainLoop:
    sbic PINB, 3      ; Wait for button press (low)
    rjmp mainLoop

    ; Check hold time: >2s for toggle, ?4s for reset (no software debounce)
    ldi R20, 255      ; Outer loop base
    ldi R21, 255      ; Middle loop
    ldi R22, 20       ; Inner loop for ~4s total
    clr R24           ; Flag: 0 = short, 1 = toggle, 2 = reset

check_hold:
    sbic PINB, 3      ; If released (high), short press
    rjmp short_press
    dec R20
    brne check_hold
    dec R21
    brne check_hold
    cpi R22, 10       ; ~2s mark
    brne not_toggle_yet
    ldi R24, 1        ; Mark toggle threshold reached (>2s)
not_toggle_yet:
    dec R22
    brne check_hold
    ldi R24, 2        ; Mark reset threshold reached (?4s)

    ; Process hold result
    cpi R24, 2        ; Check if reset threshold met (?4s)
    breq toggle_and_reset
    cpi R24, 1        ; Check if toggle threshold met (>2s, <4s)
    breq toggle_mode_only
    rjmp short_press  ; Short press if released early

toggle_and_reset:
    clr R18           ; Reset counter to 0
    rjmp update_and_release

toggle_mode_only:
    rcall toggle_mode ; Toggle mode without reset
    rjmp update_and_release

toggle_mode:
    cpi R23, 0        ; Check current mode
    breq set_increment  ; If decrement (1), switch to increment (0)
    ldi R23, 1        ; If increment (0), switch to decrement (1)
    ret
set_increment:
    clr R23           ; Set to increment mode
    ret

update_and_release:
    rcall update_display  ; Show current value (after toggle or reset)

wait_release:
    sbis PINB, 3      ; Wait for release after long press
    rjmp wait_release
    rjmp mainLoop     ; No software debounce rely on hardware

short_press:
    cpi R23, 1        ; Check mode
    breq decrement    ; If 1, decrement
increment:
    inc R18           ; Incrsement counter
    cpi R18, 16       ; Check if reached 16
    brlo update_display
    clr R18
    rjmp update_display
decrement:
    dec R18           ; Decrement counter
    brpl update_display  ; If R18 >= 0, update
    ldi R18, 15       ; If R18 < 0, wrap to 15

update_display:
    ldi ZL, LOW(digit_patterns << 1)
    ldi ZH, HIGH(digit_patterns << 1)
    add ZL, R18
    brcc no_carry
    inc ZH
no_carry:
    lpm R16, Z        ; Load base pattern
    cpi R23, 1        ; Check mode
    brne skip_dp      ; If decrement, set DP
    ori R16, 0b00010000  ; Set bit 4 (P=DP) for decrement mode
skip_dp:
    rcall display

    ; Wait for release after short press
release_wait:
    sbis PINB, 3
    rjmp release_wait
    rjmp mainLoop     ; No software debounce rely on hardware

;----------------------------------------
; 7-Segment Display Patterns (Common Cathode, Hex 0-F)
;----------------------------------------
digit_patterns:    ;0bGEFPDCBA
    .db 0b01101111, 0b00000110, 0b11001011, 0b10001111, 0b10100110, 0b10101101, 0b11101101, 0b00000111  ; 0-7
    .db 0b11101111, 0b10101111, 0b11100111, 0b11101100, 0b01101001, 0b11001110, 0b11101001, 0b11100001  ; 8-F

;----------------------------------------
; Display Subroutine (Shift Register Logic)
;----------------------------------------
display:
    push R16
    push R17
    in R17, SREG
    push R17

    ldi R17, 8        ; Loop counter (8 bits to shift)
shift_loop:
    rol R16           ; Rotate left through Carry
    brcs set_ser_1    ; If Carry is set, set SER to 1
    cbi PORTB, 0      ; Otherwise, set SER (PB0) to 0
    rjmp clk_pulse
set_ser_1:
    sbi PORTB, 0      ; Set SER (PB0) to 1
clk_pulse:
    sbi PORTB, 2      ; Set SRCLK high
    nop               ; Small delay
    cbi PORTB, 2      ; Set SRCLK low
    dec R17
    brne shift_loop

    nop               ; Small delay before latch
    sbi PORTB, 1      ; Set RCLK high
    nop               ; Ensure latch holds
    cbi PORTB, 1      ; Set RCLK low

    pop R17
    out SREG, R17
    pop R17
    pop R16
    ret


*/