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

R20 - RPG State; Stores the value of the quadrature sequences
R21 - RPG Temporary Register; Stores temporary values that aid in decoding RPG
R22 - RPG Direction; Stores the direction of RPG encoder




;R28
;R27
;R28
;R29

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

; sbi DDRB, 5       ; Turn Off LED (L)
; cbi PORTB, 5

clr R19
clr R18           ; Initialize counter to 0

; ; === Setup Timer0 (Normal Mode) ===
; ldi R25, (1 << CS02) | (1 << CS00)  ; Prescaler 1024
; out TCCR0B, R25                     ; Start Timer0
; clr R23                              

.def tmp1 = R23        ; Temporary register
.def tmp2 = R24        ; Temporary register


;----------------------------------------
; Main Program Loop
;----------------------------------------
main_loop:

	ldi R22, 0
    rcall read_RPG_direction 
    ;rcall read_button_state

    ; Check if RPG is resting on detent and update count accordignly 
	cpi R20, 3        ; if (R20 != 3) {
	brne main_loop    ; }
	cpi R22, 1        ; else if (R22 == 1) {
	breq increment    ;     R19++
	cpi R22, 2        ; } else if (R22 == 2) }
	breq decrement    ;     R19--
                      ; }

	button_check:
        sbic PINB, 3
        rjmp display_update

        rcall start_timer 

    ; Wait for button release or timeout
    wait_release:
        sbis PINB, 3
        rjmp check_time   

        rjmp wait_release ; Keep waiting if still pressed

    check_time:
        rcall check_timer ; Call function to check if time < 1 sec

        cpi R25, 1        ; Check if timer expired
        breq timeout      ; If R25 == 1, time exceeded, don't increment

        rcall increment   ; Otherwise, increment R19

    timeout:
        rjmp display_update ; Always update display after any input

    display_update:
        ; Load patterns into FLASH
        ldi ZL, low(digit_patterns)
        ldi ZH, high(digit_patterns)

        ; Set index of pattern to retrieve
        mov R16, R19
        lsl R16
        
        ; Add offset to Z register
        add ZL, R16
        adc ZH, R1

        ; Load chosen pattern to R18
        lpm R18, Z

        ; Push pattern to display
        rcall display_pattern

rjmp main_loop  


;----------------------------------------
; Helper Subroutines
;----------------------------------------
increment:
    ; Make sure counter has not reached max value
	cpi R19, 15       ; if (R19 >= 15) {
	brge main_loop    ; } else {
	inc R19           ;     R19++
    rjmp main_loop    ; }
decrement:
    ; Make sure counter has not reached min value
	cpi R19, 0        ; if (R19 == 0) {
	breq main_loop    ; } else {
	dec R19           ;     R19--
	rjmp main_loop    ; }

/*
wait_for_button_press:
    sbic PINB, 3
	rjmp wait_for_button_press
	ret

wait_for_button_release:
    sbis PINB, 3
	rjmp wait_for_button_release
	ret
*/

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

/*
read_button_state:
    sbic PINB, 3       
    rjmp button_released

button_pressed:
    ldi R26, 10
	ldi R19, 4	 
    ret               

button_released:
    ldi R26, 20      	 
    ret    
*/


; --------------------------
; Subroutine: start_timer
; --------------------------
start_timer:
    ldi R30, 0x02          ; Set prescaler
    out 0x33, R30          ; Configure Timer0
    rcall delay            ; Call provided delay function
    ret

; --------------------------
; Subroutine: check_timer
; Returns 1 in R25 if timeout exceeded, 0 otherwise
; --------------------------
check_timer:
    in tmp2, TIFR0         ; Read timer flags
    sbrs tmp2, TOV0        ; Check if timer overflow occurred
    rjmp not_expired       ; If no overflow, jump

    ldi R25, 1             ; Timer expired (set flag in R25)
    ret

not_expired:
    ldi R25, 0             ; Timer not expired
    ret

; --------------------------
; Subroutine: Delay using Timer0 Overflow
; --------------------------
delay:
    ; Stop Timer0
    in tmp1, TCCR0B        ; Save configuration
    ldi tmp2, 0x00         ; Stop Timer0
    out TCCR0B, tmp2

    ; Clear overflow flag
    in tmp2, TIFR0         ; Read TIFR0
    sbr tmp2, 1 << TOV0    ; Clear TOV0 by writing logic 1
    out TIFR0, tmp2

    ; Start Timer0 with new count
    out TCNT0, tmp1        ; Load counter
    out TCCR0B, tmp1       ; Restart Timer0

wait:
    in tmp2, TIFR0         ; Read TIFR0
    sbrs tmp2, TOV0        ; Check overflow flag
    rjmp wait              ; Wait until overflow occurs
    ret





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
