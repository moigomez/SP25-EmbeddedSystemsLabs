;----------------------------------------
; TIMING CODE
;----------------------------------------
.include "m328Pdef.inc"

.org 0x0000
rjmp reset         ; Reset vector

.org 0x0050        ; Main program starts here


;----------------------------------------
; 7-Segment Display Patterns
;----------------------------------------
digit_patterns:    ; Bit Representation:    0bGEFPDCBA
	.dw 0b01101111, 0b00000110, 0b11001011, 0b10001111, 0b10100110, 0b10101101, 0b11101101, 0b00000111  ; 0-7
	.dw 0b11101111, 0b10101111, 0b11100111, 0b11101100, 0b01101001, 0b11001110, 0b11101001, 0b11100001  ; 8-F

reset:
    ldi R19, 0x00      ; Clear R19 initially
    ldi R20, 0x00      ; Counter storage
    ldi R21, 0x00      ; Temporary register

    ; Configure I/O
    sbi DDRB, 0       ; PB0 (SER) - Output
    sbi DDRB, 1       ; PB1 (RCLK) - Output
    sbi DDRB, 2       ; PB2 (SRCLK) - Output

    ;sbi DDRB, 5        ; Set PB5 as output (for debugging)
    cbi DDRD, 3        ; Set PD2 (Button Input) as input
    sbi PORTB, 3      ; Enable pull-up on PD2

    ; Configure Timer0
    ldi R19, (1 << CS01) | (1 << CS00)  ; Prescaler 64 (~1ms per tick at 16MHz)
    out TCCR0B, R19
    clr R19
    out TCNT0, R19      ; Clear Timer0

main_loop:
    sbic PORTB, 3        ; Skip next if button is not pressed
    rjmp main_loop      ; Loop until button is pressed

    ; Button Press Detected
    clr R20             ; Reset counter
    out TCNT0, R20      ; Reset timer

wait_release:
    sbis PORTB, 3        ; Skip next if still pressed
    rjmp end_press      ; If released, proceed

    in R21, TCNT0       ; Read Timer0 value
    mov R20, R21        ; Store the count
    rjmp wait_release   ; Keep waiting

end_press:
    ; Decide based on press time
    cpi R20, 50         ; If count < 50 (short press)
    brlo short_press

    cpi R20, 150        ; If count < 150 (medium press)
    brlo medium_press

    rjmp long_press     ; Otherwise, long press

short_press:
    ldi R19, 3       ; Load a value for short press
    rcall display_update
    rjmp update_output

medium_press:
    ldi R19, 4       ; Load a value for medium press
    rcall display_update
    rjmp update_output

long_press:
    ldi R19, 5       ; Load a value for long press
    rcall display_update

update_output:
    ;out PORTB, R19      ; Output result (debugging)
    rjmp main_loop      ; Restart process


;----------------------------------------
; 7-SEGMENT DISPLAY CODE
;----------------------------------------

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