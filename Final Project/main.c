/*
 *
 * main.c
 *
 * Created: 4/26/2025 2:41:58 PM
 * Authors: Emre Akgül, Moises Gomez
 *
 * C-program for our Final Project that implements
 * a 4x4 Tic-Tac-Toe game on an LCD with joysticks
 * and an accelerometer/gyroscope sensor.
 *
 */

#define F_CPU 16000000UL

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "i2c_master.c"
#include "i2c_master.h"
#include "liquid_crystal_i2c.c"
#include "liquid_crystal_i2c.h"

#define JOY_X1_CHANNEL 0
#define JOY_Y1_CHANNEL 1
#define JOY_BUTTON1_PIN PD2
#define JOY_X2_CHANNEL 2
#define JOY_Y2_CHANNEL 3
#define JOY_BUTTON2_PIN PD3
#define MENU_BUTTON_PIN PB3
#define MPU6050_ADDR 0x68

char board[4][4];
uint8_t cursorXRow = 0, cursorXCol = 0;
uint8_t cursorORow = 0, cursorOCol = 0;
char currentPlayer = 'X';
uint8_t xScore = 0;
uint8_t oScore = 0;
uint8_t scoresUpdated = 1;
uint8_t isNewGame = 1;

#define X_SCORE_ADDR 0
#define O_SCORE_ADDR 1

uint8_t lcd_rows[4] = {0, 1, 2, 3};
uint8_t lcd_cols[4] = {2, 4, 6, 8};

typedef struct {
    uint8_t row;
    uint8_t col;
} Placement;

Placement xPlacements[16];
uint8_t xPlacementCount = 0;
Placement oPlacements[16];
uint8_t oPlacementCount = 0;

Placement winningCells[4];
uint8_t winningCellCount = 0;

enum GameState { MENU, PLAYING, GYROSCOPE };
enum GameState gameState = MENU;
uint8_t menuSelection = 0;

uint8_t cursorBlinkState = 1;
uint16_t loopCount = 0;

volatile uint32_t millis_counter = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t prev_az = 0;
uint32_t prev_time = 0;
uint32_t last_reset_time = 0;
int count = 0;
bool isShaking = false;
#define MA_WINDOW_SIZE 20
float mag_buffer[MA_WINDOW_SIZE] = {0};
uint8_t mag_index = 0;
float mag_sum = 0;

uint8_t move_count = 0;
const uint8_t MOVE_THRESHOLD = 5;

uint8_t eeprom_read_byte_safe(uint16_t addr) {
    return eeprom_read_byte((const uint8_t*)addr);
}

void eeprom_write_byte_safe(uint16_t addr, uint8_t value) {
    eeprom_write_byte((uint8_t*)addr, value);
}

void init_scores(void) {
    xScore = eeprom_read_byte_safe(X_SCORE_ADDR);
    oScore = eeprom_read_byte_safe(O_SCORE_ADDR);
    if (xScore == 0xFF) xScore = 0;
    if (oScore == 0xFF) oScore = 0;
    scoresUpdated = 1;
}

void save_scores(void) {
    eeprom_write_byte_safe(X_SCORE_ADDR, xScore);
    eeprom_write_byte_safe(O_SCORE_ADDR, oScore);
    scoresUpdated = 1;
}

void display_scores(LiquidCrystalDevice_t* dev) {
    char scoreStr[12];
    lq_setCursor(dev, 2, 11);
    lq_print(dev, "         ");
    lq_setCursor(dev, 3, 11);
    lq_print(dev, "         ");
    lq_setCursor(dev, 2, 11);
    snprintf(scoreStr, 12, "X = %d pts", xScore);
    lq_print(dev, scoreStr);
    lq_setCursor(dev, 3, 11);
    snprintf(scoreStr, 12, "O = %d pts", oScore);
    lq_print(dev, scoreStr);
    scoresUpdated = 0;
}

void draw_static_board(LiquidCrystalDevice_t* dev) {
    lq_clear(dev);
    lq_setCursor(dev, 0, 2);
    lq_print(dev, "_|_|_|_");
    lq_setCursor(dev, 1, 2);
    lq_print(dev, "_|_|_|_");
    lq_setCursor(dev, 2, 2);
    lq_print(dev, "_|_|_|_");
    lq_setCursor(dev, 3, 2);
    lq_print(dev, " | | | ");
    display_scores(dev);
}

void draw_cell(LiquidCrystalDevice_t* dev, uint8_t r, uint8_t c, char val) {
    lq_setCursor(dev, lcd_rows[r], lcd_cols[c]);
    char temp[2] = {val, '\0'};
    lq_print(dev, temp);
}

void redraw_static_element(LiquidCrystalDevice_t* dev, uint8_t r, uint8_t c) {
    lq_setCursor(dev, lcd_rows[r], lcd_cols[c]);
    if (r < 3)
        lq_print(dev, "_");
    else
        lq_print(dev, " ");
}

void display_current_player(LiquidCrystalDevice_t* dev) {
    lq_setCursor(dev, 0, 11);
    lq_print(dev, "       ");
    lq_setCursor(dev, 0, 11);
    lq_print(dev, "Turn: ");
    char temp[2] = {currentPlayer, '\0'};
    lq_print(dev, temp);
}

void display_menu(LiquidCrystalDevice_t* dev) {
    lq_clear(dev);
    lq_setCursor(dev, 0, 0);
    lq_print(dev, "4x4 Tic-Tac-Toe");
    lq_setCursor(dev, 1, 0);
    lq_print(dev, "Please Select Mode");
    lq_setCursor(dev, 2, 0);
    lq_print(dev, menuSelection == 0 ? ">Joystick" : " Joystick");
    lq_setCursor(dev, 3, 0);
    lq_print(dev, menuSelection == 1 ? ">Gyroscope" : " Gyroscope");
}

void reset_game(LiquidCrystalDevice_t* dev) {
    for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 4; j++) board[i][j] = ' ';
    xPlacementCount = 0;
    oPlacementCount = 0;
    for (uint8_t i = 0; i < 16; i++) {
        xPlacements[i].row = 0;
        xPlacements[i].col = 0;
        oPlacements[i].row = 0;
        oPlacements[i].col = 0;
    }
    cursorXRow = 0;
    cursorXCol = 0;
    cursorORow = 0;
    cursorOCol = 0;
    winningCellCount = 0;
    for (uint8_t i = 0; i < 4; i++) {
        winningCells[i].row = 0;
        winningCells[i].col = 0;
    }
    currentPlayer = 'X';
    cursorBlinkState = 1;
    draw_static_board(dev);
    display_current_player(dev);
}

void redraw_board(LiquidCrystalDevice_t* dev) {
    draw_static_board(dev);
    for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 4; j++)
            if (board[i][j] != ' ') draw_cell(dev, i, j, board[i][j]);
    display_current_player(dev);
}

void blink_winning_cells(LiquidCrystalDevice_t* dev, char winner) {
    for (uint8_t blink = 0; blink < 3; blink++) {
        for (uint8_t i = 0; i < winningCellCount; i++)
            redraw_static_element(dev, winningCells[i].row,
                                  winningCells[i].col);
        _delay_ms(400);
        for (uint8_t i = 0; i < winningCellCount; i++)
            draw_cell(dev, winningCells[i].row, winningCells[i].col, winner);
        _delay_ms(400);
    }
}

char check_win(void) {
    winningCellCount = 0;
    for (uint8_t i = 0; i < 4; i++)
        if (board[i][0] != ' ' && board[i][0] == board[i][1] &&
            board[i][1] == board[i][2] && board[i][2] == board[i][3]) {
            winningCellCount = 4;
            for (uint8_t j = 0; j < 4; j++) {
                winningCells[j].row = i;
                winningCells[j].col = j;
            }
            return board[i][0];
        }
    for (uint8_t j = 0; j < 4; j++)
        if (board[0][j] != ' ' && board[0][j] == board[1][j] &&
            board[1][j] == board[2][j] && board[2][j] == board[3][j]) {
            winningCellCount = 4;
            for (uint8_t i = 0; i < 4; i++) {
                winningCells[i].row = i;
                winningCells[i].col = j;
            }
            return board[0][j];
        }
    if (board[0][0] != ' ' && board[0][0] == board[1][1] &&
        board[1][1] == board[2][2] && board[2][2] == board[3][3]) {
        winningCellCount = 4;
        for (uint8_t i = 0; i < 4; i++) {
            winningCells[i].row = i;
            winningCells[i].col = i;
        }
        return board[0][0];
    }
    if (board[0][3] != ' ' && board[0][3] == board[1][2] &&
        board[1][2] == board[2][1] && board[2][1] == board[3][0]) {
        winningCellCount = 4;
        for (uint8_t i = 0; i < 4; i++) {
            winningCells[i].row = i;
            winningCells[i].col = 3 - i;
        }
        return board[0][3];
    }
    for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 3; j++)
            if (board[i][j] != ' ' && board[i][j] == board[i][j + 1] &&
                board[i][j] == board[i + 1][j] &&
                board[i][j] == board[i + 1][j + 1]) {
                winningCellCount = 4;
                winningCells[0].row = i;
                winningCells[0].col = j;
                winningCells[1].row = i;
                winningCells[1].col = j + 1;
                winningCells[2].row = i + 1;
                winningCells[2].col = j;
                winningCells[3].row = i + 1;
                winningCells[3].col = j + 1;
                return board[i][j];
            }
    if (board[0][0] != ' ' && board[0][0] == board[0][3] &&
        board[0][0] == board[3][0] && board[0][0] == board[3][3]) {
        winningCellCount = 4;
        winningCells[0].row = 0;
        winningCells[0].col = 0;
        winningCells[1].row = 0;
        winningCells[1].col = 3;
        winningCells[2].row = 3;
        winningCells[2].col = 0;
        winningCells[3].row = 3;
        winningCells[3].col = 3;
        return board[0][0];
    }
    return ' ';
}

void display_win_message(LiquidCrystalDevice_t* dev, char winner) {
    blink_winning_cells(dev, winner);
    lq_clear(dev);
    if (winner == 'X') {
        lq_setCursor(dev, 1, 3);
        lq_print(dev, "Player X Wins!");
        xScore++;
    } else if (winner == 'O') {
        lq_setCursor(dev, 1, 3);
        lq_print(dev, "Player O Wins!");
        oScore++;
    }
    save_scores();
    _delay_ms(3000);
    reset_game(dev);
    isNewGame = 1;
}

void add_x_placement(LiquidCrystalDevice_t* dev, uint8_t row, uint8_t col) {
    if (xPlacementCount >= 4) {
        uint8_t oldestRow = xPlacements[0].row;
        uint8_t oldestCol = xPlacements[0].col;
        board[oldestRow][oldestCol] = ' ';
        redraw_static_element(dev, oldestRow, oldestCol);
        for (uint8_t i = 0; i < xPlacementCount - 1; i++) {
            xPlacements[i] = xPlacements[i + 1];
        }
        xPlacementCount--;
    }
    xPlacements[xPlacementCount].row = row;
    xPlacements[xPlacementCount].col = col;
    xPlacementCount++;
    board[row][col] = 'X';
    draw_cell(dev, row, col, 'X');
    char winner = check_win();
    if (winner != ' ') {
        display_win_message(dev, winner);
    } else {
        currentPlayer = 'O';
        display_current_player(dev);
    }
}

void add_o_placement(LiquidCrystalDevice_t* dev, uint8_t row, uint8_t col) {
    if (oPlacementCount >= 4) {
        uint8_t oldestRow = oPlacements[0].row;
        uint8_t oldestCol = oPlacements[0].col;
        board[oldestRow][oldestCol] = ' ';
        redraw_static_element(dev, oldestRow, oldestCol);
        for (uint8_t i = 0; i < oPlacementCount - 1; i++) {
            oPlacements[i] = oPlacements[i + 1];
        }
        oPlacementCount--;
    }
    oPlacements[oPlacementCount].row = row;
    oPlacements[oPlacementCount].col = col;
    oPlacementCount++;
    board[row][col] = 'O';
    draw_cell(dev, row, col, 'O');
    char winner = check_win();
    if (winner != ' ') {
        display_win_message(dev, winner);
    } else {
        currentPlayer = 'X';
        display_current_player(dev);
    }
}

uint16_t read_adc(uint8_t channel) {
    if (channel > 7) return 0;
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void find_nearest_empty(uint8_t currRow, uint8_t currCol, int8_t rowStep,
                        int8_t colStep, uint8_t* newRow, uint8_t* newCol) {
    *newRow = currRow;
    *newCol = currCol;
    uint8_t row = currRow;
    uint8_t col = currCol;
    uint8_t steps = 0;
    while (steps < 2 && row < 4 && row >= 0 && col < 4 && col >= 0) {
        row = currRow + (steps + 1) * rowStep;
        col = currCol + (steps + 1) * colStep;
        if (row < 4 && row >= 0 && col < 4 && col >= 0 &&
            board[row][col] == ' ') {
            *newRow = row;
            *newCol = col;
            return;
        }
        steps++;
    }
    if (*newRow == currRow && *newCol == currCol) {
        int8_t adjRowSteps[] = {-1, 1, 0, 0};
        int8_t adjColSteps[] = {0, 0, -1, 1};
        for (uint8_t i = 0; i < 4; i++) {
            row = currRow;
            col = currCol;
            steps = 0;
            while (steps < 2 && row < 4 && row >= 0 && col < 4 && col >= 0) {
                row = currRow + (steps + 1) * adjRowSteps[i];
                col = currCol + (steps + 1) * adjColSteps[i];
                if (row < 4 && row >= 0 && col < 4 && col >= 0 &&
                    board[row][col] == ' ') {
                    *newRow = row;
                    *newCol = col;
                    return;
                }
                steps++;
            }
        }
    }
}

void TWI_Init(void) {
    TWSR = 0x00;
    TWBR = 0x48;  // ~100kHz @ 16MHz
    TWCR = (1 << TWEN);
}

void TWI_Start(void) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop(void) { TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); }

void TWI_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t TWI_ReadACK(void) {
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t TWI_ReadNACK(void) {
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

bool MPU6050_Init(void) {
    for (int i = 0; i < 5; i++) {  // Retry up to 5 times
        TWI_Start();
        TWI_Write(MPU6050_ADDR << 1);  // Write mode
        TWI_Write(0x6B);               // PWR_MGMT_1 register
        TWI_Write(0x00);               // Set to zero (wakes up the MPU-6050)
        TWI_Stop();
        _delay_ms(100);  // Wait for stabilization

        // Verify initialization by reading WHO_AM_I (0x75) register
        TWI_Start();
        TWI_Write(MPU6050_ADDR << 1);
        TWI_Write(0x75);
        TWI_Start();
        TWI_Write((MPU6050_ADDR << 1) | 1);  // Read mode
        uint8_t who_am_i = TWI_ReadNACK();
        TWI_Stop();
        if (who_am_i == 0x68) {  // Expected value for MPU6050
            return true;         // Initialization successful
        }
        _delay_ms(100);  // Wait before retry
    }
    return false;  // Initialization failed after retries
}

void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);
    TWI_Write(0x3B);  // Starting register for accelerometer
    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1) | 1);  // Read mode
    *ax = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *ay = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *az = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadNACK();
    TWI_Stop();
}

void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    TWI_Start();
    TWI_Write(MPU6050_ADDR << 1);
    TWI_Write(0x43);  // Starting register for gyroscope
    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1) | 1);  // Read mode
    *gx = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *gy = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadACK();
    *gz = ((int16_t)TWI_ReadACK() << 8) | TWI_ReadNACK();
    TWI_Stop();
}

ISR(TIMER1_COMPA_vect) { millis_counter++; }
void Timer1_Init(void) {
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10);
    OCR1A = 249;
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

uint32_t millis(void) {
    uint32_t ms;
    cli();
    ms = millis_counter;
    sei();
    return ms;
}

float compute_derivative(int16_t current, int16_t previous, float dt_ms) {
    return (float)(current - previous) / dt_ms;
}

int main(void) {
    TWI_Init();
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
    DDRD &= ~(1 << JOY_BUTTON1_PIN);
    PORTD |= (1 << JOY_BUTTON1_PIN);
    DDRD &= ~(1 << JOY_BUTTON2_PIN);
    PORTD |= (1 << JOY_BUTTON2_PIN);
    DDRB &= ~(1 << MENU_BUTTON_PIN);
    PORTB |= (1 << MENU_BUTTON_PIN);
    LiquidCrystalDevice_t lcd = lq_init(0x27, 20, 4, LCD_5x8DOTS);
    lq_turnOnBacklight(&lcd);
    init_scores();
    Timer1_Init();
    DDRB |= (1 << PB5);

    // Perform initial MPU6050 reads to wake the sensor and clear the I2C bus
    for (int i = 0; i < 20; i++) {
        MPU6050_ReadAccel(&ax, &ay, &az);
        MPU6050_ReadGyro(&gx, &gy, &gz);
        _delay_ms(10);
    }

    // Initialize the MPU6050
    if (!MPU6050_Init()) {
        while (1) {
            PORTB |= (1 << PB5);  // Turn on LED
            _delay_ms(500);
            PORTB &= ~(1 << PB5);  // Turn off LED
            _delay_ms(500);
        }
    }

    uint8_t prevXRow = cursorXRow, prevXCol = cursorXCol;
    uint8_t prevORow = cursorORow, prevOCol = cursorOCol;
    uint8_t buttonXPressed = 0;
    uint8_t buttonOPressed = 0;

    display_menu(&lcd);

    while (1) {
        uint32_t now = millis();
        float dt = (now - prev_time) / 1000.0;
        if ((now - last_reset_time) >= 300) {
            count = 0;
            isShaking = false;
            last_reset_time = now;
        }

        MPU6050_ReadAccel(&ax, &ay, &az);
        MPU6050_ReadGyro(&gx, &gy, &gz);

        float daz_dt = 0.0;
        if (dt > 0) daz_dt = compute_derivative(az, prev_az, dt);
        prev_az = az;
        prev_time = now;

        float mag = sqrt((double)ax * ax + (double)ay * ay + (double)az * az);
        mag_sum -= mag_buffer[mag_index];
        mag_buffer[mag_index] = mag;
        mag_sum += mag;
        mag_index = (mag_index + 1) % MA_WINDOW_SIZE;
        float mag_avg = mag_sum / MA_WINDOW_SIZE;

        if (((mag >= 30000.0 && mag < 40000.0 && mag_avg > 18000.0) ||
             (daz_dt >= 200.0 && daz_dt < 400.0))) {
            count++;
        }

        if (count >= 4) {
            isShaking = true;
            PORTB |= (1 << PB5);
            xScore = 0;
            oScore = 0;
            save_scores();
            if (gameState == PLAYING || gameState == GYROSCOPE) {
                reset_game(&lcd);
            }
            count = 0;
            last_reset_time = now;
        } else {
            PORTB &= ~(1 << PB5);
        }

        if (gameState == MENU) {
            uint16_t y1 = read_adc(JOY_Y1_CHANNEL);
            // Adjusted for Player X joystick in MENU: +Y -> -Y
            if (y1 < 300 &&
                menuSelection <
                    1) {  // Up (+Y) -> Down (-Y): Move selection down
                menuSelection = 1;
                display_menu(&lcd);
                _delay_ms(100);
            } else if (y1 > 700 &&
                       menuSelection >
                           0) {  // Down (-Y) -> Up (+Y): Move selection up
                menuSelection = 0;
                display_menu(&lcd);
                _delay_ms(100);
            }
            if (!(PIND & (1 << JOY_BUTTON1_PIN))) {
                _delay_ms(50);
                if (!(PIND & (1 << JOY_BUTTON1_PIN))) {
                    while (!(PIND & (1 << JOY_BUTTON1_PIN))) {
                    }
                    if (menuSelection == 0) {
                        gameState = PLAYING;
                        if (isNewGame)
                            reset_game(&lcd);
                        else
                            redraw_board(&lcd);
                    } else {
                        gameState = GYROSCOPE;
                        if (isNewGame)
                            reset_game(&lcd);
                        else
                            redraw_board(&lcd);
                    }
                    _delay_ms(200);
                }
            }
        } else if (gameState == PLAYING) {
            if (!(PINB & (1 << MENU_BUTTON_PIN))) {
                gameState = MENU;
                menuSelection = 0;
                isNewGame = 0;
                display_menu(&lcd);
                while (!(PINB & (1 << MENU_BUTTON_PIN))) {
                }
                _delay_ms(50);
            }

            if (currentPlayer == 'X') {
                uint16_t x1 = read_adc(JOY_X1_CHANNEL);
                uint16_t y1 = read_adc(JOY_Y1_CHANNEL);
                uint8_t newXRow = cursorXRow;
                uint8_t newXCol = cursorXCol;

                // Adjusted for Player X: +X -> -X, +Y -> -Y
                if (x1 < 300 && y1 < 300)
                    find_nearest_empty(cursorXRow, cursorXCol, 1, 1, &newXRow,
                                       &newXCol);  // Left (+X) & Up (-Y) ->
                                                   // Down (-Y) & Right (+X)
                else if (x1 > 700 && y1 < 300)
                    find_nearest_empty(cursorXRow, cursorXCol, 1, -1, &newXRow,
                                       &newXCol);  // Right (-X) & Up (-Y) ->
                                                   // Down (-Y) & Left (-X)
                else if (x1 < 300 && y1 > 700)
                    find_nearest_empty(cursorXRow, cursorXCol, -1, 1, &newXRow,
                                       &newXCol);  // Left (+X) & Down (+Y) ->
                                                   // Up (+Y) & Right (+X)
                else if (x1 > 700 && y1 > 700)
                    find_nearest_empty(cursorXRow, cursorXCol, -1, -1, &newXRow,
                                       &newXCol);  // Right (-X) & Down (+Y) ->
                                                   // Up (+Y) & Left (-X)
                else if (x1 < 300)
                    find_nearest_empty(cursorXRow, cursorXCol, 0, 1, &newXRow,
                                       &newXCol);  // Left (+X) -> Right (+X)
                else if (x1 > 700)
                    find_nearest_empty(cursorXRow, cursorXCol, 0, -1, &newXRow,
                                       &newXCol);  // Right (-X) -> Left (-X)
                else if (y1 < 300)
                    find_nearest_empty(cursorXRow, cursorXCol, 1, 0, &newXRow,
                                       &newXCol);  // Up (-Y) -> Down (-Y)
                else if (y1 > 700)
                    find_nearest_empty(cursorXRow, cursorXCol, -1, 0, &newXRow,
                                       &newXCol);  // Down (+Y) -> Up (+Y)

                if (newXRow >= 4) newXRow = 3;
                if (newXCol >= 4) newXCol = 3;
                if (newXRow < 0) newXRow = 0;
                if (newXCol < 0) newXCol = 0;

                if (cursorXRow != newXRow || cursorXCol != newXCol) {
                    if (board[prevXRow][prevXCol] == ' ')
                        redraw_static_element(&lcd, prevXRow, prevXCol);
                    cursorXRow = newXRow;
                    cursorXCol = newXCol;
                    prevXRow = cursorXRow;
                    prevXCol = cursorXCol;
                    _delay_ms(100);
                }
                if (!(PIND & (1 << JOY_BUTTON1_PIN)) && !buttonXPressed) {
                    buttonXPressed = 1;
                    if (board[cursorXRow][cursorXCol] == ' ') {
                        draw_cell(&lcd, cursorXRow, cursorXCol, 'X');
                        add_x_placement(&lcd, cursorXRow, cursorXCol);
                    }
                    _delay_ms(200);
                } else if ((PIND & (1 << JOY_BUTTON1_PIN)) && buttonXPressed) {
                    buttonXPressed = 0;
                    _delay_ms(50);
                }
            } else {
                uint16_t x2 = read_adc(JOY_X2_CHANNEL);
                uint16_t y2 = read_adc(JOY_Y2_CHANNEL);
                uint8_t newORow = cursorORow;
                uint8_t newOCol = cursorOCol;

                // Adjusted for Player O: +X -> +Y, -X -> -Y, +Y -> -X, -Y -> +X
                if (x2 < 300 && y2 < 300)
                    find_nearest_empty(cursorORow, cursorOCol, 1, -1, &newORow,
                                       &newOCol);  // Left (-X) & Up (+Y) ->
                                                   // Down (-Y) & Left (-X)
                else if (x2 > 700 && y2 < 300)
                    find_nearest_empty(cursorORow, cursorOCol, -1, -1, &newORow,
                                       &newOCol);  // Right (+X) & Up (+Y) -> Up
                                                   // (+Y) & Left (-X)
                else if (x2 < 300 && y2 > 700)
                    find_nearest_empty(cursorORow, cursorOCol, 1, 1, &newORow,
                                       &newOCol);  // Left (-X) & Down (-Y) ->
                                                   // Down (-Y) & Right (+X)
                else if (x2 > 700 && y2 > 700)
                    find_nearest_empty(cursorORow, cursorOCol, -1, 1, &newORow,
                                       &newOCol);  // Right (+X) & Down (-Y) ->
                                                   // Up (+Y) & Right (+X)
                else if (x2 < 300)
                    find_nearest_empty(cursorORow, cursorOCol, 1, 0, &newORow,
                                       &newOCol);  // Left (-X) -> Down (-Y)
                else if (x2 > 700)
                    find_nearest_empty(cursorORow, cursorOCol, -1, 0, &newORow,
                                       &newOCol);  // Right (+X) -> Up (+Y)
                else if (y2 < 300)
                    find_nearest_empty(cursorORow, cursorOCol, 0, -1, &newORow,
                                       &newOCol);  // Up (+Y) -> Left (-X)
                else if (y2 > 700)
                    find_nearest_empty(cursorORow, cursorOCol, 0, 1, &newORow,
                                       &newOCol);  // Down (-Y) -> Right (+X)

                if (newORow >= 4) newORow = 3;
                if (newOCol >= 4) newOCol = 3;
                if (newORow < 0) newORow = 0;
                if (newOCol < 0) newOCol = 0;

                if (cursorORow != newORow || cursorOCol != newOCol) {
                    if (board[prevORow][prevOCol] == ' ')
                        redraw_static_element(&lcd, prevORow, prevOCol);
                    cursorORow = newORow;
                    cursorOCol = newOCol;
                    prevORow = cursorORow;
                    prevOCol = cursorOCol;
                    _delay_ms(100);
                }
                if (!(PIND & (1 << JOY_BUTTON2_PIN)) && !buttonOPressed) {
                    buttonOPressed = 1;
                    if (board[cursorORow][cursorOCol] == ' ') {
                        draw_cell(&lcd, cursorORow, cursorOCol, 'O');
                        add_o_placement(&lcd, cursorORow, cursorOCol);
                    }
                    _delay_ms(200);
                } else if ((PIND & (1 << JOY_BUTTON2_PIN)) && buttonOPressed) {
                    buttonOPressed = 0;
                    _delay_ms(50);
                }
            }

            loopCount++;
            if (loopCount >= 10) {
                loopCount = 0;
                cursorBlinkState = !cursorBlinkState;
                if (currentPlayer == 'X' && board[cursorXRow][cursorXCol] == ' ') {
                    if (cursorBlinkState)
                        draw_cell(&lcd, cursorXRow, cursorXCol, 'X');
                    else
                        redraw_static_element(&lcd, cursorXRow, cursorXCol);
                } else if (currentPlayer == 'O' && board[cursorORow][cursorOCol] == ' ') {
                    if (cursorBlinkState) {
                        draw_cell(&lcd, cursorORow, cursorOCol, 'O');
                    }
                    else {
                        redraw_static_element(&lcd, cursorORow, cursorOCol);
                    }
                }
            }

            if (scoresUpdated) display_scores(&lcd);
        } else if (gameState == GYROSCOPE) {
            if (!(PINB & (1 << MENU_BUTTON_PIN))) {
                gameState = MENU;
                menuSelection = 0;
                isNewGame = 0;
                display_menu(&lcd);
                while (!(PINB & (1 << MENU_BUTTON_PIN))) {
                }
                _delay_ms(50);
            }

            if (!isShaking) {
                float ax_normalized = ax / 16384.0;
                float tilt_pitch = asin(ax_normalized) * 180.0 / M_PI;

                float ay_normalized = ay / 16384.0;
                float tilt_roll = asin(ay_normalized) * 180.0 / M_PI;

                move_count++;
                if (currentPlayer == 'X') {
                    uint8_t newXRow = cursorXRow;
                    uint8_t newXCol = cursorXCol;
                    if (tilt_pitch > 15.0 && fabs(tilt_pitch) > 5.0) {
                        newXRow = (cursorXRow < 3)
                                      ? cursorXRow + 1
                                      : 3;
                    } else if (tilt_pitch < -15.0 && fabs(tilt_pitch) > 5.0) {
                        newXRow =
                            (cursorXRow > 0)
                                ? cursorXRow - 1
                                : 0;
                    }
                    if (tilt_roll > 15.0 && fabs(tilt_roll) > 5.0) {
                        newXCol =
                            (cursorXCol < 3)
                                ? cursorXCol + 1
                                : 3;
                    } else if (tilt_roll < -15.0 && fabs(tilt_roll) > 5.0) {
                        newXCol =
                            (cursorXCol > 0)
                                ? cursorXCol - 1
                                : 0;
                    }

                    if (cursorXRow != newXRow || cursorXCol != newXCol) {
                        if (board[prevXRow][prevXCol] == ' ')
                            redraw_static_element(&lcd, prevXRow, prevXCol);
                        cursorXRow = newXRow;
                        cursorXCol = newXCol;
                        prevXRow = cursorXRow;
                        prevXCol = cursorXCol;
                        move_count = 0;
                    }
                } else {
                    uint8_t newORow = cursorORow;
                    uint8_t newOCol = cursorOCol;
                    if (tilt_pitch > 15.0 && fabs(tilt_pitch) > 5.0) {
                        newORow = (cursorORow < 3)
                                      ? cursorORow + 1
                                      : 3;
                    } else if (tilt_pitch < -15.0 && fabs(tilt_pitch) > 5.0) {
                        newORow =
                            (cursorORow > 0)
                                ? cursorORow - 1
                                : 0;
                    }
                    if (tilt_roll > 15.0 && fabs(tilt_roll) > 5.0) {
                        newOCol =
                            (cursorOCol < 3)
                                ? cursorOCol + 1
                                : 3;
                    } else if (tilt_roll < -15.0 && fabs(tilt_roll) > 5.0) {
                        newOCol =
                            (cursorOCol > 0)
                                ? cursorOCol - 1
                                : 0;
                    }

                    if (cursorORow != newORow || cursorOCol != newOCol) {
                        if (board[prevORow][prevOCol] == ' ')
                            redraw_static_element(&lcd, prevORow, prevOCol);
                        cursorORow = newORow;
                        cursorOCol = newOCol;
                        prevORow = cursorORow;
                        prevOCol = cursorOCol;
                        move_count = 0;
                    }
                }
            }

            if (currentPlayer == 'X') {
                if (!(PIND & (1 << JOY_BUTTON1_PIN)) && !buttonXPressed) {
                    buttonXPressed = 1;
                    if (board[cursorXRow][cursorXCol] == ' ') {
                        draw_cell(&lcd, cursorXRow, cursorXCol, 'X');
                        add_x_placement(&lcd, cursorXRow, cursorXCol);
                    }
                    _delay_ms(200);
                } else if (PIND & (1 << JOY_BUTTON1_PIN) && buttonXPressed) {
                    buttonXPressed = 0;
                    _delay_ms(50);
                }
            } else {
                if (!(PIND & (1 << JOY_BUTTON2_PIN)) && !buttonOPressed) {
                    buttonOPressed = 1;
                    if (board[cursorORow][cursorOCol] == ' ') {
                        draw_cell(&lcd, cursorORow, cursorOCol, 'O');
                        add_o_placement(&lcd, cursorORow, cursorOCol);
                    }
                    _delay_ms(200);
                } else if (PIND & (1 << JOY_BUTTON2_PIN) && buttonOPressed) {
                    buttonOPressed = 0;
                    _delay_ms(50);
                }
            }

            loopCount++;
            if (loopCount >=
                10) {  // Adjusted for 1 Hz blink rate (500ms per state)
                loopCount = 0;
                cursorBlinkState = !cursorBlinkState;
                if (currentPlayer == 'X' &&
                    board[cursorXRow][cursorXCol] == ' ') {
                    if (cursorBlinkState)
                        draw_cell(&lcd, cursorXRow, cursorXCol, 'X');
                    else
                        redraw_static_element(&lcd, cursorXRow, cursorXCol);
                } else if (currentPlayer == 'O' &&
                           board[cursorORow][cursorOCol] == ' ') {
                    if (cursorBlinkState)
                        draw_cell(&lcd, cursorORow, cursorOCol, 'O');
                    else
                        redraw_static_element(&lcd, cursorORow, cursorOCol);
                }
            }

            if (scoresUpdated) display_scores(&lcd);
        }

        _delay_ms(50);
    }
}