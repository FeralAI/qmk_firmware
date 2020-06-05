#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include "wait.h"
#include "action_layer.h"
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "mcp23017.h"
#include "quantum.h"

/*
* This constant define not debouncing time in msecs, but amount of matrix
* scan loops which should be made to get stable debounced results.
*
* On Ergodox matrix scan rate is relatively low, because of slow I2C.
* Now it's only 317 scans/second, or about 3.15 msec/scan.
* According to Cherry specs, debouncing time is 5 msec.
*
* And so, there is no sense to have DEBOUNCE higher than 2.
*/

#ifndef DEBOUNCE
#define DEBOUNCE 5
#endif

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
// static pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;

// Debouncing: store for each key the number of scans until it's eligible to
// change.  When scanning the matrix, ignore any changes in keys that have
// already changed in the last DEBOUNCE scans.
static uint8_t debounce_matrix[MATRIX_ROWS * MATRIX_COLS];

static void init_cols(void);
static void init_rows(void);
static matrix_row_t read_cols(void);
static void select_row(uint8_t row);
static void unselect_row(uint8_t);

__attribute__ ((weak))
void matrix_init_user(void) {}

__attribute__ ((weak))
void matrix_scan_user(void) {}

__attribute__ ((weak))
void matrix_init_kb(void) {
    matrix_init_user();
}

__attribute__ ((weak))
void matrix_scan_kb(void) {
    matrix_scan_user();
}

inline
uint8_t matrix_rows(void) {
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void) {
    return MATRIX_COLS;
}

void matrix_init(void) {
    mcp23017_begin(0);
    init_rows();
    init_cols();

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        for (uint8_t j=0; j < MATRIX_COLS; ++j) {
            debounce_matrix[i * MATRIX_COLS + j] = 0;
        }
    }

    matrix_init_quantum();
}

void matrix_power_up(void) {
    init_rows();
    init_cols();

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
    }
}

// Returns a matrix_row_t whose bits are set if the corresponding key should be
// eligible to change in this scan.
matrix_row_t debounce_mask(uint8_t row)
{
    matrix_row_t result = 0;
    for (uint8_t j=0; j < MATRIX_COLS; ++j) {
        if (debounce_matrix[row * MATRIX_COLS + j]) {
            --debounce_matrix[row * MATRIX_COLS + j];
        } else {
            result |= (1 << j);
        }
    }
    return result;
}

// Report changed keys in the given row.  Resets the debounce countdowns
// corresponding to each set bit in 'change' to DEBOUNCE.
void debounce_report(matrix_row_t change, uint8_t row) {
    for (uint8_t i = 0; i < MATRIX_COLS; ++i) {
        if (change & (1 << i)) {
            debounce_matrix[row * MATRIX_COLS + i] = DEBOUNCE;
        }
    }
}

uint8_t matrix_scan(void) {
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        uint8_t pin = row_pins[i];
        select_row(pin);
        // FIXME: Skip wait for now, i2c should be slow enough
        // wait_us(30);
        matrix_row_t mask = debounce_mask(i);
        matrix_row_t cols = (read_cols() & mask) | (matrix[i] & ~mask);
        debounce_report(cols ^ matrix[i], i);
        matrix[i] = cols;
        unselect_row(pin);
    }

    matrix_scan_quantum();

    return 1;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col) {
    return (matrix[row] & ((matrix_row_t)1<<col));
}

inline
matrix_row_t matrix_get_row(uint8_t row) {
    return matrix[row];
}

void matrix_print(void) {
    print("\nr/c 0123456789ABCDEF\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        phex(row); print(": ");
        pbin_reverse16(matrix_get_row(row));
        print("\n");
    }
}

uint8_t matrix_key_count(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        count += bitpop16(matrix[i]);
    }
    return count;
}

static void init_cols(void) {
    mcp23017_writeRegister(MCP23017_IODIRB, 0xFF); // Set column directions to INPUT
    mcp23017_writeRegister(MCP23017_GPIOB, 0xFF);  // Set logic levels to HIGH
    mcp23017_writeRegister(MCP23017_GPPUB, 0xFF);  // Set pull-up resistors ON
}

static void init_rows(void) {
    mcp23017_writeRegister(MCP23017_IODIRA, 0xFF); // Set column directions to INPUT
    mcp23017_writeRegister(MCP23017_GPIOA, 0x00);  // Set logic levels to LOW
    mcp23017_writeRegister(MCP23017_GPPUA, 0x00);  // Set pull-up resistors OFF
}

static matrix_row_t read_cols() {
    return ~mcp23017_readGPIO(1);
}

static void select_row(uint8_t pin) {
    // Enable row by setting direction to OUTPUT
    mcp23017_pinMode(pin, OUTPUT);
}

static void unselect_row(uint8_t pin) {
    // Disable row by setting direction to INPUT
    mcp23017_pinMode(pin, INPUT);
}

