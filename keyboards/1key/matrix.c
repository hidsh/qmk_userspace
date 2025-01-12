#include <wait.h>           // wait_ms, wait_us
#include <debug.h>          // debug_enable, dprintf, printf
#include "i2c_master.h"     // I2C_STATUS_SUCCESS, i2c_status_t, i2c_init, ...
#include "matrix.h"         // <-- info_config.h <-- keyboard.json
                            // MATRIX_COLS, MATRIX_ROWS, MATRIX_COL_PINS, MATRIX_ROW_PINS, ...
#include "mcp23017.h"

#define PASSERT(COND)                           \
    do {                                        \
        if (!(COND)) {                          \
            printf("  PASSERT: %s:%d:%s()|%s| Unmatch conditions, aborted.\n",  \
                   __FILE__, __LINE__, __func__, #COND);                        \
            while(1) { wait_ms(100); };                     /* infinite loop */ \
        }                                                                       \
    } while (0);

#define MCP23017_ERROR_PRINT(RESP)                   \
    do {                                        \
        printf("  ERROR: %s:%d:%s() got an error related to the I2C| addr=0x%02X (%08b), A2=%d, A1=%d, A0=%d| resp=%d (%d:Timeout, %d:AnotherErr)\n",       \
               __FILE__, __LINE__, __func__,                                                    \
               MCP23017_ADDR, MCP23017_ADDR, MCP23017_PIN_A2, MCP23017_PIN_A1, MCP23017_PIN_A0, \
               RESP, I2C_STATUS_TIMEOUT, I2C_STATUS_ERROR);                                     \
    } while (0);

#define MCP23017_TIMEOUT_MS ((uint16_t)100)     // todo adj

/////////////////////////////////////////////////////////////////////////////////////

// PIN_A2, PIN_A1, PIN_A0: mcp23017 device address (refer connections for them in your schematic)
#define MCP23017_PIN_A2 (0)         // 0: LO,  1: HI
#define MCP23017_PIN_A1 (0)
#define MCP23017_PIN_A0 (0)

#define  MCP23017_ADDR ((uint8_t)(0x20 | (MCP23017_PIN_A2 << 2) | (MCP23017_PIN_A1 << 1) | (MCP23017_PIN_A0 << 0)))

int mcp23017_init(void)
{
    static_assert(MCP23017_PIN_A2 == 0 || MCP23017_PIN_A2 == 1);
    static_assert(MCP23017_PIN_A1 == 0 || MCP23017_PIN_A1 == 1);
    static_assert(MCP23017_PIN_A0 == 0 || MCP23017_PIN_A0 == 1);

    i2c_init();
    wait_ms(100);

    // check connection
    const i2c_status_t resp = i2c_ping_address(MCP23017_ADDR << 1, MCP23017_TIMEOUT_MS);

    if (resp == I2C_STATUS_SUCCESS) return 0;

    MCP23017_ERROR_PRINT(resp);
    return -1;
}

int mcp23017_write(MCP23017_REG_t reg, uint8_t data)
{
    const i2c_status_t resp = i2c_write_register(MCP23017_ADDR << 1, (uint8_t)reg, &data, 1, MCP23017_TIMEOUT_MS);

    dprintf("  %s()| addr=0x%02X, reg=%d, data=%0x02X| resp=%d (%d:OK, %d:Timeout, %d:AnotherErr)\n",
            __func__, MCP23017_ADDR, reg, data,
            resp, I2C_STATUS_SUCCESS, I2C_STATUS_TIMEOUT, I2C_STATUS_ERROR);

    if (resp == I2C_STATUS_SUCCESS) return 0;

    MCP23017_ERROR_PRINT(resp);
    return -1;
}

int mcp23017_read(MCP23017_REG_t reg, uint8_t *pdata)
{
    const i2c_status_t resp = i2c_read_register(MCP23017_ADDR << 1, (uint8_t)reg, pdata, 1, MCP23017_TIMEOUT_MS);

    dprintf("  %s()| addr=0x%02X, reg=%d, pdata=%0x02X| resp=%d (%d:OK, %d:Timeout, %d:AnotherErr)\n",
            __func__, MCP23017_ADDR, reg, *pdata,
            resp, I2C_STATUS_SUCCESS, I2C_STATUS_TIMEOUT, I2C_STATUS_ERROR);

    if (resp == I2C_STATUS_SUCCESS) return 0;

    MCP23017_ERROR_PRINT(resp);
    return -1;
}

/////////////////////////////////////////////////////////////////////////////////////
// +------------------+     +-------------------------------------------------------+
// | @keyboard.json   | --> | @qmk_firmware/.build/obj_KBD_KEYMAP/src/info_config.h |
// +------------------+     +-------------------------------------------------------+
// |  diode_direction |     | DIODE_DIRECTION                                       |
// |  matrix_pins     |     |                                                       |
// |    .cols         |     |  MATRIX_COLS                                          |
// |                  |     |  MATRIX_COL_PINS                                      |
// |    .rows         |     |  MATRIX_ROWS                                          |
// |                  |     |  MATRIX_ROW_PINS                                      |
// +------------------+     +-------------------------------------------------------+
//
// DIODE_DIRECTION ROW2COL
//                 ROWS(output, active-low) => COLS(input, pull-up, negative-logic)
//
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;

#define MATRIX_COLS_EXPANDER_INDEX  (1)

matrix_row_t make_bitmask(int n_start, int bit_length)
{
    matrix_row_t mask = 0;
    for(int i=0; i<bit_length; i++) {
        mask |= 1 << i;
    }

    return mask << n_start;
}

// -----------------------------
// mcu gpio
// -----------------------------
void mcu_unselect_rows(void)
{
    for(uint8_t r=0; r<MATRIX_ROWS; r++) {
        const pin_t pin = row_pins[r];
        if (pin == NO_PIN) continue;

        gpio_write_pin_high(pin);                           // "unselect" means output HI
    }
}

void mcu_select_row(uint8_t r)
{
    PASSERT(r >= 0 && r < MATRIX_ROWS);

    const pin_t pin = row_pins[r];
    if (pin == NO_PIN) return;

    gpio_write_pin_low(pin);                                // "select" means output LO
}

matrix_row_t mcu_read_cols(void)
{
    matrix_row_t cols = 0;

    for(matrix_row_t c=0; c<MATRIX_COLS; c++) {
        const pin_t pin = col_pins[c];
        if (pin == NO_PIN) continue;

        if (gpio_read_pin(pin) == 0) {                      // 0: press,      1: release
            cols |= (1 << c);                               // 0: release,    1: press
        }
    }

    return cols;
}

// -----------------------------
// i/o expander (mcp23017)
// -----------------------------
inline int expander_unselect_rows(void)
{
    // active-low
    return mcp23017_write(MCP23017_REG_GPIOA, 0xFF);        // 0*: output LO,   1: output HI
}

inline int expander_select_row(uint8_t row_num)
{
    PASSERT(row_num >= 0 && row_num < MATRIX_ROWS);

    const uint8_t bits = 0xFF & ~(1 << row_num);            // active-low
    return mcp23017_write(MCP23017_REG_GPIOA, bits);        // 0*: output LO,   1: output HI
}

inline int expander_read_cols(uint8_t *pcols)
{
    return mcp23017_read(MCP23017_REG_GPIOB, (uint8_t *)pcols); // negative-logic (LO:press -> 1)
}

/////////////////////////////////////////////////////////////////////////////////////
#define COLS_EXPANDER_START (8)

void keyboard_post_init_user(void) {
  // Customise these values to desired behaviour
    // debug_enable = true;
    debug_matrix = false;
    //debug_keyboard = true;
    //debug_mouse = true;

    printf("p:init_custom\n");
    dprintf("init_custom\n");
    wait_ms(100);
}

void matrix_init_custom(void)
{
    // -----------------------------
    // pin setting for mcu gpio
    // -----------------------------
    for(uint8_t r=0; r<MATRIX_ROWS; r++) {
        const pin_t pin = row_pins[r];
        if (pin == NO_PIN) continue;

        gpio_set_pin_output(pin);                   // set rows: output, totem-pole
        gpio_write_pin_high(pin);                   // "unselect" means output HI
    }

    for(uint8_t c=0; c<MATRIX_COLS; c++) {
        const pin_t pin = col_pins[c];
        if (pin == NO_PIN) continue;

        gpio_set_pin_input_high(pin);               // set cols: input, pull-up
    }

    // -----------------------------
    // pin setting for mcp23017
    // -----------------------------
    mcp23017_init();

    // todo ino (this is for test) --> biz
    // GPAx --> rows: output, active-low
    //                                              // (*: reset value)
    mcp23017_write(MCP23017_REG_IPOLA,  0x00);      // 0*: read LO => 0,    1 : read LO => 1
    mcp23017_write(MCP23017_REG_GPPUA,  0x00);      // 0*: disable pullup,  1 : enable pullup
    mcp23017_write(MCP23017_REG_IODIRA, 0x00);      // 0 : output,          1*: input

    // GPBx --> COLS: input, pullup
    mcp23017_write(MCP23017_REG_IPOLB,  0xFF);      // 0*: read LO => 0,    1 : read LO => 1 (* reset value)
    mcp23017_write(MCP23017_REG_GPPUB,  0xFF);      // 0*: disable pullup,  1 : enable pullup
    mcp23017_write(MCP23017_REG_IODIRB, 0xFF);      // 0 : output,          1*: input

    expander_unselect_rows();
}

// memo:
//   current_matrix[] <-- raw_matrix[] <-- quantum/matrix_common.c
//   MATRIX_ROWS <-- qmk_firmware/.build/obj_KBD_KEYMAP/src/info_config.h
bool matrix_scan_custom(matrix_row_t current_matrix[])
{
    static_assert(COLS_EXPANDER_START >= 0 && COLS_EXPANDER_START <= MATRIX_COLS);
    dprintf("scan_custom\n");

    bool matrix_has_changed = false;
    matrix_row_t mask;

    // mcu's gpios
    mask = make_bitmask(0, COLS_EXPANDER_START);

    for(uint8_t r=0; r<MATRIX_ROWS; r++) {
        mcu_unselect_rows();
        mcu_select_row(r);
        wait_us(2);               // todo adjust

        const matrix_row_t cols = mcu_read_cols();      // 0: release,    1: press

        if ((current_matrix[r] & mask) != cols) {
            current_matrix[r] &= mask;
            current_matrix[r] = cols;
            matrix_has_changed = true;

            // printf("mcu  r:%d, c:%016b\n", r, cols);
        }
    }

    // i/o expander
    mask = make_bitmask(COLS_EXPANDER_START, MATRIX_COLS - COLS_EXPANDER_START);

    for(uint8_t r=0; r<MATRIX_ROWS; r++) {
        expander_unselect_rows();
        expander_select_row(r);
        wait_us(2);               // todo adjust

        uint8_t cols8 = 0;
        expander_read_cols(&cols8);                     // 0: release,  1: press
        const matrix_row_t cols = (matrix_row_t)cols8 << COLS_EXPANDER_START;

        if ((current_matrix[r] & mask) != cols) {
            printf("mcu matrix[%d]:%016b, cols:%016b -> ", r, current_matrix[r], cols);
            current_matrix[r] &= mask;
            current_matrix[r] = cols;
            printf("%016b\n", current_matrix[r]);
            matrix_has_changed = true;

            // printf("exp  r:%d, c:%016b\n", r, cols);
        }
    }

    return matrix_has_changed;
}
