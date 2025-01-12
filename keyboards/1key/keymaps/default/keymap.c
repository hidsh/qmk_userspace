// Copyright 2023 QMK
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H     // KC_xxx
#define XXX KC_NO

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* |<------------ mcu ------------>|    |<------- i/o expander -------->|
     *   c0  c1  c2  c3  c4  c5  c6  c7      c15 c14 c13 c12 c11 c10  c9  c8   (ino right pcb)
     * ┌───┬───┬───┬───┬───┬───┬───┬───┐    ┌───┬───┬───┬───┬───┬───┬───┬───┐
     * │ q │   │   │   │   │   │   │   │ r0 │   │ 7 │ 8 │ 9 │ 0 │ - │ = │ bs│
     * ├───┼───┼───┼───┼───┼───┼───┼───┤    ├───┼───┼───┼───┼───┼───┼───┼───┤
     * │   │   │   │   │   │   │   │   │ r1 │ y │ u │ i │ o │ p │ [ │ ] │ \ │
     * ├───┼───┼───┼───┼───┼───┼───┼───│    ├───┼───┼───┼───┼───┼───┼───┼───│
     * │   │   │   │   │   │   │   │   │ r2 │ h │ j │ k │ l │ ; │ ' │ret│   │
     * ├───┼───┼───┼───┼───┼───┼───┼───┤    ├───┼───┼───┼───┼───┼───┼───┼───┤
     * │   │   │   │   │   │   │   │   │ r3 │ b │ n │ m │ , │ . │ / │ up│del│
     * └───┴───┴───┴───┴───┴───┴───┴───┘    └───┴───┴───┴───┴───┴───┴───┴───┘
     */
    [0] = LAYOUT(
    /* col:0   1    2    3    4    5    6    7       8     9    10    11       12       13        14        15    */
        KC_1, XXX, XXX, XXX, XXX, XXX, XXX, XXX,   XXX,  KC_7, KC_8, KC_9,    KC_0,    KC_MINUS, KC_EQUAL, KC_BSPC, /* row:0 */
        XXX,  XXX, XXX, XXX, XXX, XXX, XXX, XXX,   KC_Y, KC_U, KC_I, KC_O,    KC_P,    KC_LBRC,  KC_RBRC,  KC_BSLS, /*     1 */
        XXX,  XXX, XXX, XXX, XXX, XXX, XXX, XXX,   KC_H, KC_J, KC_K, KC_L,    KC_SCLN, KC_QUOT,  KC_ENT,   XXX,     /*     2 */
        XXX,  XXX, XXX, XXX, XXX, XXX, XXX, XXX,   KC_B, KC_N, KC_M, KC_COMM, KC_DOT,  KC_SLSH,  KC_UP,    KC_DEL   /*     3 */
    )
};
