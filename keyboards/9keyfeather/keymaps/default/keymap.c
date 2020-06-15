#include QMK_KEYBOARD_H

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* LAYER 0
    * ,-----------------------.
    * |   1   |   2   |   3   |
    * |-------+-------+-------|
    * |   4   |   5   |   6   |
    * |-------+-------+-------|
    * |   7   |   8   | 9/FN  | 9/FN = Hold 9 for FN
    * `-----------------------'
    */
    [0] = LAYOUT( \
    KC_1,       KC_2,      KC_3,       \
    KC_4,       KC_5,      KC_6,       \
    KC_7,       KC_8,      LT(1, KC_9) \
    ),

    /* LAYER 1
    * ,-----------------------.
    * |  AUTO |  USB  |  BT   |
    * |-------+-------+-------|
    * |  ENT  |   *   |   /   |
    * |-------+-------+-------|
    * |  00   |   .   |       |
    * `-----------------------'
    */
    [1] = LAYOUT( \
    OUT_AUTO, OUT_USB, OUT_BT, \
    KC_ENTER, RESET,   KC_SLSH, \
    KC_0,     KC_DOT,  KC_TRNS  \
    )
};

#ifdef OLED_DRIVER_ENABLE
static void render_logo(void) {
    static const char PROGMEM qmk_logo[] = {
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94,
        0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4,
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0x00
    };
    oled_write_P(qmk_logo, false);
}

void oled_task_user(void) {
    render_logo();
}
#endif
