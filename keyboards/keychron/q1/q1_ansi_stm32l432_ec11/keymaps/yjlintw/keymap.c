/* Copyright 2021 @ Keychron (https://www.keychron.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H

#ifdef VIA_ENABLE
    #define USER_START USER00
#else
    #define USER_START SAFE_RANGE
#endif

enum layers{
    WIN_BASE,
    WIN_FN,
    WIN_NUM,
    WIN_EXTRA,
    WIN_GAME,
    LAYER_EXTRA1,
    LAYER_EXTRA2,
    LAYER_EXTRA3
};

enum custom_keycodes {
    KC_MISSION_CONTROL = USER_START,
    KC_LAUNCHPAD,
    KC_LOPTN,
    KC_ROPTN,
    KC_LCMMD,
    KC_RCMMD,
    KC_TASK_VIEW,
    KC_FILE_EXPLORER
};

#define KC_MCTL KC_MISSION_CONTROL
#define KC_LPAD KC_LAUNCHPAD
#define KC_TASK KC_TASK_VIEW
#define KC_FLXP KC_FILE_EXPLORER

typedef struct PACKED {
    uint8_t len;
    uint8_t keycode[2];
} key_combination_t;

key_combination_t key_comb_list[2] = {
    {2, {KC_LWIN, KC_TAB}},
    {2, {KC_LWIN, KC_E}}
};


static uint8_t mac_keycode[4] = { KC_LOPT, KC_ROPT, KC_LCMD, KC_RCMD };

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [WIN_BASE] = LAYOUT_all(
        KC_ESC,   KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,    KC_F12,   C(KC_PSCR),         KC_MUTE,
        KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,   KC_EQL,   KC_BSPC,            TO(WIN_GAME),
        KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,   KC_RBRC,  KC_BSLS,            G(C(KC_T)),
 CTL_T(KC_ESC),   KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,             KC_ENT,             KC_DEL,
        KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,             KC_RSFT,  KC_UP,
        KC_LCTL,  KC_LGUI,  KC_LALT,                                LT(1, KC_SPC),KC_VOLD,KC_VOLU,          KC_RALT,  G(KC_SPC),TT(WIN_FN),KC_LEFT,KC_DOWN,  KC_RGHT),

    [WIN_FN] = LAYOUT_all(
        TG(WIN_FN),TG(WIN_FN),TG(WIN_NUM),TG(WIN_EXTRA),  TG(WIN_GAME),  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_PSCR,            RGB_TOG,
        KC_TRNS,  KC_P1  ,  KC_P2  ,  KC_P3  ,  KC_P4  ,  KC_P5  ,  KC_P6  ,  KC_P7  ,  KC_P8  ,  KC_P9  ,  KC_P0  ,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  C(KC_INS),C(KC_Z),  TG(1)  ,  KC_TRNS,  S(KC_INS),KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_NUM,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_DEL ,  KC_TRNS,  KC_TRNS,  KC_LEFT,  KC_DOWN,  KC_UP  ,  KC_RIGHT, KC_TRNS,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_SPC ,  KC_HOME,  KC_PGDN,  KC_PGUP,  KC_END ,  KC_TRNS,             KC_TRNS,  KC_PGUP,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                MO(WIN_EXTRA),RGB_VAD,RGB_VAI,          KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_HOME,  KC_PGDN,  KC_END),
    
    [WIN_NUM] = LAYOUT_all(
        TG(WIN_FN),TG(WIN_FN),TG(WIN_NUM),TG(WIN_EXTRA),  TG(WIN_GAME),  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_NUM,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_P7  ,  KC_P8  ,  KC_P9  ,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_P4  ,  KC_P5  ,  KC_P6  ,  KC_TRNS,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_P1  ,  KC_P2  ,  KC_P3  ,  KC_TRNS,             KC_TRNS,  KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                KC_P0  ,  KC_TRNS,  KC_TRNS,            KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS),
    
    [WIN_EXTRA] = LAYOUT_all(
        TG(WIN_FN),TG(WIN_FN),TG(WIN_NUM),TG(WIN_EXTRA),  TG(WIN_GAME),  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_HOME,  KC_PGDN,  KC_PGUP,  KC_END ,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,  KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                KC_TRNS,  KC_TRNS,  KC_TRNS,            KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS),
    
    [WIN_GAME] = LAYOUT_all(
        KC_ESC,   KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,    KC_F12,   C(KC_PSCR),         KC_MUTE,
        KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,   KC_EQL,   KC_BSPC,            TO(WIN_BASE),
        KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,   KC_RBRC,  KC_BSLS,            KC_NO,
 CTL_T(KC_ESC),   KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,             KC_ENT,             KC_DEL,
        KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,             KC_RSFT,  KC_UP,
        KC_LCTL,  KC_LGUI,  KC_LALT,                                KC_SPC, KC_VOLD, KC_VOLU,     KC_RALT,  MO(WIN_FN),KC_RCTRL, KC_LEFT,  KC_DOWN,  KC_RGHT),

    [5] = LAYOUT_all(
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,  KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                KC_TRNS,  KC_TRNS,  KC_TRNS,            KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS),

    [6] = LAYOUT_all(
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,  KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                KC_TRNS,  KC_TRNS,  KC_TRNS,            KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS),

    [7] = LAYOUT_all(
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,            KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,            KC_TRNS,
        KC_TRNS,            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,             KC_TRNS,  KC_TRNS,
        KC_TRNS,  KC_TRNS,  KC_TRNS,                                KC_TRNS,  KC_TRNS,  KC_TRNS,            KC_TRNS,  KC_TRNS,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS)
};

#if defined(VIA_ENABLE) && defined(ENCODER_ENABLE)

#define ENCODERS 1
static uint8_t  encoder_state[ENCODERS] = {0};
static keypos_t encoder_cw[ENCODERS]    = {{ 8, 5 }};
static keypos_t encoder_ccw[ENCODERS]  = {{ 7, 5 }};

void encoder_action_unregister(void) {
    for (int index = 0; index < ENCODERS; ++index) {
        if (encoder_state[index]) {
            keyevent_t encoder_event = (keyevent_t) {
                .key = encoder_state[index] >> 1 ? encoder_cw[index] : encoder_ccw[index],
                .pressed = false,
                .time = (timer_read() | 1)
            };
            encoder_state[index] = 0;
            action_exec(encoder_event);
        }
    }
}

void encoder_action_register(uint8_t index, bool clockwise) {
    keyevent_t encoder_event = (keyevent_t) {
        .key = clockwise ? encoder_cw[index] : encoder_ccw[index],
        .pressed = true,
        .time = (timer_read() | 1)
    };
    encoder_state[index] = (clockwise ^ 1) | (clockwise << 1);
    action_exec(encoder_event);
}

void matrix_scan_user(void) {
    encoder_action_unregister();
}

bool encoder_update_user(uint8_t index, bool clockwise) {
    encoder_action_register(index, clockwise);
    return false;
};

#endif

void rgb_matrix_indicators_user(void) {
    HSV curr = rgb_matrix_get_hsv();
    HSV red = {0, 255, curr.v};
    RGB rgb_white = hsv_to_rgb(curr);
    RGB rgb_red = hsv_to_rgb(red);

    switch (get_highest_layer(layer_state))
    {
        case WIN_BASE:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            break;
        case WIN_FN:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(1, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case WIN_NUM:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(2, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case WIN_EXTRA:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(3, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case WIN_GAME:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(4, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case LAYER_EXTRA1:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(5, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case LAYER_EXTRA2:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(6, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
        case LAYER_EXTRA3:
            rgb_matrix_set_color_all(rgb_white.r, rgb_white.g, rgb_white.b);
            rgb_matrix_set_color(7, rgb_red.r, rgb_red.g, rgb_red.b);
            break;
    }

}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case KC_LOPTN:
        case KC_ROPTN:
        case KC_LCMMD:
        case KC_RCMMD:
            if (record->event.pressed) {
                register_code(mac_keycode[keycode - KC_LOPTN]);
            } else {
                unregister_code(mac_keycode[keycode - KC_LOPTN]);
            }
            return false;
        case KC_MCTL:
            if (record->event.pressed) {
                host_consumer_send(0x29F);
            } else {
                host_consumer_send(0);
            }
            return false;  // Skip all further processing of this key
        case KC_LPAD:
            if (record->event.pressed) {
                host_consumer_send(0x2A0);
            } else {
                host_consumer_send(0);
            }
            return false;  // Skip all further processing of this key
        case KC_TASK:
        case KC_FLXP:
            if (record->event.pressed) {
                for (uint8_t i = 0; i < key_comb_list[keycode - KC_TASK].len; i++) {
                    register_code(key_comb_list[keycode - KC_TASK].keycode[i]);
                }
            } else {
                for (uint8_t i = 0; i < key_comb_list[keycode - KC_TASK].len; i++) {
                    unregister_code(key_comb_list[keycode - KC_TASK].keycode[i]);
                }
            }
            return false;  // Skip all further processing of this key
        default:
            return true;  // Process all other keycodes normally
    }
}
