#include <stdio.h>

#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)

#define INDICATOR_BRIGHTNESS 30

#define SET_ALL(hsv) {0, 72, hsv}

enum sofle_layers {
   _MAIN,
   _BLU,
   _GRN,
   _RED
};

enum tap_dance_codes {
  TD_1,
  TD_2,
  TD_3,
  TD_4,
  TD_5
};


#ifdef ENCODER_ENABLE

bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 0) {
    switch(biton32(layer_state)){
            case _BLU:
    if (clockwise) {
        tap_code16(LGUI(KC_F3));
    } else {
        tap_code16(LGUI(KC_F2));
    }
    break;
    case _RED:
    if (clockwise) {
        tap_code16(LGUI(KC_RIGHT));
    } else {
        tap_code16(LGUI(KC_LEFT));
    }
    break;
    case _GRN:
    if (clockwise) {
            tap_code16(LSFT(LALT(KC_RIGHT)));
        } else {
            tap_code16(LSFT(LALT(KC_LEFT)));
    }
    break;
    default:
        if (clockwise) {
            tap_code(KC_VOLU);
        } else {
            tap_code(KC_VOLD);
        }
    break;
    }} else if (index == 1) {
 switch(biton32(layer_state)){
    case _BLU:
    if (clockwise) {
        tap_code16(LCTL(LALT(LSFT(KC_MINUS))));
    } else {
        tap_code16(LCTL(LALT(KC_MINUS)));
    }
    break;
    case _RED:
    if (clockwise) {
        tap_code16(LGUI(KC_TAB));
    } else {
        tap_code16(LGUI(LSFT(KC_TAB)));
    }
    break;
    case _GRN:
    if (clockwise) {
        tap_code16(LCTL(KC_D));
    } else {
        tap_code16(LCTL(KC_U));
    }
    break;
    default:
    if (clockwise) {
        tap_code16(LALT(LCTL(KC_L)));
    } else {
        tap_code16(LALT(LCTL(KC_J)));
    }
    break;
}}
return false;
};

#endif

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_MAIN] = LAYOUT(
  //,------------------------------------------------.                        ,---------------------------------------------------.
  KC_GRAVE, KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                           KC_6,    KC_7,   KC_8,    KC_9,    KC_0,    KC_MINUS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_TAB,   KC_Q,   KC_W,    KC_F,    KC_P,    KC_B,                           KC_J,    KC_L,   KC_U,    KC_Y,    KC_SCLN, KC_BSLS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
LCTL(KC_BSPC), KC_A,   KC_R,    KC_S,    KC_T,    KC_G,                           KC_M,    KC_N,   KC_E,    KC_I,    KC_O,    KC_QUOTE,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
  SC_LSPO,  KC_Z,   KC_X,    TD(TD_1),KC_D,    KC_V, KC_MUTE,       LCTL(KC_Z),KC_K,    KC_H,   KC_COMM, KC_DOT,  KC_SLSH, SC_RSPC,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
         KC_LGUI,TD(TD_5),TD(TD_2),LT(_GRN,KC_SPC),LCTL_T(KC_F12),RCTL_T(KC_ENTER),LT(_BLU,KC_SPACE),TD(TD_3),KC_ESCAPE,KC_LALT
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
        [_BLU] = LAYOUT(
  //,------------------------------------------------.                        ,---------------------------------------------------.
  KC_TRNS,  KC_F1,  KC_F2,   KC_F3,   KC_F4,   KC_F5,                          KC_F6,   KC_F7,  KC_F8,   KC_F9,   KC_F10,  KC_EQUAL,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  LCTL(KC_DEL),KC_TRNS,KC_TRNS,KC_LCBR,KC_RCBR,KC_TRNS,                       KC_GRAVE, KC_P7,  KC_P8,   KC_P9,  KC_TRNS,KC_TRNS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_BSPC,KC_TRNS,KC_MINUS,KC_LPRN,KC_RPRN,KC_DLR,                      KC_MINUS, KC_P4,  KC_P5,   KC_P6,   KC_KP_PLUS,KC_TRNS,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
    KC_LBRC,KC_TRNS,KC_TRNS, KC_LBRC, KC_RBRC, KC_TRNS,LGUI(KC_F4),          KC_TRNS,KC_TRNS,  KC_P1,  KC_P2,   KC_P3,   KC_TRNS, KC_RBRC,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
    KC_TRNS,RALT(KC_LBRC),RALT(KC_RBRC),LM(_RED,MOD_LGUI),KC_TRNS, LM(_RED,MOD_LGUI), KC_TRNS, KC_TRNS,  KC_P0, KC_BSPC
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
   [_GRN] = LAYOUT(
  //,------------------------------------------------.                         ,---------------------------------------------------.
  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,KC_TRNS, KC_TRNS,                          KC_TRNS, LALT(KC_LEFT),KC_PGUP, LALT(KC_RGHT),LALT(KC_A), KC_EQUAL,
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
LCTL(KC_DEL),KC_TRNS,KC_TRNS,KC_TRNS, KC_TRNS, KC_TRNS,                        KC_TRNS,KC_HOME,KC_UP,KC_END,KC_TRNS,LCTL(LSFT(KC_BSLS)),
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
KC_BSPC,KC_TRNS,KC_TRNS,KC_TRNS,KC_TRNS, KC_TRNS,                   LCTL(KC_LEFT),KC_LEFT,KC_DOWN, KC_RGHT,LCTL(KC_RGHT),KC_TRNS,
 //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
    SC_LAPO,KC_TRNS,KC_TRNS,KC_TRNS,KC_TRNS,KC_TRNS,KC_MPLY,   LCTL(KC_K),LCTL(KC_BSPC),KC_BSPC,KC_PGDN,KC_DELETE,LCTL(KC_DEL),SC_RCPC,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
        LGUI(KC_INS),KC_TRNS,LCTL(KC_DEL),KC_TRNS,LM(_RED,MOD_LGUI), KC_TRNS,LM(_RED,MOD_LGUI),KC_TRNS,KC_TRNS,KC_TRNS
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
      [_RED] = LAYOUT(
  //,------------------------------------------------.                         ,---------------------------------------------------.
  KC_TRNS,  KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                           KC_6,    KC_7,   KC_8,    KC_9,    KC_0,   KC_TRNS,
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
  KC_TRNS,  KC_Q   ,KC_W,    KC_F,    KC_P,    KC_B,                           KC_J,    KC_L,   KC_U,    KC_Y,    KC_SCLN, TO(_GRN),
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
  KC_TRNS,  KC_A,   KC_R,    KC_S,    KC_T,    KC_G,                           KC_M,    KC_N,   KC_E,    KC_I,    KC_O,    TO(_BLU),
  //|------+-------+--------+--------+--------+------|  ===  |         |  ===  |--------+-------+--------+--------+--------+---------|
  KC_LSFT,  KC_Z,   KC_X,    KC_C,    KC_D,    KC_V,  QK_BOOT,        LSFT(KC_B),KC_K,    KC_H,   KC_TRNS, KC_TRNS, KC_TRNS, KC_RSFT,
  //|------+-------+--------+--------+--------+------|  ===  |         |  ===  |--------+-------+--------+--------+--------+---------|
                  KC_SPC,  KC_DEL,   KC_UP,  KC_TRNS, KC_F12,         KC_ENTER,KC_TRNS, KC_DOWN,KC_ESC,  KC_SPC
  //            \--------+--------+--------+---------+-------|         |--------+---------+--------+---------+-------/
),
};

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state = {
    .is_press_action = true,
    .step = 0
};

void on_dance_1(tap_dance_state_t *state, void *user_data);
uint8_t dance_1_dance_step(tap_dance_state_t *state);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

uint8_t dance_1_dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_1_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case SINGLE_HOLD: register_code16(RALT(KC_COMMA)); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C); break;
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case SINGLE_HOLD: unregister_code16(RALT(KC_COMMA)); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state.step = 0;
}

void on_dance_2(tap_dance_state_t *state, void *user_data);
uint8_t dance_2_dance_step(tap_dance_state_t *state);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RALT(KC_QUOTE));
        tap_code16(RALT(KC_QUOTE));
        tap_code16(RALT(KC_QUOTE));
    }
    if(state->count > 3) {
        tap_code16(RALT(KC_QUOTE));
    }
}

uint8_t dance_2_dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_2_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(RALT(KC_QUOTE)); break;
        case SINGLE_HOLD: register_code16(RALT(KC_6)); break;
        case DOUBLE_TAP: register_code16(LSFT(RALT(KC_GRAVE))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LSFT(RALT(KC_GRAVE))); break;
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(RALT(KC_QUOTE)); break;
        case SINGLE_HOLD: unregister_code16(RALT(KC_6)); break;
        case DOUBLE_TAP: unregister_code16(LSFT(RALT(KC_GRAVE))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LSFT(RALT(KC_GRAVE))); break;
    }
    dance_state.step = 0;
}

void on_dance_3(tap_dance_state_t *state, void *user_data);
uint8_t dance_3_dance_step(tap_dance_state_t *state);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RALT(KC_GRAVE));
        tap_code16(RALT(KC_GRAVE));
        tap_code16(RALT(KC_GRAVE));
    }
    if(state->count > 3) {
        tap_code16(RALT(KC_GRAVE));
    }
}

uint8_t dance_3_dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_3_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(RALT(KC_GRAVE)); break;
        case SINGLE_HOLD: register_code16(LSFT(RALT(KC_QUOTE))); break;
        case DOUBLE_TAP: register_code16(RALT(KC_B)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RALT(KC_B)); break;
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(RALT(KC_GRAVE)); break;
        case SINGLE_HOLD: unregister_code16(LSFT(RALT(KC_QUOTE))); break;
        case DOUBLE_TAP: unregister_code16(RALT(KC_B)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RALT(KC_B)); break;
    }
    dance_state.step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
    [TD_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
    [TD_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
    [TD_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
    [TD_4] = ACTION_TAP_DANCE_DOUBLE(KC_BSPC, LALT(KC_BSPC)),
    [TD_5] = ACTION_TAP_DANCE_DOUBLE(KC_DEL, LCTL(KC_DEL))
};



#ifdef OLED_ENABLE

static void render_logo(void) {
    static const char PROGMEM qmk_logo[] = {
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94,
        0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4,
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0x00
    };

    oled_write_P(qmk_logo, false);
}

static void print_status_narrow(void) {
    // Print current mode
    oled_write_P(PSTR("\n\n"), false);
    oled_write_ln_P(PSTR("Tim\nVdL"), false);

    oled_write_ln_P(PSTR(""), false);

    oled_write_P(PSTR("\n\n"), false);
    // Print current layer
    oled_write_ln_P(PSTR("LAYER"), false);
    switch (get_highest_layer(layer_state)) {
        case _RED:
            oled_write_P(PSTR("Red  "), false);
            break;
        case _BLU:
            oled_write_P(PSTR("Blue"), false);
            break;
        case _GRN:
            oled_write_P(PSTR("Green"), false);
            break;
        default:
            oled_write_ln_P(PSTR("Base"), false);
    }
}

oled_rotation_t oled_init_user(oled_rotation_t rotation) {
    if (is_keyboard_master()) {
        return OLED_ROTATION_270;
    }
    return rotation;
}

bool oled_task_user(void) {
    if (is_keyboard_master()) {
        print_status_narrow();
    } else {
        render_logo();
    }
    return false;
}

#endif


#ifdef RGBLIGHT_ENABLE
char layer_state_str[72];

const rgblight_segment_t PROGMEM layer_main_lights[] = RGBLIGHT_LAYER_SEGMENTS(
  SET_ALL(HSV_BLACK)
);

const rgblight_segment_t PROGMEM layer_blu_lights[] = RGBLIGHT_LAYER_SEGMENTS(
    SET_ALL(HSV_BLUE)
    );

const rgblight_segment_t PROGMEM layer_grn_lights[] = RGBLIGHT_LAYER_SEGMENTS(
    SET_ALL(HSV_GREEN)
);

const rgblight_segment_t PROGMEM layer_red_lights[] = RGBLIGHT_LAYER_SEGMENTS(
  SET_ALL(HSV_RED)
);

const rgblight_segment_t* const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(

    layer_main_lights,
    layer_blu_lights,
    layer_grn_lights,
    layer_red_lights

);

layer_state_t layer_state_set_user(layer_state_t state) {
    rgblight_set_layer_state(0, layer_state_cmp(state, _MAIN) && layer_state_cmp(default_layer_state,_MAIN));

    rgblight_set_layer_state(1, layer_state_cmp(state, _BLU));
    rgblight_set_layer_state(2, layer_state_cmp(state, _GRN));
    rgblight_set_layer_state(3, layer_state_cmp(state, _RED));

    return state;
}

void keyboard_post_init_user(void) {
    // Enable the LED layers
    rgblight_layers = my_rgb_layers;

    rgblight_mode(10);// haven't found a way to set this in a more useful way

}
#endif
