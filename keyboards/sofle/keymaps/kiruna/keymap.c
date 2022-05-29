#include <stdio.h>

#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)


#define INDICATOR_BRIGHTNESS 30

#define HSV_OVERRIDE_HELP(h, s, v, Override) h, s , Override
#define HSV_OVERRIDE(hsv, Override) HSV_OVERRIDE_HELP(hsv,Override)

// Light combinations
#define SET_INDICATORS(hsv) \
    {0, 1, HSV_OVERRIDE_HELP(hsv, INDICATOR_BRIGHTNESS)}, \
    {35+0, 1, hsv}
#define SET_UNDERGLOW(hsv) \
    {1, 6, hsv}, \
    {35+1, 6,hsv}
#define SET_NUMPAD(hsv)     \
    {35+15, 5, hsv},\
      {35+22, 3, hsv},\
      {35+27, 3, hsv}
#define SET_NUMROW(hsv) \
    {10, 2, hsv}, \
        {20, 2, hsv}, \
        {30, 2, hsv}, \
      {35+ 10, 2, hsv}, \
      {35+ 20, 2, hsv}, \
      {35+ 30, 2, hsv}
#define SET_INNER_COL(hsv)  \
    {33, 4, hsv}, \
      {35+ 33, 4, hsv}

#define SET_OUTER_COL(hsv) \
    {7, 4, hsv}, \
      {35+ 7, 4, hsv}
#define SET_THUMB_CLUSTER(hsv)  \
    {25, 2, hsv}, \
      {35+ 25, 2, hsv}
#define SET_LAYER_ID(hsv)   \
    {0, 1, HSV_OVERRIDE_HELP(hsv, INDICATOR_BRIGHTNESS)}, \
    {35+0, 1, HSV_OVERRIDE_HELP(hsv, INDICATOR_BRIGHTNESS)}, \
        {1, 6, hsv}, \
    {35+1, 6, hsv}, \
        {7, 4, hsv}, \
      {35+ 7, 4, hsv}, \
        {25, 2, hsv}, \
      {35+ 25, 2, hsv}

enum sofle_layers {
   _MAIN,
   _BLU,
   _GRN,
   _RED
};

enum tap_dance_codes {
  DANCE_1,
  DANCE_2,
  DANCE_3,
};


#ifdef ENCODER_ENABLE

bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 1) {
        if (clockwise) {
            tap_code(KC_VOLU);
        } else {
            tap_code(KC_VOLD);
        }
        } else if (index == 0) {
 switch(biton32(layer_state)){
    case _BLU:
    if (clockwise) {
        tap_code16(LCTL(LSFT(KC_MINUS)));
    } else {
        tap_code16(LCTL(KC_MINUS));
    }
    break;
    case _RED:
    if (clockwise) {
        tap_code16(KC_O);
    } else {
        tap_code16(KC_M);
    }
    break;            
    case _GRN:
    if (clockwise) {
        tap_code16(LGUI(KC_D));
    } else {
        tap_code16(LGUI(KC_U));
    }
    break;
    default:
    if (clockwise) {
        tap_code16(LALT(KC_DOWN));
    } else {
        tap_code16(LALT(KC_UP));
    }  
    break;
}}
return false;
};

#endif

uint32_t layer_state_set_user(uint32_t state) {
#ifdef RGBLIGHT_ENABLE
    switch (biton32(state)) {
        case _BLU:
        rgblight_setrgb(RGB_BLUE);
        break;
        case _RED:
        rgblight_setrgb(RGB_RED);
        break;
        case _GRN:
        rgblight_setrgb(RGB_GREEN);
        break;
        default: 
        rgblight_setrgb(RGB_OFF);
        break;
    }
#endif
    return state;
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_MAIN] = LAYOUT(
  //,------------------------------------------------.                        ,---------------------------------------------------.
  KC_GRAVE, KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                           KC_6,    KC_7,   KC_8,    KC_9,    KC_0,    KC_MINUS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_TAB,   KC_Q,   KC_W,    KC_F,    KC_P,    KC_B,                           KC_J,    KC_L,   KC_U,    KC_Y,    KC_SCLN, KC_BSLASH,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_BSPC,  KC_A,   KC_R,    KC_S,    KC_T,    KC_G,                           KC_M,    KC_N,   KC_E,    KC_I,    KC_O,    KC_QUOTE,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
  KC_LSPO,  KC_Z,   KC_X, TD(DANCE_1),KC_D,    KC_V, LGUI(KC_Z),       KC_MUTE,KC_K,    KC_H,   KC_COMM, KC_DOT,  KC_SLSH, KC_RSPC,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
   OSM(MOD_LCTL),KC_DEL,TD(DANCE_2), LT(_GRN,KC_SPC),LGUI_T(KC_F12),RGUI_T(KC_ENTER),LT(_BLU,KC_SPACE),TD(DANCE_3),KC_ESCAPE,OSM(MOD_RALT)
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
        [_BLU] = LAYOUT(
  //,------------------------------------------------.                        ,---------------------------------------------------.
  KC_TRNS,  KC_F1,  KC_F2,   KC_F3,   KC_F4,   KC_F5,                          KC_F6,   KC_F7,  KC_F8,   KC_F9,   KC_F10,  KC_EQUAL,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  LALT(KC_DEL),KC_TRNS,KC_TRNS,KC_LCBR,KC_RCBR,KC_TRNS,                       KC_GRAVE, KC_P7,  KC_P8,   KC_P9,  KC_BSLASH,KC_TRNS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  LALT(KC_BSPC),KC_TRNS,KC_MINUS,KC_LPRN,KC_RPRN,KC_DLR,                      KC_MINUS, KC_P4,  KC_P5,   KC_P6,   KC_QUOTE,KC_TRNS,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
LSFT_T(KC_CAPSLOCK),KC_TRNS,KC_TRNS,KC_LBRC,KC_RBRC,KC_TRNS,KC_TRNS,  KC_TRNS,KC_TRNS,  KC_P1,  KC_P2,   KC_P3,   KC_TRNS, RSFT_T(KC_CAPSLOCK),
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
    KC_TRNS,LALT(KC_BSLASH),LALT(KC_PIPE),LM(_RED,MOD_MEH), KC_TRNS,   KC_TRNS, KC_TRNS, KC_TRNS,KC_P0,   KC_BSPC
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
   [_GRN] = LAYOUT(
  //,------------------------------------------------.                         ,---------------------------------------------------.
  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,KC_TRNS, KC_TRNS,                          KC_TRNS, KC_HOME,KC_PGUP, KC_END, KC_TRNS, KC_EQUAL,
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
LALT(KC_DEL),KC_TRNS,KC_TRNS,KC_TRNS, KC_TRNS, KC_TRNS,                        KC_TRNS,LCTL(KC_LEFT),KC_UP,LCTL(KC_RGHT),KC_TRNS,LGUI(LSFT(KC_BSLASH)),
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
LALT(KC_BSPC),KC_TRNS,KC_TRNS,KC_TRNS,KC_TRNS, KC_TRNS,                   LALT(KC_LEFT),KC_LEFT,KC_DOWN, KC_RGHT,LALT(KC_RGHT),KC_TRNS,
 //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
KC_LSFT,  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,LGUI(LSFT(KC_V)),KC_TRNS,LALT(KC_BSPC),KC_BSPACE,KC_PGDOWN,KC_DELETE,LALT(KC_DEL),KC_RSFT,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
        LGUI(LSFT(KC_V)),RESET,LALT(KC_DEL),KC_TRNS,KC_TRNS,        KC_TRNS,LM(_RED,MOD_MEH),KC_TRNS,KC_TRNS,KC_TRNS

  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
      [_RED] = LAYOUT(
  //,------------------------------------------------.                         ,---------------------------------------------------.
  KC_GRAVE, KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                           KC_6,    KC_7,   KC_8,    KC_9,    KC_0,    KC_MINUS,
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
  KC_TAB,   KC_Q,   KC_W,    KC_F,    KC_P,    KC_B,                           KC_J,    KC_L,   KC_U,    KC_Y,    KC_SCLN, KC_BSLASH,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_BSPC,  KC_A,   KC_R,    KC_S,    KC_T,    KC_G,                           KC_M,    KC_N,   KC_E,    KC_I,    KC_O,    KC_QUOTE,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
  KC_LSPO,  KC_Z,   KC_X, TD(DANCE_1),KC_D,    KC_V, LGUI(KC_Z),       KC_MUTE,KC_K,    KC_H,   KC_COMM, KC_DOT,  KC_SLSH, KC_RSPC,
  //|------+-------+--------+--------+--------+------|  ===  |        |  ===  |--------+-------+--------+--------+--------+---------|
   OSM(MOD_LCTL),KC_DEL,TD(DANCE_2), LT(_GRN,KC_SPC),LGUI_T(KC_F12),RGUI_T(KC_ENTER),LT(_BLU,KC_SPACE),TD(DANCE_3),KC_ESCAPE,OSM(MOD_RALT)
  //            \--------+--------+--------+---------+-------|        |--------+---------+--------+---------+-------/
),
    // [_GRN] = LAYOUT_ortho_5x12(
    //    KC_TRNS,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           KC_HOME,          KC_PGUP,        KC_END,         KC_TRNS,             KC_EQUAL,
    //    LALT(KC_DEL),          KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           LCTL(KC_LEFT),    KC_UP,          LCTL(KC_RGHT),  KC_TRNS,             LGUI(LSFT(KC_BSLASH)),
    //    LALT(KC_BSPC),         KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_LEFT),     KC_LEFT,          KC_DOWN,        KC_RGHT,        LALT(KC_RGHT),       KC_TRNS,
    //    KC_LSFT,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_BSPC),     KC_BSPACE,        KC_PGDOWN,      KC_DELETE,      LALT(KC_DEL),        KC_RSFT,
    //    LGUI(LSFT(KC_V)),      KC_TRNS,             RESET,         LALT(KC_DEL),    KC_TRNS,       KC_TRNS,           LM(_RED,MOD_MEH),  KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_TRNS),
    // [_RED] = LAYOUT_ortho_5x12(
    //    KC_TRNS,               KC_F1,               KC_F2,         KC_F3,           KC_F4,         KC_F5,             KC_F6,             KC_F7,            KC_F8,          KC_F9,          KC_F10,              KC_F16,
    //    KC_TRNS,               KC_TRNS,             KC_W,          KC_F,            KC_P,          KC_TRNS,           KC_TRNS,           KC_L,             KC_U,           KC_Y,           KC_TRNS,             TO(_GRN),
    //    KC_TRNS,               KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                TO(_BLU),
    //    KC_LEFT,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_LGUI,           KC_LGUI,           KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_RGHT, 
    //    KC_F17,                LGUI_T(KC_F12),      KC_TRNS,       KC_F13,          KC_UP,         KC_TRNS,           KC_TRNS,           KC_DOWN,          KC_ESCAPE,      KC_TRNS,        RGUI_T(KC_F12),      RGUI_T(KC_ENTER)),
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

void on_dance_1(qk_tap_dance_state_t *state, void *user_data);
uint8_t dance_1_dance_step(qk_tap_dance_state_t *state);
void dance_1_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_1_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_1(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

uint8_t dance_1_dance_step(qk_tap_dance_state_t *state) {
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

void dance_1_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_1_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case SINGLE_HOLD: register_code16(LALT(KC_C)); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C); break;
    }
}

void dance_1_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_C)); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state.step = 0;
}

void on_dance_2(qk_tap_dance_state_t *state, void *user_data);
uint8_t dance_2_dance_step(qk_tap_dance_state_t *state);
void dance_2_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_2_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_2(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(KC_E));
        tap_code16(LALT(KC_E));
        tap_code16(LALT(KC_E));
    }
    if(state->count > 3) {
        tap_code16(LALT(KC_E));
    }
}

uint8_t dance_2_dance_step(qk_tap_dance_state_t *state) {
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

void dance_2_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_2_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(LALT(KC_E)); break;
        case SINGLE_HOLD: register_code16(LALT(KC_I)); break;
        case DOUBLE_TAP: register_code16(LALT(KC_N)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(KC_N)); break;
    }
}

void dance_2_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_E)); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_I)); break;
        case DOUBLE_TAP: unregister_code16(LALT(KC_N)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(KC_N)); break;
    }
    dance_state.step = 0;
}

void on_dance_3(qk_tap_dance_state_t *state, void *user_data);
uint8_t dance_3_dance_step(qk_tap_dance_state_t *state);
void dance_3_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_3_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_3(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(KC_GRAVE));
        tap_code16(LALT(KC_GRAVE));
        tap_code16(LALT(KC_GRAVE));
    }
    if(state->count > 3) {
        tap_code16(LALT(KC_GRAVE));
    }
}

uint8_t dance_3_dance_step(qk_tap_dance_state_t *state) {
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

void dance_3_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_3_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(LALT(KC_GRAVE)); break;
        case SINGLE_HOLD: register_code16(LALT(KC_U)); break;
        case DOUBLE_TAP: register_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LSFT(KC_9))); break;
    }
}

void dance_3_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_GRAVE)); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_U)); break;
        case DOUBLE_TAP: unregister_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LSFT(KC_9))); break;
    }
    dance_state.step = 0;
}

qk_tap_dance_action_t tap_dance_actions[] = {
    [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
    [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
    [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
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

    //snprintf(layer_state_str, sizeof(layer_state_str), "Layer: Undef-%ld", layer_state)


    switch (get_highest_layer(default_layer_state)) {
        // case _QWERTY:
        //     oled_write_ln_P(PSTR("Qwrt"), false);
        //     break;
        // case _COLEMAK:
        //     oled_write_ln_P(PSTR("Clmk"), false);
        //     break;
        // case _COLEMAKDH:
        //     oled_write_ln_P(PSTR("CmkDH"), false);
        //     break;

        default:
            oled_write_ln_P(PSTR("CmkDH"), false);
    }
    oled_write_P(PSTR("\n\n"), false);
    // Print current layer
    oled_write_ln_P(PSTR("LAYER"), false);
    switch (get_highest_layer(layer_state)) {
        case _RED:
            oled_write_P(PSTR("Red"), false);
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


// #ifdef RGBLIGHT_ENABLE
// char layer_state_str[70];
// // Now define the array of layers. Later layers take precedence

// // QWERTY,
// // Light on inner column and underglow
// const rgblight_segment_t PROGMEM layer_qwerty_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//   SET_LAYER_ID(HSV_RED)

// );
// const rgblight_segment_t PROGMEM layer_colemakdh_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//   SET_LAYER_ID(HSV_PINK)
// );

// // _NUM,
// // Light on outer column and underglow
// const rgblight_segment_t PROGMEM layer_num_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//     SET_LAYER_ID(HSV_TEAL)

// );
// // _SYMBOL,
// // Light on inner column and underglow
// const rgblight_segment_t PROGMEM layer_symbol_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//     SET_LAYER_ID(HSV_BLUE)

//     );
// // _COMMAND,
// // Light on inner column and underglow
// const rgblight_segment_t PROGMEM layer_command_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//   SET_LAYER_ID(HSV_PURPLE)
// );

// //_NUMPAD
// const rgblight_segment_t PROGMEM layer_numpad_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//     SET_INDICATORS(HSV_ORANGE),
//     SET_UNDERGLOW(HSV_ORANGE),
//     SET_NUMPAD(HSV_BLUE),
//     {7, 4, HSV_ORANGE},
//     {25, 2, HSV_ORANGE},
//     {35+6, 4, HSV_ORANGE},
//     {35+25, 2, HSV_ORANGE}
//     );
// // _SWITCHER   // light up top row
// const rgblight_segment_t PROGMEM layer_switcher_lights[] = RGBLIGHT_LAYER_SEGMENTS(
//     SET_LAYER_ID(HSV_GREEN),
//     SET_NUMROW(HSV_GREEN)
// );

// const rgblight_segment_t* const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(

//     layer_qwerty_lights,
//     layer_num_lights,// overrides layer 1
//     layer_symbol_lights,
//     layer_command_lights,
//     layer_numpad_lights,
//     layer_switcher_lights,  // Overrides other layers
//     layer_colemakdh_lights
// );

// layer_state_t layer_state_set_user(layer_state_t state) {
//     rgblight_set_layer_state(0, layer_state_cmp(state, _DEFAULTS) && layer_state_cmp(default_layer_state,_QWERTY));
//     rgblight_set_layer_state(7, layer_state_cmp(state, _DEFAULTS) && layer_state_cmp(default_layer_state,_COLEMAKDH));


//     rgblight_set_layer_state(1, layer_state_cmp(state, _LOWER));
//     rgblight_set_layer_state(2, layer_state_cmp(state, _RAISE));
//     rgblight_set_layer_state(3, layer_state_cmp(state, _ADJUST));
//     rgblight_set_layer_state(4, layer_state_cmp(state, _NUMPAD));
//     rgblight_set_layer_state(5, layer_state_cmp(state, _SWITCH));
//     return state;
// }
// void keyboard_post_init_user(void) {
//     // Enable the LED layers
//     rgblight_layers = my_rgb_layers;

//     rgblight_mode(10);// haven't found a way to set this in a more useful way

// }
// #endif
