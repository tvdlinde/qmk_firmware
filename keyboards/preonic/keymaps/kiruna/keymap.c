#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)

enum layers {
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

bool encoder_update_user(uint8_t index, bool clockwise) {
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
        tap_code16(KC_RGHT);
    } else {
        tap_code16(KC_LEFT);
    }  
    break;
}
return false;
};

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
    [_MAIN] = LAYOUT_ortho_5x12(  
       KC_GRAVE,              KC_1,                KC_2,          KC_3,            KC_4,          KC_5,              KC_6,              KC_7,             KC_8,           KC_9,           KC_0,                KC_MINUS,  
       KC_TAB,                KC_Q,                KC_W,          KC_F,            KC_P,          KC_B,              KC_J,              KC_L,             KC_U,           KC_Y,           KC_SCOLON,           KC_BSLASH, 
       KC_BSPC,               KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                KC_QUOTE,
       KC_LSPO,               KC_Z,                KC_X,          TD(DANCE_1),     KC_D,          KC_V,              KC_K,              KC_H,             KC_COMMA,       KC_DOT,         KC_SLASH,            KC_RSPC,
       LGUI(KC_Z),            LGUI_T(KC_F12),      OSM(MOD_LCTL), KC_DEL,          TD(DANCE_2),   LT(_GRN,KC_SPACE), LT(_BLU,KC_SPACE), TD(DANCE_3),      KC_ESCAPE,      OSM(MOD_RALT),  RGUI_T(KC_F12),      RSFT_T(KC_ENTER)),
    [_BLU] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_F1,               KC_F2,         KC_F3,           KC_F4,         KC_F5,             KC_F6,             KC_F7,            KC_F8,          KC_F9,          KC_F10,              KC_EQUAL,
       LALT(KC_DEL),          KC_TRNS,             KC_TRNS,       KC_LCBR,         KC_RCBR,       KC_TRNS,           KC_GRAVE,          KC_P7,            KC_P8,          KC_P9,          KC_TRNS,             KC_TRNS,
       LALT(KC_BSPC),         KC_TRNS,             KC_MINUS,      KC_LPRN,         KC_RPRN,       KC_DLR,            KC_MINUS,          KC_P4,            KC_P5,          KC_P6,          KC_KP_PLUS,          KC_TRNS,
       LSFT_T(KC_CAPSLOCK),   KC_TRNS,             KC_TRNS,       KC_LBRC,         KC_RBRC,       KC_TRNS,           KC_TRNS,           KC_P1,            KC_P2,          KC_P3,          KC_TRNS,             RSFT_T(KC_CAPSLOCK),
       KC_TRNS,               KC_TRNS,             KC_TRNS,       LALT(KC_BSLASH), LALT(KC_PIPE), LM(_RED,MOD_MEH),  KC_TRNS,           KC_TRNS,          KC_P0,          KC_BSPC,        KC_PDOT,             KC_TRNS),
    [_GRN] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           KC_HOME,          KC_PGUP,        KC_END,         KC_TRNS,             KC_EQUAL,
       LALT(KC_DEL),          KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           LCTL(KC_LEFT),    KC_UP,          LCTL(KC_RGHT),  KC_TRNS,             LGUI(LSFT(KC_BSLASH)),
       LALT(KC_BSPC),         KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_LEFT),     KC_LEFT,          KC_DOWN,        KC_RGHT,        LALT(KC_RGHT),       KC_TRNS,
       KC_LSFT,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_BSPC),     KC_BSPACE,        KC_PGDOWN,      KC_DELETE,      LALT(KC_DEL),        KC_RSFT,
       LGUI(LSFT(KC_V)),      KC_TRNS,             RESET,         LALT(KC_DEL),    KC_TRNS,       KC_TRNS,           LM(_RED,MOD_MEH),  KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_TRNS),
    [_RED] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_F1,               KC_F2,         KC_F3,           KC_F4,         KC_F5,             KC_F6,             KC_F7,            KC_F8,          KC_F9,          KC_F10,              KC_F16,
       KC_TRNS,               KC_TRNS,             KC_W,          KC_F,            KC_P,          KC_TRNS,           KC_TRNS,           KC_L,             KC_U,           KC_Y,           KC_TRNS,             TO(_GRN),
       KC_TRNS,               KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                TO(_BLU),
       KC_LEFT,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_LGUI,           KC_LGUI,           KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_RGHT, 
       KC_F17,                LGUI_T(KC_F12),      KC_TRNS,       KC_F13,          KC_UP,         KC_TRNS,           KC_TRNS,           KC_DOWN,          KC_ESCAPE,      KC_TRNS,        RGUI_T(KC_F12),      RGUI_T(KC_ENTER)),
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
