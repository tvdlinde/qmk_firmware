#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)

enum layers {
   _MAIN,
   _BLU,
   _GRN,
   _MEH
};

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
};

bool encoder_update_user(uint8_t index, bool clockwise) {
 switch(biton32(layer_state)){
    case _BLU:
    if (clockwise) {
        tap_code16(LCTL(LSFT(KC_RGHT)));
    } else {
        tap_code16(LCTL(LSFT(KC_LEFT)));
    }
    break;
    case _MEH:
    if (clockwise) {
        tap_code16(MEH(KC_G));
    } else {
        tap_code16(MEH(KC_A));
    }
    break;            
    case _GRN:
    if (clockwise) {
        tap_code16(LCTL(LSFT(KC_MINUS)));
    } else {
        tap_code16(LCTL(KC_MINUS));
    }
    break;
    default:
    if (clockwise) {
        tap_code16(KC_DEL);
    } else {
        tap_code16(LGUI(KC_Z));
    }  
    break;
}
return true;
};

uint32_t layer_state_set_user(uint32_t state) {
#ifdef RGBLIGHT_ENABLE
    switch (biton32(state)) {
        case _BLU:
        rgblight_setrgb(RGB_BLUE);
        break;
        case _MEH:
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
       KC_GRAVE,            KC_1,                KC_2,          KC_3,            KC_4,          KC_5,              KC_6,              KC_7,             KC_8,           KC_9,           KC_0,                KC_MINUS,  
       LALT(KC_DEL),        KC_Q,                KC_W,          KC_F,            KC_P,          KC_B,              KC_J,              KC_L,             KC_U,           KC_Y,           KC_SCOLON,           KC_BSLASH, 
       LALT(KC_BSPC),       KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                KC_QUOTE,
       LSFT_T(KC_CAPSLOCK), KC_Z,                KC_X,          TD(DANCE_1),     KC_D,          KC_V,              KC_K,              KC_H,             KC_COMMA,       KC_DOT,         KC_SLASH,            LSFT_T(KC_CAPSLOCK),
       TD(DANCE_0),         LGUI_T(KC_F12),      OSM(MOD_LCTL), KC_EQUAL,        TD(DANCE_2),   LT(_GRN,KC_SPACE), LT(_BLU,KC_SPACE), TD(DANCE_3),      KC_ESCAPE,      OSM(MOD_RALT),  RGUI_T(KC_F12),      KC_ENTER),
    [_BLU] = LAYOUT_ortho_5x12(
       KC_TRNS,             KC_F1,               KC_F2,         KC_F3,           KC_F4,         KC_F5,             KC_F6,             KC_F7,            KC_F8,          KC_F9,          KC_F10,              KC_TRNS,
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_LCBR,         KC_RCBR,       KC_TRNS,           KC_GRAVE,          KC_P7,            KC_P8,          KC_P9,          KC_BSLASH,           KC_TRNS,
       KC_TRNS,             KC_TRNS,             KC_MINUS,      KC_LPRN,         KC_RPRN,       KC_DLR,            KC_MINUS,          KC_P4,            KC_P5,          KC_P6,          KC_QUOTE,            KC_TRNS,
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_LBRC,         KC_RBRC,       KC_TRNS,           KC_TRNS,           KC_P1,            KC_P2,          KC_P3,          KC_TRNS,             KC_TRNS,
       LALT(LSFT(KC_9)),    KC_TRNS,             KC_TRNS,       LALT(KC_BSLASH), LALT(KC_PIPE), LT(_MEH,KC_SPACE), KC_TRNS,           KC_TRNS,          KC_P0,          KC_BSPC,        KC_PDOT,             KC_TRNS),
    [_GRN] = LAYOUT_ortho_5x12(
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           KC_TRNS,          KC_HOME,        KC_PGUP,        KC_END,              LALT(LSFT(KC_9)),
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           LCTL(KC_LEFT),    KC_UP,          LCTL(KC_RGHT),  KC_TRNS,             KC_TRNS,
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_LEFT),     KC_LEFT,          KC_DOWN,        KC_RGHT,        LALT(KC_RGHT),       KC_TRNS,
       KC_TRNS,             KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LALT(KC_BSPC),     KC_BSPACE,        KC_PGDOWN,      KC_DELETE,      LALT(KC_DEL),        KC_TRNS,
       RESET,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LT(_MEH,KC_SPACE), KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_TRNS),
    [_MEH] = LAYOUT_ortho_5x12(
       KC_TRNS,             MEH(KC_F1),          MEH(KC_F2),    MEH(KC_F3),      MEH(KC_F4),    MEH(KC_F5),        MEH(KC_F6),        MEH(KC_F7),       MEH(KC_F8),     MEH(KC_F9),     MEH(KC_F10),         MEH(KC_F16),
       MEH(KC_DEL),         KC_TRNS,             MEH(KC_W),     MEH(KC_F),       MEH(KC_P),     KC_TRNS,           KC_TRNS,           MEH(KC_L),        MEH(KC_U),      MEH(KC_Y),      KC_TRNS,             KC_TRNS,
       KC_TRNS,             MEH(KC_A),           MEH(KC_R),     MEH(KC_S),       LCA(KC_T),     MEH(KC_G),         MEH(KC_M),         MEH(KC_N),        MEH(KC_E),      LCA(KC_I),      MEH(KC_O),           KC_TRNS,
       MEH(KC_LEFT),        KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_LGUI,           KC_LGUI,           KC_TRNS,          KC_TRNS,        KC_TRNS,        KC_TRNS,             MEH(KC_RGHT), 
       MEH(KC_TAB),         LGUI_T(MEH(KC_F12)), KC_TRNS,       MEH(KC_F13),     MEH(KC_UP),    KC_TRNS,           KC_TRNS,           MEH(KC_DOWN),     MEH(KC_ESCAPE), KC_TRNS,        LGUI_T(MEH(KC_F12)), MEH(KC_ENTER)),
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

void on_dance_0(qk_tap_dance_state_t *state, void *user_data);
uint8_t dance_0_dance_step(qk_tap_dance_state_t *state);
void dance_0_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_0_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_0(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
};

uint8_t dance_0_dance_step(qk_tap_dance_state_t *state) {
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

void dance_0_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_0_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case SINGLE_HOLD: register_code16(LALT(KC_N)); break;
        case DOUBLE_TAP: register_code16(KC_TAB); register_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_0_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_N)); break;
        case DOUBLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state.step = 0;
}

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
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C);
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
        case DOUBLE_TAP: register_code16(LALT(KC_E)); register_code16(LALT(KC_E)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(KC_E)); register_code16(LALT(KC_E));
    }
}

void dance_2_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_E)); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_I)); break;
        case DOUBLE_TAP: unregister_code16(LALT(KC_E)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(KC_E)); break;
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
        case DOUBLE_TAP: register_code16(LALT(KC_GRAVE)); register_code16(LALT(KC_GRAVE)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(KC_GRAVE)); register_code16(LALT(KC_GRAVE));
    }
}

void dance_3_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_GRAVE)); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_U)); break;
        case DOUBLE_TAP: unregister_code16(LALT(KC_GRAVE)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(KC_GRAVE)); break;
    }
    dance_state.step = 0;
}

void on_dance_4(qk_tap_dance_state_t *state, void *user_data);
uint8_t dance_4_dance_step(qk_tap_dance_state_t *state);
void dance_4_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_4_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_4(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_BSLASH);
        tap_code16(KC_BSLASH);
        tap_code16(KC_BSLASH);
    }
    if(state->count > 3) {
        tap_code16(KC_BSLASH);
    }
}

uint8_t dance_4_dance_step(qk_tap_dance_state_t *state) {
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

void dance_4_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state.step = dance_4_dance_step(state);
    switch (dance_state.step) {
        case SINGLE_TAP: register_code16(KC_BSLASH); break;
        case SINGLE_HOLD: register_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_TAP: register_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_BSLASH); register_code16(KC_BSLASH);
    }
}

void dance_4_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(KC_BSLASH); break;
        case SINGLE_HOLD: unregister_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LSFT(KC_9))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_BSLASH); break;
    }
    dance_state.step = 0;
}

qk_tap_dance_action_t tap_dance_actions[] = {
    [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
    [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
    [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
    [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
    [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
};
