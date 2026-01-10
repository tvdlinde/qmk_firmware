#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)

enum layers {
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

bool encoder_update_user(uint8_t index, bool clockwise) {
 switch(biton32(layer_state)){
    case _BLU:
    if (clockwise) {
        tap_code16(LCTL(LALT(LSFT(KC_MINUS))));
    } else {
        tap_code16(LCTL(LALT((KC_MINUS))));
    }
    break;
    case _RED:
    if (clockwise) {
        tap_code16(LWIN(KC_TAB));
    } else {
        tap_code16(LWIN(LSFT(KC_TAB)));
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
        tap_code(KC_VOLU);
    } else {
        tap_code(KC_VOLD);
    }
    break;
}
return false;
};

layer_state_t layer_state_set_user(layer_state_t state) {
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

// ------------------------------------------------------------------
// 1️⃣  Declare a custom keycode (must be after SAFE_RANGE)
enum custom_keycodes {
    CTL_BSPC_TAP = SAFE_RANGE   // our special key
};

// ------------------------------------------------------------------
// 2️⃣  Optional: give this key a shorter tapping term if you like
uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case CTL_BSPC_TAP: return 180;   // ms – adjust to your taste
        default:            return TAPPING_TERM;
    }
}

// ------------------------------------------------------------------
// 3️⃣  Core logic – run on every press/release of the custom key
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    static uint16_t press_timestamp;   // remembers when we pressed it

    switch (keycode) {
        case CTL_BSPC_TAP:
            if (record->event.pressed) {
                // ----- key pressed -------------------------------------------------
                press_timestamp = timer_read();           // remember the moment
                register_mods(MOD_BIT(KC_LCTL));         // start holding Left‑Ctrl
            } else {
                // ----- key released ------------------------------------------------
                uint16_t held_time = timer_elapsed(press_timestamp);

                // Always stop the Ctrl modifier we started
                unregister_mods(MOD_BIT(KC_LCTL));

                // If the press was short enough → treat it as a tap
                if (held_time < get_tapping_term(keycode, record)) {
                    // Send Ctrl+Backspace as a single atomic action
                    tap_code16(LCTL(KC_BSPC));
                }
                // If it was a long hold we already left Ctrl active,
                // so nothing else to do.
            }
            // We handled the key completely – stop further processing
            return false;
    }

    // All other keys fall through to the default handler
    return true;
}

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_MAIN] = LAYOUT_ortho_5x12(
       KC_GRAVE,              KC_1,                KC_2,          KC_3,            KC_4,          KC_5,              KC_6,              KC_7,             KC_8,           KC_9,           KC_0,                KC_MINUS,
       KC_TAB,                KC_Q,                KC_W,          KC_F,            KC_P,          KC_B,              KC_J,              KC_L,             KC_U,           KC_Y,           KC_SCOLON,           KC_BSLASH,
      CTL_BSPC_TAP,              KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                KC_QUOTE,
       KC_LSPO,               KC_Z,                KC_X,          TD(TD_1),        KC_D,          KC_V,              KC_K,              KC_H,             KC_COMMA,       KC_DOT,         KC_SLASH,            KC_RSPC,
       LCTL(KC_Z),            LCTL_T(KC_F12),      KC_LGUI,       TD(TD_5),        TD(TD_2),      LT(_GRN,KC_SPACE), LT(_BLU,KC_SPACE), TD(TD_3),         KC_ESCAPE,      KC_LALT,        RCTL_T(KC_F12),      RSFT_T(KC_ENTER)),
    [_BLU] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_F1,               KC_F2,         KC_F3,           KC_F4,         KC_F5,             KC_F6,             KC_F7,            KC_F8,          KC_F9,          KC_F10,              KC_EQUAL,
       LCTL(KC_DEL),          KC_TRNS,             KC_TRNS,       KC_LCBR,         KC_RCBR,       KC_TRNS,           KC_GRAVE,          KC_P7,            KC_P8,          KC_P9,          KC_TRNS,             KC_TRNS,
       KC_CAPS,         KC_TRNS,             KC_MINUS,      KC_LPRN,         KC_RPRN,       KC_DLR,            KC_MINUS,          KC_P4,            KC_P5,          KC_P6,          KC_KP_PLUS,          KC_TRNS,
       KC_LBRC,               KC_TRNS,             KC_TRNS,       KC_LBRC,         KC_RBRC,       KC_TRNS,           KC_TRNS,           KC_P1,            KC_P2,          KC_P3,          KC_TRNS,             KC_RBRC,
       LCTL(KC_Z),      KC_TRNS,             KC_TRNS,       RALT(KC_LBRC), RALT(KC_RBRC), LM(_RED,MOD_LGUI),  KC_TRNS,           KC_TRNS,          KC_P0,          KC_BSPC,        KC_PDOT,             KC_TRNS),
    [_GRN] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           LALT(KC_LEFT),          KC_PGUP,        LALT(KC_RGHT),         LALT(KC_A),             KC_EQUAL,
       LCTL(KC_DEL),          KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           KC_TRNS,           KC_HOME,    KC_UP,          KC_END,  KC_TRNS,             LCTL(LSFT(KC_BSLASH)),
       KC_BSPACE,         KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LCTL(KC_LEFT),     KC_LEFT,          KC_DOWN,        KC_RGHT,        LCTL(KC_RGHT),       KC_TRNS,
       KC_LCPO,               KC_TRNS,             KC_TRNS,       KC_TRNS,         KC_TRNS,       KC_TRNS,           LCTL(KC_BSPC),     KC_BSPACE,        KC_PGDOWN,      KC_DELETE,      LCTL(KC_DEL),        KC_RCPC,
       LCTL(KC_K),             RESET,       LGUI(KC_INS),  LCTL(KC_U),    KC_TRNS,       KC_TRNS,           LM(_RED,MOD_LGUI),  KC_TRNS,          LCTL(KC_L),        KC_TRNS,        KC_TRNS,             KC_TRNS),
    [_RED] = LAYOUT_ortho_5x12(
       KC_TRNS,               KC_1,               KC_2,         KC_3,           KC_4,         KC_5,             KC_6,             KC_7,            KC_8,          KC_9,          KC_0,              KC_TRNS,
       KC_TRNS,               KC_Q,             KC_W,          KC_F,            KC_P,          KC_B,           KC_J,           KC_L,             KC_U,           KC_Y,           KC_TRNS,             TO(_GRN),
       KC_TRNS,               KC_A,                KC_R,          KC_S,            KC_T,          KC_G,              KC_M,              KC_N,             KC_E,           KC_I,           KC_O,                TO(_BLU),
       KC_LSFT,               KC_Z,             KC_X,       KC_C,         KC_D,       KC_V,           KC_K,           KC_H,          KC_TRNS,        KC_TRNS,        KC_TRNS,             KC_RSFT,
       LSFT(KC_B),                LSFT_T(KC_F12),      KC_INS,        KC_DEL,         KC_UP,         KC_TRNS,           KC_TRNS,           KC_DOWN,          KC_ESCAPE,      KC_TRNS,        RSFT_T(KC_F12),      RSFT_T(KC_ENTER)),
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

qk_tap_dance_action_t tap_dance_actions[] = {
    [TD_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
    [TD_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
    [TD_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
    [TD_4] = ACTION_TAP_DANCE_DOUBLE(KC_BSPC, LALT(KC_BSPC)),
    [TD_5] = ACTION_TAP_DANCE_DOUBLE(KC_DEL, LCTL(KC_DEL))
};
