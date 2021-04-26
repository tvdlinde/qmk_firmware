 #include QMK_KEYBOARD_H

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)
#define MOON_LED_LEVEL LED_LEVEL

enum layers {
     _MAIN,
     _RSE,
     _LWR,
};


void encoder_update_user(uint8_t index, bool clockwise) {
   switch(biton32(layer_state)){
             case _RSE:
                if (clockwise) {
                tap_code(KC_DELETE);
                 } else {
                tap_code(KC_BSPACE);
                }  
                break;            
            case _LWR:
                if (clockwise) {
                tap_code(KC_DOWN);
                } else {
                tap_code(KC_UP);
                }
                break;
            default:
                if (clockwise) {
                tap_code(KC_RIGHT);
                } else {
                tap_code(KC_LEFT);
                }
                break;
      }
  }

float song_one[][2] = SONG(TERMINAL_SOUND);
float song_two[][2] = SONG(CAPS_LOCK_OFF_SOUND);
float song_base[][2] = SONG(NUM_LOCK_ON_SOUND);

uint32_t layer_state_set_user(uint32_t state) {
#ifdef RGBLIGHT_ENABLE
//#ifdef AUDIO_ENABLE
    switch (biton32(state)) {
    case _RSE:
      rgblight_setrgb(RGB_RED);
      //PLAY_SONG(song_one);
      break;
    case _LWR:
      rgblight_setrgb(RGB_GREEN);
      //PLAY_SONG(song_two);
      break;
    default: //  for any other layers, or the default layer
      rgblight_setrgb(RGB_OFF);
      //PLAY_SONG(song_base);
      break;}
#endif
//#endif
  return state;
}

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
	[_MAIN] = LAYOUT_ortho_5x12(
		KC_1,           KC_2,         KC_3,            KC_4,         KC_5,     KC_ESCAPE,       TD(DANCE_0),  KC_6,           KC_7,                KC_8,         KC_9,          KC_0,  
		KC_Q,           KC_W,         KC_F,            KC_P,         KC_B,     KC_EQUAL,        KC_MINUS,     KC_J,           KC_L,                KC_U,         KC_Y,          KC_SCOLON, 
		LT(1,KC_A),     LT(2,KC_R),   LGUI_T(KC_S),    LSFT_T(KC_T), KC_G,     KC_BSLASH,       KC_QUOTE,     KC_M,           RSFT_T(KC_N),        RGUI_T(KC_E), KC_I,          LT(1,KC_O),
		KC_Z,           KC_X,         TD(DANCE_1),     KC_D,         KC_V,     TG(2),           TG(1),        KC_K,           KC_H,                KC_COMMA,     KC_DOT,        KC_SLASH, 
		OSM(MOD_LALT),  TD(DANCE_4),  KC_LBRACKET,     TD(DANCE_2),  KC_SPACE, LT(2,KC_DELETE), LT(1,KC_ENT), LALT(KC_BSPC),  TD(DANCE_3),         KC_RBRACKET,  OSM(MOD_LCTL), OSM(MOD_LGUI)),
	[_RSE] = LAYOUT_ortho_5x12(
		KC_EXLM,        KC_AT,        KC_HASH,         KC_DLR,       KC_PERC,  KC_TAB,          KC_TRNS,      KC_CIRC,        KC_AMPR,             KC_ASTR,      KC_LPRN,       KC_RPRN, 
		KC_TRNS,        KC_TRNS,      KC_LPRN,         KC_RPRN,      KC_TRNS,  KC_PLUS,         KC_UNDS,      KC_GRAVE,       KC_7,                KC_8,         KC_9,          KC_SCLN, 
		KC_TRNS,        KC_BSLS,      KC_LCBR,         KC_RCBR,      KC_DLR,   KC_LCBR,         KC_RCBR,      KC_DLR,         KC_4,                KC_5,         KC_6,          KC_TRNS, 
		KC_TRNS,        KC_TRNS,      KC_LBRC,         KC_RBRC,      KC_TRNS,  TO(0),           KC_TRNS,      KC_TRNS,        KC_1,                KC_2,         KC_3,          KC_TRNS, 
		KC_TRNS,        KC_TRNS,    LALT(KC_BSLASH),   KC_TRNS,      KC_TRNS,  KC_DELETE,       KC_TRNS,      KC_BSPC,        KC_0,      LALT(LSFT(KC_BSLASH)),  KC_TRNS,       KC_TRNS),
	[_LWR] = LAYOUT_ortho_5x12(
		KC_TRNS,        KC_TRNS,      KC_TRNS,         KC_TRNS,      KC_TRNS,  KC_TAB,          KC_TRNS,      KC_TRNS,        KC_TRNS,             KC_PGUP,      KC_TRNS,       KC_TRNS,
		KC_TRNS,        KC_TRNS,      KC_TRNS,         KC_TRNS,      KC_TRNS,  KC_TRNS,         KC_TRNS,      KC_HOME,        LALT(KC_LEFT),       KC_UP,        LALT(KC_RGHT), KC_END, 
		KC_TRNS,        KC_TRNS,      KC_TRNS,         KC_TRNS,      KC_TRNS,  KC_TRNS,         KC_TRNS,      KC_DLR,         KC_LEFT,             KC_DOWN,      KC_RGHT,       KC_DLR, 
		KC_TRNS,        KC_TRNS,      KC_TRNS,         KC_TRNS,      KC_TRNS,  KC_TRNS,         TO(0),        KC_TRNS,        KC_BSPACE,           KC_PGDOWN,    KC_DELETE,     KC_TRNS, 
		KC_TRNS,        KC_TRNS,      KC_TRNS,         KC_TRNS,      KC_TRNS,  KC_TRNS,         KC_ENT,       KC_BSPC,        KC_TRNS,             KC_TRNS,      KC_TRNS,       RESET),
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
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
    }
    if(state->count > 3) {
        tap_code16(KC_GRAVE);
    }
}

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
        case SINGLE_TAP: register_code16(KC_GRAVE); break;
        case SINGLE_HOLD: register_code16(LALT(KC_N)); break;
        case DOUBLE_TAP: register_code16(KC_GRAVE); register_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_GRAVE); register_code16(KC_GRAVE);
    }
}

void dance_0_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(KC_GRAVE); break;
        case SINGLE_HOLD: unregister_code16(LALT(KC_N)); break;
        case DOUBLE_TAP: unregister_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_GRAVE); break;
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
        tap_code16(LGUI(KC_T));
        tap_code16(LGUI(KC_T));
        tap_code16(LGUI(KC_T));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_T));
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
        case SINGLE_TAP: register_code16(LGUI(KC_T)); break;
        case SINGLE_HOLD: register_code16(LALT(LCTL(LGUI(LSFT(KC_T))))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(LSFT(KC_T))))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_T)); register_code16(LGUI(KC_T));
    }
}

void dance_4_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state.step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_T)); break;
        case SINGLE_HOLD: unregister_code16(LALT(LCTL(LGUI(LSFT(KC_T))))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(LSFT(KC_T))))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_T)); break;
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

