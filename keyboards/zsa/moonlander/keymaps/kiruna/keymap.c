#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
};

#define DUAL_FUNC_0 LT(8, KC_8)
#define DUAL_FUNC_1 LT(3, KC_F)
#define DUAL_FUNC_2 LT(1, KC_F20)
#define DUAL_FUNC_3 LT(11, KC_7)
#define DUAL_FUNC_4 LT(7, KC_F10)
#define DUAL_FUNC_5 LT(6, KC_1)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_moonlander(
    KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           KC_LBRC,                                        KC_RBRC,        KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_TAB,         KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,           KC_EQUAL,                                       KC_MINUS,       KC_J,           KC_L,           KC_U,           KC_Y,           KC_SCLN,        KC_BSLS,        
    DUAL_FUNC_0,    KC_A,           KC_R,           KC_S,           KC_T,           KC_G,           LCTL(KC_DELETE),                                                                KC_ESCAPE,      KC_M,           KC_N,           KC_E,           KC_I,           KC_O,           MT(MOD_RGUI, KC_QUOTE),
    SC_LSPO,        KC_Z,           KC_X,           TD(DANCE_0),    KC_D,           KC_V,                                           KC_K,           KC_H,           KC_COMMA,       KC_DOT,         KC_SLASH,       SC_RSPC,        
    LGUI(KC_INSERT),MT(MOD_LCTL, KC_F12),KC_LEFT_GUI,    MT(MOD_LGUI, KC_DELETE),TD(DANCE_1),    LM(3,MOD_LGUI),                                                                                                 LM(3,MOD_LGUI), TD(DANCE_2),    MT(MOD_LALT, KC_ESCAPE),MT(MOD_LALT, KC_ESCAPE),MT(MOD_LCTL, KC_F12),DUAL_FUNC_1,    
    LT(2, KC_SPACE),MT(MOD_LCTL, KC_F12),LCTL(LSFT(KC_Z)),                LCTL(KC_Z),     MT(MOD_LCTL, KC_ENTER),LT(1, KC_SPACE)
  ),
  [1] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          RALT(KC_LBRC),                                  RALT(KC_RBRC),  KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_EQUAL,       
    LCTL(KC_DELETE),KC_TRANSPARENT, KC_TRANSPARENT, KC_LCBR,        KC_RCBR,        KC_TRANSPARENT, KC_KP_PLUS,                                     KC_UNDS,        KC_GRAVE,       KC_KP_7,        KC_KP_8,        KC_KP_9,        KC_COLN,        KC_PIPE,        
    KC_CAPS,        KC_TRANSPARENT, KC_MINUS,       KC_LPRN,        KC_RPRN,        KC_DLR,         KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_KP_MINUS,    KC_KP_4,        KC_KP_5,        KC_KP_6,        KC_KP_PLUS,     KC_DQUO,        
    MT(MOD_LSFT, KC_LBRC),KC_TRANSPARENT, KC_TRANSPARENT, KC_LBRC,        KC_RBRC,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_KP_1,        KC_KP_2,        KC_KP_3,        KC_KP_SLASH,    MT(MOD_RSFT, KC_RBRC),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RALT(KC_LBRC),  RALT(KC_RBRC),  TO(0),                                                                                                          TO(0),          KC_KP_0,        KC_KP_0,        KC_KP_DOT,      KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TOGGLE_LAYER_COLOR,                                RM_TOGG,        KC_TRANSPARENT, LALT(KC_LEFT),  KC_PAGE_UP,     LALT(KC_RIGHT), LALT(KC_A),     KC_EQUAL,       
    LCTL(KC_DELETE),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RM_VALU,                                        RM_VALD,        KC_TRANSPARENT, KC_HOME,        KC_UP,          KC_END,         KC_TRANSPARENT, LCTL(LSFT(KC_BSLS)),
    KC_BSPC,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                                                 QK_BOOT,        LCTL(KC_LEFT),  KC_LEFT,        KC_DOWN,        KC_RIGHT,       LCTL(KC_RIGHT), KC_TRANSPARENT, 
    DUAL_FUNC_2,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 LCTL(KC_BSPC),  KC_BSPC,        KC_PGDN,        KC_DELETE,      LCTL(KC_DELETE),DUAL_FUNC_3,    
    KC_LEFT_CTRL,   KC_TRANSPARENT, KC_TRANSPARENT, LCTL(KC_U),     MS_BTN1,     LCTL(KC_U),                                                                                                     TD(DANCE_3),    MS_BTN2,     LCTL(KC_L),     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, LCTL(LSFT(KC_MINUS)),                LCTL(KC_MINUS), KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_moonlander(
    LGUI(KC_GRAVE), LGUI(KC_1),     LGUI(KC_2),     LGUI(KC_3),     LGUI(KC_4),     LGUI(KC_5),     KC_TRANSPARENT,                                 KC_TRANSPARENT, LGUI(KC_6),     LGUI(KC_7),     LGUI(KC_8),     LGUI(KC_9),     LGUI(KC_0),     DUAL_FUNC_5,    
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LGUI(KC_DELETE),                                                                LGUI(KC_ESCAPE),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    LGUI(LSFT(KC_B)),DUAL_FUNC_4,    KC_TRANSPARENT, LGUI(KC_DELETE),KC_UP,          KC_TRANSPARENT,                                                                                                 KC_TRANSPARENT, KC_DOWN,        LGUI(KC_ESCAPE),KC_TRANSPARENT, DUAL_FUNC_4,    LGUI(KC_ENTER), 
    KC_SPACE,       KC_TRANSPARENT, LGUI(KC_TAB),                   LGUI(LSFT(KC_TAB)),KC_TRANSPARENT, KC_SPACE
  ),
};

const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] PROGMEM = LAYOUT(
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', '*', '*', 'R', 'R', 'R', 'R', 'R',
                 '*', '*', '*', '*', '*', '*'
);



bool capslock_active = false;

bool led_update_user(led_t led_state) {
  capslock_active = led_state.caps_lock;
  return true;
}

extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,211,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [1] = { {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {170,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {170,255,255}, {170,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {170,255,255}, {170,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {234,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255}, {170,255,255} },

    [2] = { {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {41,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255}, {0,245,245}, {85,255,255}, {85,255,255}, {85,255,255}, {85,255,255} },

    [3] = { {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (!keyboard_config.disable_layer_led) { 
    switch (biton32(layer_state)) {
      case 0:
        set_layer_color(0);
        break;
      case 1:
        set_layer_color(1);
        break;
      case 2:
        set_layer_color(2);
        break;
      case 3:
        set_layer_color(3);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  if (capslock_active && biton32(layer_state) == 1) {
    RGB rgb = hsv_to_rgb_with_value((HSV) { 0, 245, 245 });
    rgb_matrix_set_color( 2, rgb.r, rgb.g, rgb.b );
  } 
  return true;
}



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

static tap dance_state[4];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
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


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_HOLD: register_code16(RALT(KC_COMMA)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_HOLD: unregister_code16(RALT(KC_COMMA)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RALT(KC_QUOTE));
        tap_code16(RALT(KC_QUOTE));
        tap_code16(RALT(KC_QUOTE));
    }
    if(state->count > 3) {
        tap_code16(RALT(KC_QUOTE));
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(RALT(KC_QUOTE)); break;
        case SINGLE_HOLD: register_code16(RALT(KC_6)); break;
        case DOUBLE_TAP: register_code16(LSFT(RALT(KC_GRAVE))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RALT(KC_QUOTE)); register_code16(RALT(KC_QUOTE));
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(RALT(KC_QUOTE)); break;
        case SINGLE_HOLD: unregister_code16(RALT(KC_6)); break;
        case DOUBLE_TAP: unregister_code16(LSFT(RALT(KC_GRAVE))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RALT(KC_QUOTE)); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RALT(KC_GRAVE));
        tap_code16(RALT(KC_GRAVE));
        tap_code16(RALT(KC_GRAVE));
    }
    if(state->count > 3) {
        tap_code16(RALT(KC_GRAVE));
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(RALT(KC_GRAVE)); break;
        case SINGLE_HOLD: register_code16(LSFT(RALT(KC_QUOTE))); break;
        case DOUBLE_TAP: register_code16(RALT(KC_B)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RALT(KC_GRAVE)); register_code16(RALT(KC_GRAVE));
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(RALT(KC_GRAVE)); break;
        case SINGLE_HOLD: unregister_code16(LSFT(RALT(KC_QUOTE))); break;
        case DOUBLE_TAP: unregister_code16(RALT(KC_B)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RALT(KC_GRAVE)); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_D));
        tap_code16(LCTL(KC_D));
        tap_code16(LCTL(KC_D));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_D));
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_D)); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_K)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_D)); register_code16(LCTL(KC_D));
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_D)); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_K)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_D)); break;
    }
    dance_state[3].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
};

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LT(2, KC_SPACE):
        case LT(1, KC_SPACE):
            // Space is the highest-frequency key on the board (every word
            // boundary), so with fast/rolling typing the next key is often
            // pressed before Space is released. HOLD_ON_OTHER_KEY_PRESS
            // would resolve that as a layer hold instead of a tap, dropping
            // the space and reinterpreting the next keystroke on layer 1/2.
            // Fall back to the default (wait for tapping term / release) so
            // deliberate holds still reach the layer.
            return false;
        default:
            return true;
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
  case QK_MODS ... QK_MODS_MAX: 
    // Mouse keys with modifiers work inconsistently across operating systems, this makes sure that modifiers are always
    // applied to the mouse key that was pressed.
    if (IS_MOUSE_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode))) {
    if (record->event.pressed) {
        add_mods(QK_MODS_GET_MODS(keycode));
        send_keyboard_report();
        wait_ms(2);
        register_code(QK_MODS_GET_BASIC_KEYCODE(keycode));
        return false;
      } else {
        wait_ms(2);
        del_mods(QK_MODS_GET_MODS(keycode));
      }
    }
    break;

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(LCTL(KC_BSPC));
        } else {
          unregister_code16(LCTL(KC_BSPC));
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_GUI);
        } else {
          unregister_code16(KC_LEFT_GUI);
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(LGUI(KC_T));
        } else {
          unregister_code16(LGUI(KC_T));
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_CTRL);
        } else {
          unregister_code16(KC_LEFT_CTRL);
        }  
      }  
      return false;
    case DUAL_FUNC_2:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_LCBR);
        } else {
          unregister_code16(KC_LCBR);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_SHIFT);
        } else {
          unregister_code16(KC_LEFT_SHIFT);
        }  
      }  
      return false;
    case DUAL_FUNC_3:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_RCBR);
        } else {
          unregister_code16(KC_RCBR);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_RIGHT_SHIFT);
        } else {
          unregister_code16(KC_RIGHT_SHIFT);
        }  
      }  
      return false;
    case DUAL_FUNC_4:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(LGUI(KC_F12));
        } else {
          unregister_code16(LGUI(KC_F12));
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_GUI);
        } else {
          unregister_code16(KC_LEFT_GUI);
        }  
      }  
      return false;
    case DUAL_FUNC_5:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(LGUI(KC_MINUS));
        } else {
          unregister_code16(LGUI(KC_MINUS));
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_MINUS)));
        } else {
          unregister_code16(LGUI(LSFT(KC_MINUS)));
        }  
      }  
      return false;
    case RGB_SLD:
        if (record->event.pressed) {
            rgblight_mode(1);
        }
        return false;
  }
  return true;
}

