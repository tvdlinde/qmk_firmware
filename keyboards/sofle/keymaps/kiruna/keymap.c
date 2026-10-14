#include <stdio.h>

#include QMK_KEYBOARD_H

#define HYP(kc) MT(MOD_LSFT | MOD_LALT | MOD_LCTL | MOD_LGUI, kc)

#define INDICATOR_BRIGHTNESS 90  // layer underglow; was 120 (RGBLIGHT_LIMIT_VAL), dimmed to ease right-OLED noise

#define SET_ALL(hsv) {0, 72, hsv}
#define SET_ALL_DIM(hue) {0, 72, hue, 255, INDICATOR_BRIGHTNESS}

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


// The two GUI home-pinky keys are ordinary mod-taps (LGUI_T on backspace and quote);
// give them a slightly shorter tapping term than the global default.
uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LGUI_T(KC_BSPC): return 180;   // ms – adjust to your taste
        case LGUI_T(KC_QUOTE): return 180;   // ms – adjust to your taste
        default:            return TAPPING_TERM;
    }
}

// Flow Tap is for the home-pinky mod-taps; the thumb layer-taps to _GRN/_BLU
// live on Space, which Flow Tap's default alphabet also covers. Without this
// override, holding one right after a fast keystroke (i.e. most of the time)
// resolves as a bare space instead of activating the layer. Force those two
// keys to always honor a real hold, and leave the default behavior for
// everything else Flow Tap applies to.
uint16_t get_flow_tap_term(uint16_t keycode, keyrecord_t *record, uint16_t prev_keycode) {
    switch (keycode) {
        case LT(_GRN, KC_SPACE):
        case LT(_BLU, KC_SPACE):
            return 0;
        default:
            if (is_flow_tap_key(keycode) && is_flow_tap_key(prev_keycode)) {
                return FLOW_TAP_TERM;
            }
            return 0;
    }
}

// Only the backspace mod-tap needs custom handling (its tap sends Ctrl+Backspace).
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LGUI_T(KC_BSPC):
            // Now a genuine mod-tap: hold = Left GUI (decided by QMK's tap-hold engine,
            // so Flow Tap and Chordal Hold apply). Override only the TAP so it emits
            // Ctrl+Backspace (delete word) instead of a bare Backspace.
            if (record->tap.count && record->event.pressed) {
                tap_code16(LCTL(KC_BSPC));
                return false;
            }
            break;
    }

    // All other keys fall through to the default handler
    return true;
}

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_MAIN] = LAYOUT(
  //,------------------------------------------------.                        ,---------------------------------------------------.
  KC_GRAVE, KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                           KC_6,    KC_7,   KC_8,    KC_9,    KC_0,    KC_MINUS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
  KC_TAB,   KC_Q,   KC_W,    KC_F,    KC_P,    KC_B,                           KC_J,    KC_L,   KC_U,    KC_Y,    KC_SCLN, KC_BSLS,
  //|------+-------+--------+--------+--------+------|                        |--------+-------+--------+--------+--------+---------|
LGUI_T(KC_BSPC), KC_A,   KC_R,    KC_S,    KC_T,    KC_G,                           KC_M,    KC_N,   KC_E,    KC_I,    KC_O,    LGUI_T(KC_QUOTE),
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
  KC_CAPS,KC_TRNS,KC_MINUS,KC_LPRN,KC_RPRN,KC_DLR,                      KC_MINUS, KC_P4,  KC_P5,   KC_P6,   KC_KP_PLUS,KC_QUOTE,
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


const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] = LAYOUT(
  //,------------------------------------------------.                         ,---------------------------------------------------.
  'L', 'L', 'L', 'L', 'L', 'L',                                               'R', 'R', 'R', 'R', 'R', 'R',
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
 'L', 'L', 'L', 'L', 'L', 'L',                                                'R', 'R', 'R', 'R', 'R', 'R',
  //|------+-------+--------+--------+--------+------|                         |--------+-------+--------+--------+--------+---------|
  'L', 'L', 'L', 'L', 'L', 'L',                                                'R', 'R', 'R', 'R', 'R', 'R',
  //|------+-------+--------+--------+--------+------|  ===  |         |  ===  |--------+-------+--------+--------+--------+---------|
  'L', 'L', 'L', 'L', 'L', 'L', '*',                                          '*', 'R', 'R', 'R', 'R', 'R', 'R',
  //|------+-------+--------+--------+--------+------|  ===  |         |  ===  |--------+-------+--------+--------+--------+---------|
                  'L', 'L', 'L', '*', '*',                                '*', '*', 'R', 'R', 'R'
  //            \--------+--------+--------+---------+-------|         |--------+---------+--------+---------+-------/
);

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

#include "raw_hid.h"
#include "transactions.h"

// Status packet from host/sofle_status.py (32-byte Raw HID report, ~1 Hz).
// Bytes 8..12 are the right-half fields; they are forwarded to the slave.
#define STATUS_MAGIC  0x5A
enum {
    ST_MAGIC, ST_HOUR, ST_MIN, ST_DAY, ST_MONTH, ST_VOL, ST_FLAGS, ST_BAT,
    ST_LOAD_LO, ST_LOAD_HI, ST_CPU, ST_DISK, ST_MEM, ST_LEN
};
#define ST_RIGHT      ST_LOAD_LO
#define ST_RIGHT_LEN  (ST_LEN - ST_RIGHT)
#define FLAG_MUTED    0x01
#define FLAG_CHARGING 0x02
#define FLAG_NO_BAT   0x04

#define STATUS_STALE_MS 5000   // fields show "--" after this long without data
#define OLED_IDLE_MS    60000  // screen off when no data and no typing for this long

static uint8_t  status[ST_LEN];
static uint32_t last_rx;
static bool     have_rx;
static bool     forward_pending;

void raw_hid_receive(uint8_t *data, uint8_t length) {
    if (length < ST_LEN || data[ST_MAGIC] != STATUS_MAGIC) {
        return;
    }
    memcpy(status, data, ST_LEN);
    last_rx         = timer_read32();
    have_rx         = true;
    forward_pending = true;
}

static void status_slave_handler(uint8_t in_len, const void *in_data, uint8_t out_len, void *out_data) {
    if (in_len == ST_RIGHT_LEN) {
        memcpy(&status[ST_RIGHT], in_data, ST_RIGHT_LEN);
        last_rx = timer_read32();
        have_rx = true;
    }
}

static void status_init(void) {
    transaction_register_rpc(USER_SYNC_STATUS, status_slave_handler);
}

void housekeeping_task_user(void) {
    if (forward_pending && is_keyboard_master()) {
        if (transaction_rpc_send(USER_SYNC_STATUS, ST_RIGHT_LEN, &status[ST_RIGHT])) {
            forward_pending = false;
        }
    }
}

// The screens are rotated to portrait: 5 characters x 16 rows.
static char line[6];

static void line_clear(void) {
    memset(line, ' ', 5);
    line[5] = 0;
}

static void put_2d(uint8_t pos, uint8_t v) {
    line[pos]     = '0' + v / 10 % 10;
    line[pos + 1] = '0' + v % 10;
}

// Right-aligned 0..255 ending at pos.
static void put_num(uint8_t pos, uint8_t v) {
    do {
        line[pos--] = '0' + v % 10;
        v /= 10;
    } while (v);
}

static void write_row(uint8_t row) {
    oled_set_cursor(0, row);
    oled_write(line, false);
}

static void write_label(uint8_t row, const char *label) {
    oled_set_cursor(0, row);
    oled_write_P(label, false);
}

// "  23%" or "   --"
static void write_pct(uint8_t row, uint8_t v, bool fresh) {
    line_clear();
    if (fresh) {
        put_num(3, v);
        line[4] = '%';
    } else {
        line[3] = line[4] = '-';
    }
    write_row(row);
}

static void render_left(bool fresh) {
    line_clear();
    if (fresh) {
        put_2d(0, status[ST_HOUR]);
        line[2] = ':';
        put_2d(3, status[ST_MIN]);
    } else {
        line[0] = line[1] = line[3] = line[4] = '-';
        line[2] = ':';
    }
    write_row(0);
    line_clear();
    if (fresh) {
        put_2d(0, status[ST_DAY]);
        line[2] = '/';
        put_2d(3, status[ST_MONTH]);
    }
    write_row(1);

    write_label(3, PSTR("Vol  "));
    if (fresh && (status[ST_FLAGS] & FLAG_MUTED)) {
        write_label(4, PSTR(" mute"));
    } else {
        write_pct(4, status[ST_VOL], fresh);
    }

    write_label(6, PSTR("Bat  "));
    if (fresh && (status[ST_FLAGS] & FLAG_NO_BAT)) {
        write_label(7, PSTR("  AC "));
    } else {
        write_pct(7, status[ST_BAT], fresh);
        if (fresh && (status[ST_FLAGS] & FLAG_CHARGING)) {
            oled_set_cursor(0, 7);
            oled_write_char('+', false);
        }
    }

    write_label(13, PSTR("LAYER"));
    switch (get_highest_layer(layer_state)) {
        case _RED:
            write_label(14, PSTR("Red  "));
            break;
        case _BLU:
            write_label(14, PSTR("Blue "));
            break;
        case _GRN:
            write_label(14, PSTR("Green"));
            break;
        default:
            write_label(14, PSTR("Base "));
    }
}

static void render_right(bool fresh) {
    write_label(0, PSTR("Load "));
    line_clear();
    if (fresh) {
        // load average x100 -> " 1.24" / "12.40" / "  99+" (beyond 99.99)
        uint16_t load = status[ST_LOAD_LO] | (status[ST_LOAD_HI] << 8);
        if (load > 9999) {
            line[2] = line[3] = '9';
            line[4] = '+';
        } else {
            put_2d(3, load % 100);
            line[2] = '.';
            put_num(1, load / 100);
        }
    } else {
        line[3] = line[4] = '-';
    }
    write_row(1);

    write_label(3, PSTR("CPU  "));
    write_pct(4, status[ST_CPU], fresh);
    write_label(6, PSTR("Disk "));
    write_pct(7, status[ST_DISK], fresh);
    write_label(9, PSTR("Mem  "));
    write_pct(10, status[ST_MEM], fresh);
}

oled_rotation_t oled_init_user(oled_rotation_t rotation) {
    return OLED_ROTATION_270;
}

bool oled_task_user(void) {
    bool fresh = have_rx && timer_elapsed32(last_rx) < STATUS_STALE_MS;

    // Keep the screens on while data flows (or, on the master, while typing);
    // blank them otherwise to spare the OLEDs.
    if (!fresh && (!is_keyboard_master() || last_input_activity_elapsed() > OLED_IDLE_MS)) {
        oled_off();
        return false;
    }
    oled_on();

    if (is_keyboard_master()) {
        render_left(fresh);
    } else {
        render_right(fresh);
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
    SET_ALL_DIM(170)  // HSV_BLUE hue
    );

const rgblight_segment_t PROGMEM layer_grn_lights[] = RGBLIGHT_LAYER_SEGMENTS(
    SET_ALL_DIM(85)   // HSV_GREEN hue
);

const rgblight_segment_t PROGMEM layer_red_lights[] = RGBLIGHT_LAYER_SEGMENTS(
  SET_ALL_DIM(0)    // HSV_RED hue
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

#ifdef OLED_ENABLE
    status_init();
#endif

}
#endif
