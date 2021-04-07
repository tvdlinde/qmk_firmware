#include QMK_KEYBOARD_H

enum layers {
     _MAIN,
     _UPDOWN,
     _PAGEUPDOWN,
     _DELETE,
};

void encoder_update_user(uint8_t index, bool clockwise) {
   switch(biton32(layer_state)){
             case _DELETE:
                if (clockwise) {
   			    tap_code(KC_DELETE);
   				 } else {
    		    tap_code(KC_BSPACE);
                }  
                break;            
            case _PAGEUPDOWN:
                if (clockwise) {
                tap_code(KC_PGDOWN);
                } else {
                tap_code(KC_PGUP);
                }
                break;
            case _UPDOWN:
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


//
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = { //buttion closest to usb is first
  [_MAIN] = LAYOUT_ortho_1x5(
     LGUI(KC_V), LT(_UPDOWN,LGUI(KC_C)), KC_LSFT, LT(_DELETE,LGUI(KC_X)), KC_LOPT
  ),
  [_UPDOWN] = LAYOUT_ortho_1x5(
     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, MO(_PAGEUPDOWN)
  ),  
  [_PAGEUPDOWN] = LAYOUT_ortho_1x5(
     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [_DELETE] = LAYOUT_ortho_1x5(
     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LOPT
  ),
};
