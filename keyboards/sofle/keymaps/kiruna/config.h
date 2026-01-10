#pragma once

#define PERMISSIVE_HOLD
#define CHORDAL_HOLD

#define NO_AUTO_SHIFT_ALPHA


/* #define LSPO_KEYS KC_LSFT, KC_LSFT, KC_LBRC
#define RSPC_KEYS KC_RSFT, KC_LSFT, KC_RBRC */
#define LAPO_KEYS KC_LSFT, KC_LSFT, KC_LBRC
#define RCPC_KEYS KC_RSFT, KC_RSFT, KC_RBRC
#define LCPO_KEYS KC_LGUI, KC_TRNS, KC_LBRC
#define RAPC_KEYS KC_LALT, KC_TRNS, KC_RBRC

#define MASTER_LEFT

#define CUSTOM_FONT

#define CUSTOM_LAYER_READ //if you remove this it causes issues - needs better guarding


#define TAPPING_FORCE_HOLD
#ifdef TAPPING_TERM
    #undef TAPPING_TERM
    #define TAPPING_TERM 200
#endif

#define ENCODER_DIRECTION_FLIP
#undef ENCODER_RESOLUTION
#define ENCODER_RESOLUTION 4

#define RGBLIGHT_SLEEP

#define RGBLIGHT_LAYERS

/* ws2812 RGB LED */
#define RGB_DI_PIN D3

#ifdef RGB_MATRIX_ENABLE
#define RGBLED_NUM 35    // Number of LEDs
#define RGBLED_NUM 35    // Number of LEDs
#define DRIVER_LED_TOTAL RGBLED_NUM
#endif

#ifdef RGBLIGHT_ENABLE
    #undef RGBLED_NUM
/*
    #define RGBLED_NUM 70

    #define RGBLED_SPLIT { 35, 35 } // haven't figured out how to use this yet
*/

    #define RGBLIGHT_LIMIT_VAL 120
    #define RGBLIGHT_HUE_STEP 10
    #define RGBLIGHT_SAT_STEP 17
    #define RGBLIGHT_VAL_STEP 17
#endif

#ifdef RGB_MATRIX_ENABLE
// #   define RGB_MATRIX_KEYPRESSES // reacts to keypresses
// #   define RGB_MATRIX_KEYRELEASES // reacts to keyreleases (instead of keypresses)
// #   define RGB_DISABLE_AFTER_TIMEOUT 0 // number of ticks to wait until disabling effects
#   define RGB_DISABLE_WHEN_USB_SUSPENDED // turn off effects when suspended
// #   define RGB_MATRIX_FRAMEBUFFER_EFFECTS
// #   define RGB_MATRIX_LED_PROCESS_LIMIT (DRIVER_LED_TOTAL + 4) / 5 // limits the number of LEDs to process in an animation per task run (increases keyboard responsiveness)
// #   define RGB_MATRIX_LED_FLUSH_LIMIT 16 // limits in milliseconds how frequently an animation will update the LEDs. 16 (16ms) is equivalent to limiting to 60fps (increases keyboard responsiveness)
#    define RGB_MATRIX_MAXIMUM_BRIGHTNESS 150 // limits maximum brightness of LEDs to 150 out of 255. Higher may cause the controller to crash.

// #define RGB_MATRIX_STARTUP_MODE RGB_MATRIX_GRADIENT_LEFT_RIGHT

#    define RGB_MATRIX_HUE_STEP 8
#    define RGB_MATRIX_SAT_STEP 8
#    define RGB_MATRIX_VAL_STEP 8
#    define RGB_MATRIX_SPD_STEP 10

#endif
