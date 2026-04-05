#include "rgb_led.h"
#ifdef MAIN_BRAIN

#include <Arduino.h>
#include <FastLED.h>

// Onboard WS2812B on ESP32-S3 DevKitC-1
#define RGB_PIN        48
#define RGB_NUM_LEDS   1
#define RGB_BRIGHTNESS 64

static CRGB s_leds[RGB_NUM_LEDS];
static RGBState_t s_state    = RGB_STATE_BOOT;
static uint32_t   s_last_ms  = 0;
static bool       s_phase    = false;

void rgb_led_init(void) {
    FastLED.addLeds<WS2812B, RGB_PIN, GRB>(s_leds, RGB_NUM_LEDS);
    FastLED.setBrightness(RGB_BRIGHTNESS);
    s_leds[0] = CRGB::White;
    FastLED.show();
    Serial.printf("[RGB] init OK — GPIO%d, %d LED(s)\n", RGB_PIN, RGB_NUM_LEDS);
}

void rgb_led_set_state(RGBState_t state) {
    s_state   = state;
    s_phase   = false;
    s_last_ms = 0;
}

void rgb_led_update(void) {
    uint32_t now      = millis();
    uint32_t interval = 500;
    CRGB     color_on = CRGB::Black;
    bool     blink    = false;

    switch (s_state) {
        case RGB_STATE_BOOT:
            color_on = CRGB::White;   interval = 200; blink = true;  break;
        case RGB_STATE_IDLE:
            color_on = CRGB::Blue;    interval = 1000; blink = true; break;
        case RGB_STATE_CHARGING:
            color_on = CRGB::Green;   interval = 0;   blink = false; break;
        case RGB_STATE_OVERTEMP:
            color_on = CRGB::Red;     interval = 150; blink = true;  break;
        case RGB_STATE_UNDERTEMP:
            color_on = CRGB::Cyan;    interval = 300; blink = true;  break;
        case RGB_STATE_BMS_FAULT:
            color_on = CRGB::Orange;  interval = 200; blink = true;  break;
        case RGB_STATE_COMM_TIMEOUT:
            color_on = CRGB::Yellow;  interval = 300; blink = true;  break;
        case RGB_STATE_FAULT:
            color_on = CRGB::Red;     interval = 100; blink = true;  break;
        default:
            color_on = CRGB::Purple;  interval = 500; blink = true;  break;
    }

    if (!blink) {
        s_leds[0] = color_on;
        FastLED.show();
        return;
    }

    if ((now - s_last_ms) >= interval) {
        s_last_ms = now;
        s_phase   = !s_phase;
        s_leds[0] = s_phase ? color_on : CRGB::Black;
        FastLED.show();
    }
}

#endif // MAIN_BRAIN
