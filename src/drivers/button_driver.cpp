#ifdef MAIN_BRAIN

#include "button_driver.h"
#include <Arduino.h>

typedef struct {
    uint8_t  pin;
    bool     last_raw;
    bool     state;
    uint32_t last_change_ms;
    bool     event;
} Button_t;

static Button_t s_next      = { BTN_NEXT_PIN, true, true, 0, false };
static Button_t s_hold_btn  = { BTN_HOLD_PIN, true, true, 0, false };
static bool     s_hold_active = false;

static void update_btn(Button_t *b) {
    bool raw = (bool)digitalRead(b->pin);   // HIGH = released (internal pullup)
    if (raw == b->last_raw) return;
    b->last_raw = raw;
    if ((millis() - b->last_change_ms) < BTN_DEBOUNCE_MS) return;
    b->last_change_ms = millis();
    bool pressed = !raw;                    // active LOW
    if (pressed && !b->state) b->event = true;
    b->state = pressed;
}

void button_driver_init(void) {
    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(BTN_HOLD_PIN, INPUT_PULLUP);
    Serial.printf("[BTN] init OK — NEXT=GPIO%d HOLD=GPIO%d\n",
                  BTN_NEXT_PIN, BTN_HOLD_PIN);
}

void button_driver_update(void) {
    update_btn(&s_next);
    update_btn(&s_hold_btn);
    if (s_hold_btn.event) {
        s_hold_btn.event = false;
        s_hold_active    = !s_hold_active;
        Serial.printf("[BTN] hold %s\n", s_hold_active ? "ON" : "OFF");
    }
}

bool button_next_pressed(void) {
    if (s_next.event) { s_next.event = false; return true; }
    return false;
}

bool button_hold_active(void) { return s_hold_active; }

#endif // MAIN_BRAIN
