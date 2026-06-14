#include "system_state.h"
#include "motors.h"        // disableMotors()
#include "telemetry.h"     // btPrintln(), btConnected()

#include <FastLED.h>

/* The single addressable LED. enterSafeState() also drives this array,
 * so it lives at file scope. */
static CRGB leds[NUM_LEDS];

/* ---------------------------------------------------------------------
 *  Debounce bookkeeping for the two UI switches. We use the classic
 *  "wait for the reading to stay stable for DEBOUNCE_MS before believing
 *  it" approach, driven entirely by millis() (no blocking).
 * ------------------------------------------------------------------- */
#define DEBOUNCE_MS 25

struct DebouncedInput {
    uint8_t  pin;
    int      stableState;   // last accepted (debounced) level
    int      lastReading;   // most recent raw sample
    uint32_t lastChangeMs;  // when lastReading last changed
    const char *name;
};

static DebouncedInput uiAlgo = { ALGO_CONTROL_PIN, HIGH, HIGH, 0, "ALGO_CONTROL" };
static DebouncedInput uiRun  = { RUN_MODE_PIN,     HIGH, HIGH, 0, "RUN_MODE" };

void systemStateInit() {
    // 12-bit ADC so VBAT math matches ADC_MAX_COUNTS (4095).
    analogReadResolution(12);

    // Hold every ToF XSHUT line LOW -> ToF sensors stay shut down.
    for (int p = TOF_XSHUT_FIRST; p <= TOF_XSHUT_LAST; p++) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }

    // UI switches: INPUT_PULLUP assumes each switch shorts the pin to GND
    // when actuated (idle = HIGH, pressed = LOW). Flip logic if your wiring
    // is active-high.
    pinMode(ALGO_CONTROL_PIN, INPUT_PULLUP);
    pinMode(RUN_MODE_PIN,     INPUT_PULLUP);

    // WS2812B is GRB-ordered; FastLED handles the precise timing on Pin 10.
    FastLED.addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    leds[0] = CRGB::Green;
    FastLED.show();
}

/* ---- Read TRUE battery voltage --------------------------------------
 * We oversample (fast, ~microseconds) and average to suppress ADC noise,
 * then undo the divider. This is NOT a blocking wait — it's just N quick
 * back-to-back conversions.
 *
 *   v_pin   = (avg_counts / 4095) * 3.3        // voltage at the ADC pin
 *   v_pack  = v_pin * 4                         // undo the 30k/10k divider
 * ------------------------------------------------------------------- */
float readBatteryVoltage() {
    const int N = 16;
    uint32_t sum = 0;
    for (int i = 0; i < N; i++) sum += analogRead(VBAT_PIN);
    float avg   = (float)sum / N;
    float v_pin = (avg / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    return v_pin * VBAT_DIVIDER;
}

void serviceBattery() {
    if (readBatteryVoltage() < VBAT_CUTOFF) {
        enterSafeState();   // does not return
    }
}

/* ---- The kill-switch safe state -------------------------------------
 * Motors are latched OFF and we flash the LED red forever. Timing is done
 * with millis() (no delay()). This loop intentionally never exits — the
 * only way out is a power cycle, which is exactly what you want on a LiPo
 * that has hit its under-voltage cutoff.
 * ------------------------------------------------------------------- */
void enterSafeState() {
    disableMotors();

    bool lastOn = false;
    for (;;) {
        bool on = ((millis() / 250) % 2) == 0;   // ~2 Hz flash
        if (on != lastOn) {                      // only refresh on change
            leds[0] = on ? CRGB::Red : CRGB::Black;
            FastLED.show();
            lastOn = on;
        }
    }
}

/* ---- Status LED state machine (non-blocking) ------------------------
 *   State 1 (Normal)             : solid green
 *   State 2 (Bluetooth connected): pulsing blue
 *   State 3 (Low battery / kill) : handled in enterSafeState() (red flash)
 *
 * beatsin8()/FastLED timing are millis()-based, so nothing blocks here.
 * ------------------------------------------------------------------- */
void updateStatusLED() {
    if (btConnected()) {
        // Smoothly breathe the blue channel between 20 and 255 at ~40 BPM.
        uint8_t bright = beatsin8(40, 20, 255);
        leds[0] = CHSV(160, 255, bright);   // hue 160 ~ blue
    } else {
        leds[0] = CRGB::Green;              // solid green = normal/idle
    }
    FastLED.show();
}

/* ---- One debounced switch; returns true if the stable state changed -- */
static bool serviceOneSwitch(DebouncedInput &sw) {
    int reading = digitalRead(sw.pin);

    if (reading != sw.lastReading) {
        sw.lastReading  = reading;
        sw.lastChangeMs = millis();          // restart the settle timer
    }

    if ((millis() - sw.lastChangeMs) >= DEBOUNCE_MS &&
        reading != sw.stableState) {
        sw.stableState = reading;            // accept the new stable level
        return true;
    }
    return false;
}

void serviceUI() {
    if (serviceOneSwitch(uiAlgo)) {
        btPrint(String(uiAlgo.name));
        btPrint(" -> ");
        btPrintln(uiAlgo.stableState == LOW ? "PRESSED" : "RELEASED");
    }
    if (serviceOneSwitch(uiRun)) {
        btPrint(String(uiRun.name));
        btPrint(" -> ");
        btPrintln(uiRun.stableState == LOW ? "PRESSED" : "RELEASED");
    }
}
