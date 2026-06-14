#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

/* =====================================================================
 *  MODULE 1 : POWER & SYSTEM STATUS
 * =====================================================================
 *  - 3S LiPo battery monitor with a hardware kill-switch.
 *  - WS2812B status LED state machine (FastLED).
 *  - Two debounced user switches reported over Bluetooth.
 *  - Holds the (unused) ToF XSHUT pins LOW so those sensors stay off.
 * ===================================================================== */

/* ---- Battery monitor -------------------------------------------------
 * VBAT is read on Pin 27 / A13 through a 30k(top)/10k(bottom) divider.
 *
 *   Divider ratio = (R_top + R_bottom) / R_bottom = (30k + 10k)/10k = 4
 *
 * So the real battery voltage is the ADC-pin voltage * 4. The /4 divider
 * keeps a full 12.6 V pack at ~3.15 V on the pin, safely under the
 * Teensy's 3.3 V ADC ceiling.
 * --------------------------------------------------------------------- */
#define VBAT_PIN          27        // A13
#define VBAT_DIVIDER      4.0f      // (30k + 10k) / 10k
#define VBAT_MAX          12.6f     // 3S fully charged (4.2 V/cell)
#define VBAT_CUTOFF       9.6f      // kill threshold (3.2 V/cell)
#define ADC_REF_VOLTAGE   3.3f      // Teensy ADC reference
#define ADC_MAX_COUNTS    4095.0f   // 12-bit resolution (set in init)

/* ---- WS2812B status LED --------------------------------------------- */
#define STATUS_LED_PIN    10
#define NUM_LEDS          1

/* ---- ToF (VL53L0X) XSHUT pins to keep DISABLED ----------------------
 * Pins 32..37 are driven LOW so every ToF sensor is held in shutdown.
 * (No ToF / MPU6050 code is initialized anywhere in this firmware.)
 * --------------------------------------------------------------------- */
#define TOF_XSHUT_FIRST   32
#define TOF_XSHUT_LAST    37

/* ---- User UI switches ------------------------------------------------ */
#define ALGO_CONTROL_PIN  29
#define RUN_MODE_PIN      12

/* ---- Public API ----------------------------------------------------- */
void  systemStateInit();
float readBatteryVoltage();   // returns TRUE pack voltage (already *4)
void  serviceBattery();       // may call enterSafeState() and never return
void  serviceUI();            // debounced read; prints changes over BT
void  updateStatusLED();      // non-blocking LED state machine
void  enterSafeState();       // motors OFF + flashing red, forever

#endif // SYSTEM_STATE_H
