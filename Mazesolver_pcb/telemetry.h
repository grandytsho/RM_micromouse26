#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

/* =====================================================================
 *  MODULE 2 : TELEMETRY & PID TUNING
 * =====================================================================
 *  Bluetooth link is an HC-05 on the Teensy 4.1 hardware UART "Serial2".
 *    - Serial2 RX = Pin 7  (Teensy receives  <- HC-05 TXD)
 *    - Serial2 TX = Pin 8  (Teensy transmits -> HC-05 RXD)
 *
 *  The HC-05 "STATE" pin (here wired to Pin 6) is driven HIGH by the
 *  module ONLY while a remote device is actually paired/connected.
 *  We gate every outbound transmission on this pin so we never blast
 *  bytes into a disconnected radio (which just wastes CPU + UART time).
 * ===================================================================== */

#define BT_SERIAL        Serial2     // hardware UART, pins 7(RX)/8(TX)
#define BT_BAUD          115200
#define BT_STATE_PIN     6           // HIGH == a phone/laptop is paired

/* ---- Global PID gains -------------------------------------------------
 * These live here (definition in telemetry.cpp) so the tuning loop and
 * the BT parser share ONE copy. They are updated from loop() context
 * (not from an ISR), so they do not need to be volatile.
 * --------------------------------------------------------------------- */
extern float Kp;
extern float Ki;
extern float Kd;

/* ---- Public API ----------------------------------------------------- */
void telemetryInit();              // open Serial2, configure STATE pin
bool btConnected();                // true when BT_STATE_PIN == HIGH
void serviceBluetoothParser();     // NON-BLOCKING: call once per loop()
void btPrint(const String &s);     // transmits only if btConnected()
void btPrintln(const String &s);

#endif // TELEMETRY_H
