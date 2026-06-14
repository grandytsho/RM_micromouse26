#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

/* =====================================================================
 *  MODULE 3 : IMU & SENSORS
 * =====================================================================
 *  IMU  : BNO085 on the SECONDARY I2C bus (Wire1):
 *             SCL1 = Pin 16, SDA1 = Pin 17.
 *         The breakout carries its own I2C pull-ups, so we do not add any.
 *         Pin 26 (INT_GPIO) is the BNO085 data-ready interrupt: the chip
 *         pulls it LOW when a fresh report is waiting. We read the I2C bus
 *         ONLY in response to that interrupt -> we never busy-poll I2C.
 *
 *  IR   : 6-channel analog reflectance array read DIFFERENTIALLY so that
 *         ambient/room light is cancelled (see sensors.cpp for the math).
 * ===================================================================== */

#define BNO08X_INT   26        // IMU data-ready interrupt (active LOW)
#define BNO08X_ADDR  0x4A      // default BNO085 I2C address (0x4B if ADR high)

/* ---- IR array channel indices --------------------------------------- */
enum IRSensor {
    IR_FRONT1 = 0,   // Emitter 39, Sensor 38
    IR_FRONT2,       // Emitter 23, Sensor 22
    IR_FRONT_LEFT,   // Emitter 15, Sensor 14
    IR_FRONT_RIGHT,  // Emitter 24, Sensor 25
    IR_LEFT,         // Emitter 21, Sensor 20
    IR_RIGHT,        // Emitter 41, Sensor 40
    IR_COUNT
};

/* ---- Orientation quaternion ----------------------------------------- */
struct Quaternion {
    float i, j, k, real;
    bool  valid;     // false until the first good report arrives
};

/* ---- Public API ----------------------------------------------------- */
void       sensorsInit();
void       serviceIMU();                 // call every loop(); reads on INT only
Quaternion getQuaternion();              // latest cached orientation
int        readIRDifferential(IRSensor s);
void       readAllIR(int *out);          // fills out[IR_COUNT]

#endif // SENSORS_H
