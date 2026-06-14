#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

/* =====================================================================
 *  MODULE 4 : MOTOR CONTROL & ODOMETRY
 * =====================================================================
 *  Driver  : TB6612FNG dual H-bridge, driven with analogWrite() PWM.
 *  Encoders: N20 quadrature, decoded with hardware interrupts on CHANGE.
 *
 *  TB6612 truth table (per channel):
 *      IN1  IN2   ->  motor
 *      H    L     ->  forward
 *      L    H     ->  reverse
 *      L    L     ->  coast (free spin)
 *      H    H     ->  short brake
 *  PWM (PWMA/PWMB) sets the duty / speed.
 * ===================================================================== */

/* ---- TB6612 pin map -------------------------------------------------- */
#define LEFT_PWM     9
#define LEFT_DIR_A   2
#define LEFT_DIR_B   4
#define RIGHT_PWM    28
#define RIGHT_DIR_A  3
#define RIGHT_DIR_B  5

/* ---- N20 quadrature encoder pins ------------------------------------
 * Two channels per wheel, 90 deg out of phase. Interrupting on CHANGE of
 * BOTH channels gives full 4x decoding (max resolution).
 * --------------------------------------------------------------------- */
#define LEFT_ENC_A   0
#define LEFT_ENC_B   1
#define RIGHT_ENC_A  30
#define RIGHT_ENC_B  31

/* ---- Tick counters --------------------------------------------------
 * Modified inside ISRs, read from loop() -> MUST be volatile.
 * 'long' (32-bit) gives plenty of range before wrap.
 * --------------------------------------------------------------------- */
extern volatile long leftEncoderTicks;
extern volatile long rightEncoderTicks;

/* ---- Public API ----------------------------------------------------- */
void motorsInit();
void setMotorSpeeds(int left, int right);  // -255..+255 (sign = direction)
void disableMotors();                      // hard coast-stop (kill switch)
long getLeftTicks();                       // safe (interrupt-guarded) read
long getRightTicks();
void resetEncoders();

#endif // MOTORS_H
