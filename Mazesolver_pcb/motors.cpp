#include "motors.h"

volatile long leftEncoderTicks  = 0;
volatile long rightEncoderTicks = 0;

/* Once the kill-switch fires we latch motors OFF so any stray
 * setMotorSpeeds() call is ignored. */
static bool motorsKilled = false;

/* =====================================================================
 *  QUADRATURE DECODE (4x)
 * =====================================================================
 *  With both channels (A,B) interrupting on CHANGE, each mechanical
 *  detent produces 4 edges. Direction is recovered from the relative
 *  phase of A and B:
 *
 *    On an A edge :  if (A == B)  count up   else count down
 *    On a  B edge :  if (A == B)  count down else count up
 *
 *  If your wheel counts the "wrong way", simply swap the A/B pins of
 *  that encoder (or flip the +/- in that motor's two ISRs).
 *  digitalReadFast() is used because these run in interrupt context and
 *  must be as short as possible.
 * ===================================================================== */
static void leftA_ISR() {
    if (digitalReadFast(LEFT_ENC_A) == digitalReadFast(LEFT_ENC_B))
        leftEncoderTicks++;
    else
        leftEncoderTicks--;
}
static void leftB_ISR() {
    if (digitalReadFast(LEFT_ENC_A) == digitalReadFast(LEFT_ENC_B))
        leftEncoderTicks--;
    else
        leftEncoderTicks++;
}
static void rightA_ISR() {
    if (digitalReadFast(RIGHT_ENC_A) == digitalReadFast(RIGHT_ENC_B))
        rightEncoderTicks++;
    else
        rightEncoderTicks--;
}
static void rightB_ISR() {
    if (digitalReadFast(RIGHT_ENC_A) == digitalReadFast(RIGHT_ENC_B))
        rightEncoderTicks--;
    else
        rightEncoderTicks++;
}

void motorsInit() {
    // --- TB6612 outputs ---
    pinMode(LEFT_PWM,    OUTPUT);
    pinMode(LEFT_DIR_A,  OUTPUT);
    pinMode(LEFT_DIR_B,  OUTPUT);
    pinMode(RIGHT_PWM,   OUTPUT);
    pinMode(RIGHT_DIR_A, OUTPUT);
    pinMode(RIGHT_DIR_B, OUTPUT);

    // 8-bit PWM (0..255) to match setMotorSpeeds() range.
    analogWriteResolution(8);
    // Push PWM above the audible band (~20 kHz) so the motors don't whine.
    analogWriteFrequency(LEFT_PWM,  20000);
    analogWriteFrequency(RIGHT_PWM, 20000);

    // --- Encoder inputs ---
    // N20 modules usually have their own pull-ups; INPUT_PULLUP is a safe
    // default in case they don't.
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

    disableMotors();          // start stopped
    motorsKilled = false;     // ...but not latched off
}

/* ---- Drive one channel ----------------------------------------------
 * speed: -255..+255. Sign sets direction, magnitude sets duty.
 * --------------------------------------------------------------------- */
static void driveChannel(int pwmPin, int dirA, int dirB, int speed) {
    if (speed >= 0) {                 // forward
        digitalWrite(dirA, HIGH);
        digitalWrite(dirB, LOW);
    } else {                          // reverse
        digitalWrite(dirA, LOW);
        digitalWrite(dirB, HIGH);
        speed = -speed;
    }
    if (speed > 255) speed = 255;     // clamp to 8-bit PWM
    analogWrite(pwmPin, speed);
}

void setMotorSpeeds(int left, int right) {
    if (motorsKilled) return;         // safety latch: ignore after kill
    driveChannel(LEFT_PWM,  LEFT_DIR_A,  LEFT_DIR_B,  left);
    driveChannel(RIGHT_PWM, RIGHT_DIR_A, RIGHT_DIR_B, right);
}

void disableMotors() {
    // Coast stop: PWM 0 and both direction pins LOW on each channel.
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
    digitalWrite(LEFT_DIR_A,  LOW);
    digitalWrite(LEFT_DIR_B,  LOW);
    digitalWrite(RIGHT_DIR_A, LOW);
    digitalWrite(RIGHT_DIR_B, LOW);
    motorsKilled = true;              // latch off (cleared only by motorsInit)
}

/* ---- Glitch-free counter reads --------------------------------------
 * A 32-bit long is not read atomically on every path, and an encoder ISR
 * could fire mid-read. Briefly disabling interrupts guarantees a coherent
 * snapshot.
 * --------------------------------------------------------------------- */
long getLeftTicks() {
    noInterrupts();
    long v = leftEncoderTicks;
    interrupts();
    return v;
}
long getRightTicks() {
    noInterrupts();
    long v = rightEncoderTicks;
    interrupts();
    return v;
}
void resetEncoders() {
    noInterrupts();
    leftEncoderTicks  = 0;
    rightEncoderTicks = 0;
    interrupts();
}
