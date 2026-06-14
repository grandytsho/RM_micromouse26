#include "sensors.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

/* =====================================================================
 *  IR pin tables. emitterPins[] are digital outputs (LED drivers),
 *  sensorPins[] are analog inputs (phototransistor / ADC). The order
 *  matches the IRSensor enum exactly.
 * ===================================================================== */
static const uint8_t emitterPins[IR_COUNT] = {39, 23, 15, 24, 21, 41};
static const uint8_t sensorPins[IR_COUNT]  = {38, 22, 14, 25, 20, 40};

/* =====================================================================
 *  BNO085 driver state
 * ===================================================================== */
static Adafruit_BNO08x   bno08x(-1);          // -1 = no hardware reset pin
static sh2_SensorValue_t sensorValue;
static Quaternion        latestQuat = {0, 0, 0, 1, false};

/* Set by the Pin 26 ISR; consumed in serviceIMU(). volatile because it
 * crosses the ISR <-> main-loop boundary. */
static volatile bool imuDataReady = false;
static void imuISR() { imuDataReady = true; }

void sensorsInit() {
    /* ---------- IR array ---------- */
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(emitterPins[i], OUTPUT);
        digitalWrite(emitterPins[i], LOW);   // emitters OFF by default
        pinMode(sensorPins[i], INPUT);       // analog input
    }

    /* ---------- BNO085 on Wire1 ---------- */
    // INT pin: configured as plain INPUT per spec. The line is open-drain
    // active-LOW and held HIGH by the breakout, so INPUT is sufficient.
    pinMode(BNO08X_INT, INPUT);

    Wire1.begin();              // SCL1 = 16, SDA1 = 17
    Wire1.setClock(400000);     // 400 kHz fast-mode I2C

    if (bno08x.begin_I2C(BNO08X_ADDR, &Wire1)) {
        // Rotation Vector = fused absolute orientation quaternion.
        // 10000 us -> ~100 Hz reporting.
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    }
    // else: IMU absent -> latestQuat stays .valid == false

    // Trigger on the falling edge (data-ready assertion).
    attachInterrupt(digitalPinToInterrupt(BNO08X_INT), imuISR, FALLING);
}

void serviceIMU() {
    if (!imuDataReady) return;      // no new data -> do NOT touch I2C
    imuDataReady = false;

    // If the IMU was reset (brown-out, etc.) its reports stop until we
    // re-enable them.
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    }

    // Drain every report queued behind this interrupt.
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            latestQuat.i    = sensorValue.un.rotationVector.i;
            latestQuat.j    = sensorValue.un.rotationVector.j;
            latestQuat.k    = sensorValue.un.rotationVector.k;
            latestQuat.real = sensorValue.un.rotationVector.real;
            latestQuat.valid = true;
        }
    }
}

Quaternion getQuaternion() {
    return latestQuat;
}

/* ---------------------------------------------------------------------
 *  settleMicros(): a SHORT, BOUNDED busy-wait built on micros().
 *
 *  This is deliberately NOT delay(). The IR phototransistor needs a brief
 *  settling time after the emitter switches on before its output is valid.
 *  We spin on micros() for a fixed, tiny window (~1 ms) instead of using
 *  delay() so we stay compliant with the "no delay()" rule while still
 *  giving the analog front-end time to respond. Keep this as small as the
 *  hardware allows.
 * ------------------------------------------------------------------- */
static inline void settleMicros(uint32_t us) {
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) { /* spin */ }
}

/* ---------------------------------------------------------------------
 *  DIFFERENTIAL IR READ — why it cancels ambient light
 *
 *  A bare reflectance read is corrupted by sunlight / room lighting that
 *  also lands on the phototransistor. The trick: take two samples close
 *  together in time and subtract them.
 *
 *      ambient = read with emitter OFF   (room light only)
 *      active  = read with emitter ON    (room light + our reflected IR)
 *      signal  = active - ambient        (room light cancels out)
 *
 *  Because the two reads happen ~1 ms apart, the ambient component is
 *  essentially identical in both and subtracts to zero, leaving only the
 *  light WE emitted and that the wall reflected back. A larger 'signal'
 *  therefore means a nearer/more-reflective surface.
 *
 *  Sign note: this assumes the receiver circuit produces a HIGHER ADC
 *  count with more incident IR. If your front-end inverts (pull-up +
 *  phototransistor to ground -> more light = lower count), flip the
 *  subtraction order.
 * ------------------------------------------------------------------- */
int readIRDifferential(IRSensor s) {
    uint8_t emit = emitterPins[s];
    uint8_t sens = sensorPins[s];

    int ambient = analogRead(sens);   // 1) emitter OFF -> ambient baseline
    digitalWrite(emit, HIGH);         // 2) emitter ON
    settleMicros(1000);               //    ~1 ms for the receiver to settle
    int active = analogRead(sens);    // 3) emitter ON  -> ambient + reflected
    digitalWrite(emit, LOW);          // 4) emitter OFF

    return active - ambient;          // 5) ambient cancels -> reflected IR
}

void readAllIR(int *out) {
    for (int i = 0; i < IR_COUNT; i++) {
        out[i] = readIRDifferential((IRSensor)i);
    }
}
