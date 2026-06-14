#include <Arduino.h>
#include "robot_api.h"
#include <math.h>

/* =====================================================================
 *  MAIN  (main.cpp)
 * =====================================================================
 *  setup() brings every HAL module online.
 *  loop()  is a clean non-blocking scheduler:
 *      - services the Bluetooth PID parser
 *      - debounces + reports the UI switches
 *      - monitors the battery (kill-switch)
 *      - services the IMU (only on its data-ready interrupt)
 *      - drives the status LED
 *      - prints a formatted telemetry line every 100 ms
 *
 *  There is NO delay() anywhere. All cadence is from millis().
 * ===================================================================== */

static uint32_t lastStatusMs = 0;
static const uint32_t STATUS_PERIOD = 100;  // ms between telemetry lines

float kp_turn = 0.0;
float kd_turn = 0.0;
float ki_turn = 0.0;

float baseline;

void setup() {
  Serial.begin(115200);  // USB serial: local debug / bring-up

  // Order matters a little: telemetry first so other modules can print,
  // then power/status (also disables ToF + sets ADC resolution), then
  // motors and sensors.
  telemetryInit();
  systemStateInit();
  motorsInit();
  sensorsInit();

  btPrintln("Micromouse HAL online.");
  Serial.println("Micromouse HAL online.");

  baseline = getBaseline(); 
}

void loop() {
  // ---- 1. Fast, every-iteration services (all non-blocking) ----
  serviceBluetoothParser();  // apply any "P../I../D.." commands
  serviceUI();               // debounce switches, report changes over BT
  serviceBattery();          // <-- may enter the safe loop and never return
  serviceIMU();              // read quaternion only if Pin 26 fired
  updateStatusLED();         // green / pulsing-blue state machine

  // ---- 2. Throttled telemetry (every 100 ms) ----
  uint32_t now = millis();
  if (now - lastStatusMs >= STATUS_PERIOD) {
    lastStatusMs = now;

    // Snapshot all the data we want to report.
    int ir[IR_COUNT];
    readAllIR(ir);  // 6x differential reads
    Quaternion q = getQuaternion();
    float vb = readBatteryVoltage();

    // Compose one formatted line. snprintf keeps it bounded & fast;
    // Teensy's newlib supports %f.
    char buf[220];
    snprintf(buf, sizeof(buf),
             "VBAT:%.2fV | IR F1:%d F2:%d FL:%d FR:%d L:%d R:%d | "
             "ENC L:%ld R:%ld | Q:%.3f,%.3f,%.3f,%.3f | "
             "Kp:%.3f Ki:%.3f Kd:%.3f",
             vb,
             ir[IR_FRONT1], ir[IR_FRONT2], ir[IR_FRONT_LEFT],
             ir[IR_FRONT_RIGHT], ir[IR_LEFT], ir[IR_RIGHT],
             getLeftTicks(), getRightTicks(),
             q.i, q.j, q.k, q.real,
             Kp, Ki, Kd);

    btPrintln(String(buf));  // out the radio (only if a device is paired)
    Serial.println(buf);     // and out USB for bench debugging
  }
}

float getYaw() {
  Quaternion q;
  while (true) {
    q = getQUaternion();
    if (q.valid) break;
  }

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
void turnAngle(float angle) {
  float currentAngle = getYaw();
  float targetAngle = currentAngle + angle;

  if (targetAngle > 360.0) targetHeading -= 360.0;
  if (targetAngle < 0.0) targetHeading += 360.0;

  float angleThreshold = 1.0;
  float error = targetAngle - currentAngle;
  float lastError = 0.0;

  while (true) {
    currentAngle = getYaw();

    error = targetAngle - currentAngle;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if(abs(error)<=angleThreshold) break; 

    float errorDerivative = error - lastError; 
    lastError = error; 

    int turnSpeed = kp_turn*error + kd_turn*errorDerivative ;
    turnSpeed = constrain(turnSpeed, 60, 140); 
    setMotorSpeeds(speed,-speed);
  }
}

float getLeft(){ return 0.0;}

float getRight(){ return 0.0;}

float getBaseline(){
    float avgBaseline = (getLeft() + getRight())/2.0;
    return avgBaseline; 
}

void centerUntilDist(float dist){
    
}

void moveForward() {
}