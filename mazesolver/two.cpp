#include "one.h"
#include "two.h"
#include <Arduino.h>
#include <cmath>

float Kp_old = 0.7;
float Kd_old = 0.15; 
float Ki_old = 0.0;

float Kp = 0.7;
float Kd = 0.15;
float Ki = 0.0;

float Kp_100 = 0.5;
float Kd_100 = 0.15;
float Ki_100 = 0.0;

float Kp_turn = 1.1;
float Ki_turn = 0.0;
float Kd_turn = 0.03;

float Kp_align = 0.4;
float Kd_align = 0.01;

volatile float targetYaw = 0;
int TURN_THRESHOLD = 10;
const float IMU_SCALE = 1.0f;         //not using
const float BOTH_WALL_BLEND = 0.85f;  //not using
const float ONE_WALL_BLEND = 0.60f;   //not using

static unsigned long _approach_lastTime = 0;
static float _approach_lastError = NAN;
static float _approach_derivFilt = 0.0f;

//centering and wall thresholds
int LEFT_SETPOINT = 555;
int LEFT_MAX = 860;
int RIGHT_SETPOINT = 534;
int RIGHT_MAX = 930;
int LEFT_WALL_THRESHOLD = 317;
int RIGHT_WALL_THRESHOLD = 330;
int FRONT_WALL_DETECTION_THRESHOLD = 410;
int FRONT2_WALL_DETECTION_THRESHOLD = 360;
int FRONT_WALL_COLLISION_THRESHOLD = 495;
int FRONT2_WALL_COLLISION_THRESHOLD = 525;

int FRONT_MAX = 416;
int FRONT2_MAX = 576;

static float _deriv_filtered = 0.0f;
const float TICKS_PER_CM_LEFT = TICKS_PER_REVOLUTION_LEFT / WHEEL_CIRCUMFERENCE;
const float TICKS_PER_CM_RIGHT = TICKS_PER_REVOLUTION_RIGHT / WHEEL_CIRCUMFERENCE;

int MAX_BASE_SPEED = 140; // Cruising speed
int MIN_BASE_SPEED_START =50;  // Minimum speed to overcome friction
int MIN_BASE_SPEED_STOP =0;
long ACCEL_TICKS = 300;   // How many ticks it takes to reach max speed
long DECEL_TICKS = 600;   // How many ticks before target to start slowing down
bool isStopped = true;


bool wallFollower = false;

void motor1Forward(int speed) {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_PWM, speed);
}

void motor1Reverse(int speed) {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_PWM, speed);
}

void motor2Forward(int speed) {
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  analogWrite(M2_PWM, speed);
}

void motor2Reverse(int speed) {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
  analogWrite(M2_PWM, speed);
}

void motorStop() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_PWM, 0);

  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, HIGH);
  analogWrite(M2_PWM, 0);

  delay(150);

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);

  isStopped = true;  // <--- ADD THIS LINE
}

void motor1Wraper(int speed) {
  if (speed < 0) motor1Reverse(abs(speed));
  else motor1Forward(speed);
}

void motor2Wraper(int speed) {
  if (speed < 0) motor2Reverse(abs(speed));
  else motor2Forward(speed);
}


bool isWallLeft() {
  int raw = getCorrectedReadingAvg(3);
  return (raw > LEFT_WALL_THRESHOLD);
}
bool isWallRight() {
  int raw = getCorrectedReadingAvg(1);
  return (raw > RIGHT_WALL_THRESHOLD);
}

bool isWallFront() {
  int front = getCorrectedReadingAvg(0);
  int front2 = getCorrectedReadingAvg(5);

  int avg = (front + front2) / 2;
  int FRONT_WALL_DETECTION_AVG = (FRONT_WALL_DETECTION_THRESHOLD + FRONT2_WALL_DETECTION_THRESHOLD) / 2;
  return (avg > FRONT_WALL_DETECTION_AVG);
}
bool isWallFrontCollision() {
  int front = getCorrectedReadingAvg(0);
  int front2 = getCorrectedReadingAvg(5);

  int avg = (front + front2) / 2;
  int FRONT_WALL_COLLISION_AVG = (FRONT_WALL_COLLISION_THRESHOLD + FRONT2_WALL_COLLISION_THRESHOLD) / 2;
  return (avg > FRONT_WALL_COLLISION_AVG);
}
void turn(float angleDeg) {
  const int upper = 110;
  const int lower = 60;
  const float turnThreshold = 1.0f;
  // degrees — slightly relaxed for stability
  const float DERIV_ALPHA = 0.25f;  // EMA weight for new sample (lower = smoother)
  const float DT_FLOOR = 0.004f;    // FIX 1: 4 ms floor matches delay(5) reality

  float startTime = millis();

  // BT.println("turning");
  // BT.print("Kp|"); BT.println(Kp_turn);
  // BT.print("Kd|"); BT.println(Kd_turn);
  // BT.print("Upper|"); BT.println(upper);
  // BT.print("Lower|"); BT.println(lower);

  float integral = 0.0f;
  float previousError = 0.0f;
  _deriv_filtered = 0.0f;

  unsigned long lastTime = millis();

  // Fast read: 2ms timeout prevents blocking
  float startYaw = getYaw(15);
  delay(15);

  // A positive angleDeg turns right, negative turns left
  targetYaw = startYaw + angleDeg;

  // STRICT NORMALIZATION: Wrap target directly to the IMU's native -180 to +180 range
  while (targetYaw > 180.0f) targetYaw -= 360.0f;
  while (targetYaw <= -180.0f) targetYaw += 360.0f;

  while (true) {
    if (millis() - startTime > 3000UL) {
      BT.println("turn timed out");
      break;
    }

    // Fast read inside the loop
    float currentYaw = getYaw(15);
    float error = targetYaw - currentYaw;

    // Shortest path calculation (guarantees error is -180 to +180)
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // Break condition
    if (abs(error) <= turnThreshold) break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < DT_FLOOR) dt = DT_FLOOR;

    if (fabsf(error) < 5.0f) {
      integral = 0.0f;
    } else {
      integral += error * dt;
      integral = constrain(integral, -30.0f, 30.0f);
    }

    float raw_deriv = (error - previousError) / dt;
    _deriv_filtered = DERIV_ALPHA * raw_deriv
                      + (1.0f - DERIV_ALPHA) * _deriv_filtered;
    previousError = error;

    // ── PID output ────────────────────────────────────────────────────
    float output = Kp_turn * error
                   + Ki_turn * integral
                   + Kd_turn * _deriv_filtered;

    int pwm = constrain(abs(output), lower, upper);

    if (output > 0) {
      motor1Reverse(pwm);
      motor2Forward(pwm * MOTOR_BIAS_TURN);
    } else {
      motor1Forward(pwm);
      motor2Reverse(pwm * MOTOR_BIAS_TURN);
    }

    delay(5);
  }

  motorStop();
}


// Wrapper functions connecting PID logic to your motor functions
// Assuming Motor 1 = Left, Motor 2 = Right


void applyPIDCentering(int rawLeft, int rawRight, int currentBaseSpeed) {

  static float lastError = 0.0f;
  static float integral = 0.0f;
  static bool wasUsingIMU = false;
  static unsigned long lastTime = millis();

  const float IMU_SCALE = 5.0f;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastTime = now;

  // A. Determine which walls are present
  bool hasLeftWall = (rawLeft > LEFT_WALL_THRESHOLD);
  bool hasRightWall = (rawRight > RIGHT_WALL_THRESHOLD);

  // Flag to check if we are flying blind this loop
  bool usingIMU = (!hasLeftWall && !hasRightWall);

  // B. State Transition Check: Reset Integral!
  if (usingIMU != wasUsingIMU) {
    integral = 0.0f;
  }
  wasUsingIMU = usingIMU;

  // C. Normalize the readings
  int leftNormalized = map(rawLeft, LEFT_SETPOINT, LEFT_MAX, 50, 100);
  int rightNormalized = map(rawRight, RIGHT_SETPOINT, RIGHT_MAX, 50, 100);

  // D. Calculate the Steering Error
  float error = 0.0f;

  if (hasLeftWall && hasRightWall) {
    error = (float)(leftNormalized - rightNormalized);
  } else if (hasLeftWall && !hasRightWall) {
    error = (leftNormalized - 50.0f);
  } else if (hasRightWall && !hasLeftWall) {
    error = (50.0f - rightNormalized);
  } else {
    float currentYaw = getYaw(2);
    float headingError = targetYaw - currentYaw;

    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;

    error = -headingError * IMU_SCALE;
  }

  // E. Smart Integral Logic (Anti-Windup)
  const float INTEGRATION_BAND = 25.0f;

  if (abs(error) < INTEGRATION_BAND) {
    integral += error * dt;
  } else {
    // If we are way off course, gently bleed the integral back to zero
    integral *= 0.95f;
  }

  // Hard clamp on the integral output to prevent it from dominating the motors
  float maxIntegralInfluence = 30.0f;
  float i_term = Ki * integral;
  i_term = constrain(i_term, -maxIntegralInfluence, maxIntegralInfluence);

  // F. Perform the PID Math
  float P = Kp * error;
  float D = Kd * ((error - lastError) / dt);
  lastError = error;

  float turnAdjustment = P + i_term + D;
  turnAdjustment = constrain(turnAdjustment, -75, 75);

  // G. Apply adjustment to the motors using the dynamic speed
  int leftSpeed = currentBaseSpeed + turnAdjustment;
  int rightSpeed = currentBaseSpeed - turnAdjustment;

  if (isWallFrontCollision()) {
    BT.println("Front wall detected in PID loop");
    motorStop();
    return;
  }  // check for walls before giving PWM

  // Constrain based on the current profile speed, not a hardcoded value
  leftSpeed = constrain(leftSpeed, currentBaseSpeed - 40, currentBaseSpeed + 40);
  rightSpeed = constrain(rightSpeed, currentBaseSpeed - 40, currentBaseSpeed + 40);

  // Motors are driven purely by the dynamic speed and PID output
  motor1Forward(leftSpeed);
  motor2Forward(rightSpeed);
}

// Add a default parameter 'stopAtEnd'
void centerUntilDistance(float dist, bool stopAtEnd = true) {
  // Capture this once. isStopped gets set to false inside the loop,
  // so reading it live would end the accel ramp after the first iteration.
  const bool startedFromStop = isStopped;


  long startTicks = (leftEncoderTicks + rightEncoderTicks) / 2;
  long totalTargetTicks = dist * (TICKS_PER_CM_LEFT + TICKS_PER_CM_RIGHT) / 2.0;
  long TARGET_TICKS = startTicks + totalTargetTicks;

  // --- TRAPEZOIDAL PROFILE SETUP ---
  long moveAccelTicks = startedFromStop ? ACCEL_TICKS : 0;
  long moveDecelTicks = stopAtEnd ? DECEL_TICKS : 0;
  int moveMaxSpeed = MAX_BASE_SPEED;

  // Short move: ramps would eat the whole distance (triangle).
  // Instead, cap the ramps at a fraction of the move so a cruise
  // plateau always remains, and lower the peak speed so the
  // acceleration slope stays the same.
  const float MAX_RAMP_FRACTION = 0.6f;  // ramps use at most 60% of the move
  long rampTicks = moveAccelTicks + moveDecelTicks;
  long maxRampTicks = (long)(totalTargetTicks * MAX_RAMP_FRACTION);

  if (rampTicks > maxRampTicks && rampTicks > 0) {
    float scale = (float)maxRampTicks / (float)rampTicks;
    moveAccelTicks = (long)(moveAccelTicks * scale);
    moveDecelTicks = (long)(moveDecelTicks * scale);
    moveMaxSpeed = MIN_BASE_SPEED_START + (int)((MAX_BASE_SPEED - MIN_BASE_SPEED_START) * scale);
  }

  while (true) {
    long currentTicks = (leftEncoderTicks + rightEncoderTicks) / 2;
    long ticksMoved = currentTicks - startTicks;
    long ticksRemaining = TARGET_TICKS - currentTicks;

    if (ticksRemaining > 0 && !isWallFrontCollision()) {

      // Start at cruise speed (the flat top of the trapezoid)
      long dynamicBaseSpeed = moveMaxSpeed;

      // 1. ACCELERATION RAMP
      if (moveAccelTicks > 0 && ticksMoved < moveAccelTicks) {
        long accelSpeed = map(ticksMoved, 0, moveAccelTicks, MIN_BASE_SPEED_START, moveMaxSpeed);
        if (accelSpeed < dynamicBaseSpeed) dynamicBaseSpeed = accelSpeed;
      }

      // 2. DECELERATION RAMP
      if (moveDecelTicks > 0 && ticksRemaining < moveDecelTicks) {
        long decelSpeed = map(ticksRemaining, moveDecelTicks, 0, moveMaxSpeed, MIN_BASE_SPEED_STOP);
        if (decelSpeed < dynamicBaseSpeed) dynamicBaseSpeed = decelSpeed;
      }

      // Safeguard
      dynamicBaseSpeed = constrain(dynamicBaseSpeed, MIN_BASE_SPEED_START, moveMaxSpeed);

      isStopped = false;  // We are moving
      float rawL = getCorrectedReading(3);
      float rawR = getCorrectedReading(1);
      applyPIDCentering(rawL, rawR, (int)dynamicBaseSpeed);
    } else {
      if (!wallFollower && stopAtEnd) {
        motorStop();
      }
      break;
    }
    delay(2);
  }

}

void centerUntilWall() {
  while (true) {
    if (isWallRight() && !isWallFrontCollision()) {
      int currentLeft = getCorrectedReading(3);
      int currentRight = getCorrectedReading(1);

      // FIX: Added MAX_BASE_SPEED as the third argument
      applyPIDCentering(currentLeft, currentRight, MAX_BASE_SPEED);
    } else {
      motorStop();  // Good practice to ensure it stops when a wall is detected
      return;
    }
  }
}
void centerUntilDistanceWallFollow(float dist){
  long currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
  long TARGET_TICKS = dist * ((TICKS_PER_CM_LEFT + TICKS_PER_CM_RIGHT)/2.0) + currentTicks;
  while(true){
    currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
    if(currentTicks < TARGET_TICKS&& !isWallFrontCollision()){
      int currentLeft = getCorrectedReading(3);
      int currentRight = getCorrectedReading(1); 
      applyPIDCentering(currentLeft,currentRight,MAX_BASE_SPEED); 
    }
    else{
      if(!wallFollower){motorStop();}
      BT.print("KP value: ");BT.println(Kp);
      BT.print("KD value: ");BT.println(Kd); 
      break; 
    }
    delay(2);
  }
}
