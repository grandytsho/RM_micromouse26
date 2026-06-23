#include "one.h"
#include "two.h"
#include <Arduino.h>

float Kp = 0.5;
float Kd = 0.15; 
float Ki = 0.0;

float Kp_turn = 1.2;
float Ki_turn = 0.0;
float Kd_turn = 0.015;

float targetYaw = 0;
int TURN_THRESHOLD = 10; 
const float IMU_SCALE        = 1.0f;  // 1° heading drift → 2 normalized units; tune this
const float BOTH_WALL_BLEND  = 0.85f; // weight given to walls when both present
const float ONE_WALL_BLEND   = 0.60f;

//centering and wall thresholds
int LEFT_SETPOINT = 460;
int LEFT_MAX = 800;
int RIGHT_SETPOINT = 429;
int RIGHT_MAX = 800;
int LEFT_WALL_THRESHOLD = 360; 
int RIGHT_WALL_THRESHOLD = 320;
int FRONT_WALL_THRESHOLD = 329;
int FRONT2_WALL_THRESHOLD = 365;

int FRONT_MAX = 0; 
int FRONT2_MAX = 0; 

static float _deriv_filtered = 0.0f;
const float TICKS_PER_CM_LEFT = TICKS_PER_REVOLUTION_LEFT/WHEEL_CIRCUMFERENCE;
const float TICKS_PER_CM_RIGHT = TICKS_PER_REVOLUTION_RIGHT/WHEEL_CIRCUMFERENCE;


// Motor Control Functions
void motor1Forward(int speed) {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_PWM, speed);
}

void motor1Reverse(int speed) {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  // No PWM inversion needed for the TB6612!
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
  // No PWM inversion needed for the TB6612!
  analogWrite(M2_PWM, speed);
}

void motorStop() {
  // 1. Hard Brake Phase (TB6612 short brake = both IN pins HIGH)
  // digitalWrite(M1_IN1, HIGH);
  // digitalWrite(M1_IN2, HIGH);
  // analogWrite(M1_PWM, 0); 
  
  // digitalWrite(M2_IN1, HIGH);
  // digitalWrite(M2_IN2, HIGH);
  // analogWrite(M2_PWM, 0);
  
  delay(150); // Hold the brake for 150ms just like your old code
  
  // 2. Coast / Standby Phase (TB6612 coast = both IN pins LOW)
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}
//wall detection functions
bool isWallLeft(){
  int raw = getCorrectedReading(3);
  return (raw > LEFT_WALL_THRESHOLD);
}
bool isWallRight(){
  int raw = getCorrectedReading(1);
  return (raw > RIGHT_WALL_THRESHOLD);
}

bool isWallFront(){
  int front = getCorrectedReading(0);
  int front2 = getCorrectedReading(5);
  return ((front > FRONT_WALL_THRESHOLD)&&(front2 > FRONT_WALL_THRESHOLD));
}
void turn(float angleDeg){
  const int   upper         = 110;
  const int   lower         = 20;
  const float turnThreshold = 0.25f;   // degrees — slightly relaxed for stability
  const float DERIV_ALPHA   = 0.25f;   // EMA weight for new sample (lower = smoother)
  const float DT_FLOOR      = 0.004f;  // FIX 1: 4 ms floor matches delay(5) reality

  float startTime= millis();

  BT.println("turning");
  BT.print("Kp|");BT.println(Kp_turn);
  BT.print("Kd|");BT.println(Kd_turn);
  BT.print("Upper|");BT.println(upper);
  BT.print("Lower|");BT.println(lower); 

  float integral = 0.0f;
  float previousError = 0.0f;
  _deriv_filtered     = 0.0f;

  unsigned long lastTime = millis();

  float startYaw = getYaw(15); // Will return 0 to 360
  delay(15);

  // A positive angleDeg turns right, negative turns left
  float targetYaw = startYaw + angleDeg;

  // NEW: Normalize target strictly to 0 to 360
  while (targetYaw >= 360.0f) targetYaw -= 360.0f;
  while (targetYaw < 0.0f) targetYaw += 360.0f;

  while (true) {
    if (millis() - startTime > 3000UL){ 
      BT.println("turn timed out");
      break;
    }
    float currentYaw = getYaw(15); 
    float error = targetYaw - currentYaw;

    // It guarantees the error is always the shortest path (-180 to +180)
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // Break condition
    if (abs(error) <= turnThreshold) break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
        if (dt < DT_FLOOR) dt = DT_FLOOR;

        // ── integral with anti-windup clamp ───────────────────────────────
        // FIX 6: reset integral when close to target to prevent overshoot
        // from accumulated wind-up during the approach.
        if (fabsf(error) < 5.0f) {
            integral = 0.0f;
        } else {
            integral += error * dt;
            integral  = constrain(integral, -30.0f, 30.0f);
        }

        // ── derivative with EMA low-pass filter ───────────────────────────
        // FIX 2: raw derivative from IMU noise causes spikes.  An
        // exponential moving average damps high-frequency jitter while
        // still reacting to real angular-velocity changes.
        float raw_deriv   = (error - previousError) / dt;
        _deriv_filtered   = DERIV_ALPHA * raw_deriv
                          + (1.0f - DERIV_ALPHA) * _deriv_filtered;
        previousError = error;

        // ── PID output ────────────────────────────────────────────────────
        float output = Kp_turn * error
                     + Ki_turn * integral
                     + Kd_turn * _deriv_filtered;

    int pwm = constrain(abs(output), lower, upper);

    if (output > 0) {
      motor1Reverse(pwm);   
      motor2Forward(pwm*MOTOR_BIAS_TURN);  
    } else {
      motor1Forward(pwm);  
      motor2Reverse(pwm*MOTOR_BIAS_TURN);   
    }

    delay(5);
  }

 motorStop();
}


// Wrapper functions connecting PID logic to your motor functions
// Assuming Motor 1 = Left, Motor 2 = Right


void applyPIDCentering(int rawLeft, int rawRight) {
  
  static float lastError = 0;
  float integral = 0;

  const int BASE_SPEED = 100; 
  
  // A. Determine which walls are present
  bool hasLeftWall = (rawLeft > LEFT_WALL_THRESHOLD);
  bool hasRightWall = (rawRight > RIGHT_WALL_THRESHOLD);

  BT.print("right raw");BT.print(rawRight);BT.print("left raw");BT.println(rawLeft);
  BT.print("right wall");BT.print(hasLeftWall);BT.print("Left wall");BT.println(hasRightWall);
  // B. Normalize the readings using your calibration data
  int leftNormalized = map(rawLeft, LEFT_SETPOINT, LEFT_MAX, 50, 100);
  int rightNormalized = map(rawRight, RIGHT_SETPOINT, RIGHT_MAX, 50, 100);

  BT.print("Left");BT.print(leftNormalized);BT.print("|right");BT.println(rightNormalized);
  // C. Calculate the Steering Error
  float error = 0.0f;
  float currentYaw   = getYaw(15);
  float wallError =0.0f;

  float headingError = targetYaw - currentYaw;
  if (headingError >  180.0f) headingError -= 360.0f;
  if (headingError < -180.0f) headingError += 360.0f;
  float imuError = headingError * IMU_SCALE;

  

  if (hasLeftWall && hasRightWall) {
    wallError = (float)(leftNormalized - rightNormalized);
    error     = wallError * BOTH_WALL_BLEND + imuError * (1.0f - BOTH_WALL_BLEND);
    BT.print("Wall Error: "); BT.print(wallError); BT.print("|Error: ");BT.println(error);
  } 
  else if (hasLeftWall && !hasRightWall) {
    wallError = (leftNormalized - 50);
    error     = wallError * ONE_WALL_BLEND + imuError * (1.0f - ONE_WALL_BLEND);
    BT.print("Wall Error: "); BT.print(wallError); BT.print("|Error: ");BT.println(error);
  } 
  else if (hasRightWall && !hasLeftWall) {
    wallError = (50 - rightNormalized); 
    error     = wallError * ONE_WALL_BLEND+ imuError * (1.0f - ONE_WALL_BLEND);
    BT.print("Wall Error: "); BT.print(wallError); BT.print("|Error: ");BT.println(error); 
  } 
  else {
    error = 0;//imuError*2;
  }

  // D. Perform the PID Math
  float P = error;
  //integral = integral + error;
  float D = error - lastError;

  float turnAdjustment = (Kp * P) + (Ki * integral) + (Kd * D);
  lastError = error;
  turnAdjustment = constrain(turnAdjustment,-75, 75);
  BT.print("turn Adjustment:");BT.println(turnAdjustment);
  // E. Apply adjustment to the motors
  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;
  BT.print("right speed");BT.print(rightSpeed);BT.print("Left Speed");BT.println(leftSpeed);
  // Allow values down to -255 so the mouse can reverse a wheel to steer sharply if needed
  leftSpeed = constrain(leftSpeed, 60, 140);
  rightSpeed = constrain(rightSpeed, 60, 140);

  if(hasLeftWall && hasRightWall){
    motor1Forward(leftSpeed*1);
    motor2Forward(rightSpeed);
  }
  if(hasLeftWall && !hasRightWall){
    motor1Forward(leftSpeed*1);
    motor2Forward(rightSpeed);
  }
  if(!hasLeftWall && hasRightWall){
    motor1Forward(leftSpeed*MOTOR_BIAS);
    motor2Forward(rightSpeed);
  }
  if(!hasLeftWall && !hasRightWall){
    motor1Forward(leftSpeed*MOTOR_BIAS);
    motor2Forward(rightSpeed);
  }
}

void centerUntilDistance(float dist){
  long currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
  long TARGET_TICKS = dist * ((TICKS_PER_CM_LEFT + TICKS_PER_CM_RIGHT)/2.0) + currentTicks;
  while(true){
    currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
    if(currentTicks < TARGET_TICKS){
      int currentLeft = getCorrectedReading(3);
      int currentRight = getCorrectedReading(1); 
      applyPIDCentering(currentLeft,currentRight); 
    }
    else{
      motorStop();
      BT.print("KP value: ");BT.println(Kp);
      BT.print("KD value: ");BT.println(Kd); 
      break; 
    }
    delay(5);
  }
}

// void squareUp(int rawFront, int rawFront2){
//   int frontNormalized = map(rawLeft, FRONT_WALL_THRESHOLD, FRONT_MAX, 50, 100);
//   int front2Normalized = map(rawRight, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100);

//   float error = frontNormalised - front2Normalised;
//   static float lastError = 0; 
//   float derivative  = error - lastError;
//   lastError = error; 
//   float turnAdjustment = Kp*error+ Kd*derivative; 

//   motor1Forward(-turnAdjustment); 
//   motor2Forward(turnAdjustment);

// }