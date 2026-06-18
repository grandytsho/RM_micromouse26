#include "one.h"
#include "two.h"
#include <Arduino.h>

float Kp = 1.2;
float Kd = 3.5; 
float Ki = 0.0;
int LEFT_SETPOINT = 294;
int LEFT_MAX = 880;

int RIGHT_SETPOINT = 215;
int RIGHT_MAX = 980;

int LEFT_WALL_THRESHOLD = 140;  
int RIGHT_WALL_THRESHOLD = 150; 

float targetYaw = 0;
int TURN_THRESHOLD = 10; 
const float IMU_SCALE        = 1.0f;  // 1° heading drift → 2 normalized units; tune this
const float BOTH_WALL_BLEND  = 0.85f; // weight given to walls when both present
const float ONE_WALL_BLEND   = 0.60f;

// Motor Control Functions
void motor1Forward(int speed) {

  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, speed*MOTOR_BIAS);
}
void motor1Reverse(int speed) {
  digitalWrite(M1_IN1, HIGH);
  // Invert the PWM because IN1 is HIGH
  int mappedSpeed = 255 - (speed * MOTOR_BIAS);
  // Prevent negative values if speed * MOTOR_BIAS exceeds 255
  if (mappedSpeed < 0) mappedSpeed = 0; 
  analogWrite(M1_IN2, mappedSpeed);
}
void motor2Forward(int speed) {
  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, speed);
}
void motor2Reverse(int speed) {
  digitalWrite(M2_IN1, HIGH);
  // Invert the PWM because IN1 is HIGH
  analogWrite(M2_IN2, 255 - speed);
}
void motorStop() {
  // digitalWrite(M2_IN1, HIGH);
  // analogWrite(M2_IN2, 255);
  // digitalWrite(M1_IN1, HIGH);
  // analogWrite(M1_IN2, 255);
  delay(150);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 0);
  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 0);

}
bool isWallLeft(){
  int rawLeft = getCorrectedReading(3);
  return (rawLeft > LEFT_WALL_THRESHOLD);
}
bool isWallRight(){
  int rawRight = getCorrectedReading(1);
  return (rawRight > RIGHT_WALL_THRESHOLD);
}
bool isWallFront(){
  return false;
}
void turn(float angleDeg){
  float Kp = 1.0;
  float Ki = 0.0;
  float Kd = 0.03;
  int upper = 110;
  int lower = 40; 
  float startTime= millis();
  BT.println("turning");
  BT.print("Kp|");BT.println(Kp);
  BT.print("Kd|");BT.println(Kd);
  BT.print("Upper|");BT.println(upper);
  BT.print("Lower|");BT.println(lower); 
  float integral = 0.0f;
  float previousError = 0.0f;
  unsigned long lastTime = millis();

  float startYaw = getYaw(); // Will return 0 to 360
  delay(15);

  // A positive angleDeg turns right, negative turns left
  targetYaw = startYaw + angleDeg;

  // NEW: Normalize target strictly to 0 to 360
  while (targetYaw >= 360.0f) targetYaw -= 360.0f;
  while (targetYaw < 0.0f) targetYaw += 360.0f;

  while (true) {
    if (millis() - startTime > 3000){ 
      BT.println("turn timed out");
      break;
    }
    float turnThreshold = 0.25;
    float currentYaw = getYaw(); 
    float error = targetYaw - currentYaw;

    // It guarantees the error is always the shortest path (-180 to +180)
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // Break condition
    if (abs(error) <= turnThreshold) break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < 0.001f) dt = 0.001f;

    integral += error * dt;
    integral = constrain(integral, -50.0f, 50.0f);

    float derivative = (error - previousError) / dt;
    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int pwm = constrain(abs(output), lower, upper);

    if (output > 0) {
      motor2Reverse(pwm);   
      motor1Forward(pwm);  
    } else {
      motor2Forward(pwm);  
      motor1Reverse(pwm);   
    }

    delay(5);
  }

 motorStop();
}

float lastError = 0;
void applyPIDCentering(int rawLeft, int rawRight) {
  
  float lastError = 0;
  float integral = 0;

  const int BASE_SPEED = 110; 
  
  // A. Determine which walls are present
  bool hasLeftWall = isWallLeft();
  bool hasRightWall = isWallRight();
  BT.print("right raw");BT.print(rawRight);BT.print("left raw");BT.println(rawLeft);
  BT.print("right wall");BT.print(hasLeftWall);BT.print("Left wall");BT.println(hasRightWall);

  // B. Normalize the readings using your calibration data
    // Clamp before map() to prevent out-of-range outputs
  int leftNormalized  = map(constrain(rawLeft,  LEFT_SETPOINT,  LEFT_MAX),
                            LEFT_SETPOINT,  LEFT_MAX,  50, 100);
  int rightNormalized = map(constrain(rawRight, RIGHT_SETPOINT, RIGHT_MAX),
                            RIGHT_SETPOINT, RIGHT_MAX, 50, 100);
  BT.print("Left");BT.print(leftNormalized);BT.print("|right");BT.println(rightNormalized);

  float currentYaw   = getYaw();
  float headingError = targetYaw - currentYaw;
  if (headingError >  180.0f) headingError -= 360.0f;
  if (headingError < -180.0f) headingError += 360.0f;
  float imuError = 0.0f;//headingError * IMU_SCALE;

  float error = 0.0f;
  float wallError =0.0f;

  if (hasLeftWall && hasRightWall) {
    wallError = (float)(leftNormalized - rightNormalized);
    error     = wallError * BOTH_WALL_BLEND + imuError * (1.0f - BOTH_WALL_BLEND);
  } 
  else if (hasLeftWall && !hasRightWall) {
    wallError = leftNormalized - 50;
    error     = wallError * ONE_WALL_BLEND + imuError * (1.0f - ONE_WALL_BLEND);
  } 
  else if (hasRightWall && !hasLeftWall) {
    wallError = 50 - rightNormalized; 
    error     = wallError * ONE_WALL_BLEND + imuError * (1.0f - ONE_WALL_BLEND);
  }
  else {
    error = imuError*2;
  }

  // D. Perform the PID Math
  float P = error;
  //integral = integral + error;
  float D = error - lastError;

  static float turnAdjustment = (Kp * P) + (Ki * integral) + (Kd * D);
  lastError = error;
  turnAdjustment = constrain(turnAdjustment,-75, 75);
  BT.print("turn Adjustment:");BT.println(turnAdjustment);
  // E. Apply adjustment to the motors
  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;
  BT.print("yaw error");BT.println(imuError);
  BT.print("right speed");BT.print(rightSpeed);BT.print("Left Speed");BT.println(leftSpeed);
  // Allow values down to -255 so the mouse can reverse a wheel to steer sharply if needed
  leftSpeed = constrain(leftSpeed, 40, 150);
  rightSpeed = constrain(rightSpeed, 40, 150);

  motor1Forward(leftSpeed);
  motor2Forward(rightSpeed);
}

void centerUntilDistance(float dist){
  long currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
  long TARGET_TICKS = dist * TICKS_PER_CM + currentTicks;
  while(true){
    currentTicks = (leftEncoderTicks + rightEncoderTicks)/2;
    if(currentTicks < TARGET_TICKS){
      int currentLeft = getCorrectedReading(3);
      int currentRight = getCorrectedReading(1); 
      applyPIDCentering(currentLeft,currentRight); 
    }
    else{
      motorStop(); 
      break; 
    }
    delay(5);
  }
}