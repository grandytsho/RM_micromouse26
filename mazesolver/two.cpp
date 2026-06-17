#include "one.h"
#include "two.h"
#include <Arduino.h>

float Kp = 1.2;
float Kd = 3.5; 
float Ki = 0.0;
const int M1_DIR = M1_IN1; // Motor 1 (Left) Direction
const int M1_PWM = M1_IN2; // Motor 1 (Left) PWM
const int M2_DIR = M2_IN1; // Motor 2 (Right) Direction
const int M2_PWM = M2_IN2; // Motor 2 (Right) PWM

// Motor Control Functions
void motor1Forward(int speed) {

  digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, speed*MOTOR_BIAS);
}
void motor1Reverse(int speed) {
  digitalWrite(M1_DIR, HIGH);
  // Invert the PWM because IN1 is HIGH
  int mappedSpeed = 255 - (speed * MOTOR_BIAS);
  // Prevent negative values if speed * MOTOR_BIAS exceeds 255
  if (mappedSpeed < 0) mappedSpeed = 0; 
  analogWrite(M1_PWM, mappedSpeed);
}
void motor2Forward(int speed) {
  digitalWrite(M2_DIR, LOW);
  analogWrite(M2_PWM, speed);
}
void motor2Reverse(int speed) {
  digitalWrite(M2_DIR, HIGH);
  // Invert the PWM because IN1 is HIGH
  analogWrite(M2_PWM, 255 - speed);
}
void motorStop() {
  // digitalWrite(M2_DIR, HIGH);
  // analogWrite(M2_PWM, 255);
  // digitalWrite(M1_DIR, HIGH);
  // analogWrite(M1_PWM, 255);
  delay(150);
  digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_DIR, LOW);
  analogWrite(M2_PWM, 0);

}
//wall detection functions
bool isWallLeft(){
  return true;
}
bool isWallRight(){
  return true;
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
  float targetYaw = startYaw + angleDeg;

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

int LEFT_SETPOINT = 294;
int LEFT_MAX = 880;

int RIGHT_SETPOINT = 215;
int RIGHT_MAX = 980;
int LEFT_WALL_THRESHOLD = 140;  
int RIGHT_WALL_THRESHOLD = 150; 

int TURN_THRESHOLD = 10; 

// Wrapper functions connecting PID logic to your motor functions
// Assuming Motor 1 = Left, Motor 2 = Right
void setLeftMotorSpeed(int speed) {
  if (speed >= 0) {
    motor1Forward(speed);
  } else {
    motor1Reverse(-speed); // Pass positive magnitude to reverse function
  }
}

void setRightMotorSpeed(int speed) {
  if (speed >= 0) {
    motor2Forward(speed);
  } else {
    motor2Reverse(-speed); // Pass positive magnitude to reverse function
  }
}

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
  float error = 0;

  if (hasLeftWall && hasRightWall) {
    error = leftNormalized - rightNormalized;
  } 
  // else if (hasLeftWall && !hasRightWall) {
  //   error = leftNormalized - 50; 
  // } 
  // else if (hasRightWall && !hasLeftWall) {
  //   error = 50 - rightNormalized; 
  // } 
  // else {
  //   error = 0; 
  // }

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
  leftSpeed = constrain(leftSpeed, 40, 140);
  rightSpeed = constrain(rightSpeed, 40, 140);

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