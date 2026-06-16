#include "one.h"
#include "two.h"
#include <Arduino.h>
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.05;
int upper = 110;
int lower = 40; 

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
  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_IN2, 255);
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_IN2, 255);
  delay(150);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 0);
  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 0);

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