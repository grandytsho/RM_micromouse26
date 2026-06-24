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

  float startTime = millis();

  BT.println("turning");
  BT.print("Kp|"); BT.println(Kp_turn);
  BT.print("Kd|"); BT.println(Kd_turn);
  BT.print("Upper|"); BT.println(upper);
  BT.print("Lower|"); BT.println(lower); 

  float integral = 0.0f;
  float previousError = 0.0f;
  _deriv_filtered     = 0.0f;

  unsigned long lastTime = millis();

  // Fast read: 2ms timeout prevents blocking
  float startYaw = getYaw(2); 
  delay(15);

  // A positive angleDeg turns right, negative turns left
  targetYaw = startYaw + angleDeg;

  // STRICT NORMALIZATION: Wrap target directly to the IMU's native -180 to +180 range
  while (targetYaw > 180.0f) targetYaw -= 360.0f;
  while (targetYaw <= -180.0f) targetYaw += 360.0f;

  while (true) {
    if (millis() - startTime > 3000UL){ 
      BT.println("turn timed out");
      break;
    }
    
    // Fast read inside the loop
    float currentYaw = getYaw(2); 
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
        integral  = constrain(integral, -30.0f, 30.0f);
    }

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


void applyPIDCentering(int rawLeft, int rawRight) {
  
  static float lastError = 0.0f;
  static float integral = 0.0f;
  static bool wasUsingIMU = false; 
  static unsigned long lastTime = millis();

  const int BASE_SPEED = 100; 
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
  } 
  else if (hasLeftWall && !hasRightWall) {
    error = (leftNormalized - 50.0f);
  } 
  else if (hasRightWall && !hasLeftWall) {
    error = (50.0f - rightNormalized); 
  } 
  else {
    float currentYaw = getYaw(2);
    float headingError = targetYaw - currentYaw;
    
    if (headingError >  180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;
    
    error = headingError * IMU_SCALE;
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

  // G. Apply adjustment to the motors
  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;
  
  leftSpeed = constrain(leftSpeed, 60, 140);
  rightSpeed = constrain(rightSpeed, 60, 140);

  // Motors are driven purely by the baseline speed and PID output
  motor1Forward(leftSpeed);
  motor2Forward(rightSpeed);
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
    delay(2);
  }
}

// --- PID GAINS ---
// Outer Loop: Position to Steering Angle (TINY values for raw ADC)
float Kp_pos = 0.015;  
float Kd_pos = 0.05;   

// Inner Loop: Heading Angle to Motor PWM (LARGER values for IMU)
float Kp_head = 4.0;
float Kd_head = 0.5;

void applyPIDCenteringCascaded(int rawLeft, int rawRight) {
  static float lastPosError = 0.0f;
  static float lastHeadingError = 0.0f;
  
  const int BASE_SPEED = 100;
  
  bool hasLeftWall = (rawLeft > LEFT_WALL_THRESHOLD);
  bool hasRightWall = (rawRight > RIGHT_WALL_THRESHOLD);

  // ==========================================
  // 1. OUTER LOOP: Positional Error
  // ==========================================
  float posError = 0.0f;

  if (hasLeftWall && hasRightWall) {
    // Both Walls: Balance the raw ADC deviance
    float leftDev = rawLeft - LEFT_SETPOINT;
    float rightDev = rawRight - RIGHT_SETPOINT;
    posError = leftDev - rightDev; // Positive = drifting left
  } 
  else if (hasLeftWall && !hasRightWall) {
    // Left Wall Only
    posError = rawLeft - LEFT_SETPOINT;
  } 
  else if (hasRightWall && !hasLeftWall) {
    // Right Wall Only (Negate so drifting right yields negative error)
    posError = -(rawRight - RIGHT_SETPOINT);
  } 
  else {
    // No Walls: Trust the IMU
    posError = 0.0f;
  }

  // Calculate steering correction (Degrees)
  float posDerivative = posError - lastPosError;
  lastPosError = posError;
  
  float steeringCorrection = (Kp_pos * posError) + (Kd_pos * posDerivative);
  steeringCorrection = constrain(steeringCorrection, -15.0f, 15.0f);
  
  // Calculate dynamic target heading
  float dynamicTargetYaw = targetYaw + steeringCorrection;

  // ==========================================
  // 2. INNER LOOP: Heading Error
  // ==========================================
  // Fast 2ms read to prevent loop stalling
  float currentYaw = getYaw(2); 
  
  float headingError = dynamicTargetYaw - currentYaw;
  
  // Shortest path logic (-180 to +180)
  if (headingError >  180.0f) headingError -= 360.0f;
  if (headingError < -180.0f) headingError += 360.0f;

  float headingDerivative = headingError - lastHeadingError;
  lastHeadingError = headingError;

  // Calculate Motor PWM Adjustment
  float turnAdjustment = (Kp_head * headingError) + (Kd_head * headingDerivative);
  turnAdjustment = constrain(turnAdjustment, -75, 75);

  // ==========================================
  // 3. APPLY TO MOTORS
  // ==========================================
  // Positive turnAdjustment = Steer Right (Left speeds up, Right slows down)
  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;
  
  // Clamp to prevent stalling or PWM overflow
  leftSpeed = constrain(leftSpeed, 40, 160);
  rightSpeed = constrain(rightSpeed, 40, 160);

  // Apply to TB6612FNG (handles reverse if sharply correcting)
  if(leftSpeed > 0) motor1Forward(leftSpeed); else motor1Reverse(-leftSpeed);
  if(rightSpeed > 0) motor2Forward(rightSpeed); else motor2Reverse(-rightSpeed);
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