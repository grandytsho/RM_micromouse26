#include "one.h"
#include "two.h"
#include <Arduino.h>
#include <cmath>

float Kp = 0.5;
float Kd = 0.15; 
float Ki = 0.0;

float Kp_turn = 1.2;
float Ki_turn = 0.0;
float Kd_turn = 0.015;

float Kp_align =0.5;
float Kd_align = 0.01; 

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
int FRONT_WALL_THRESHOLD = 330;
int FRONT2_WALL_THRESHOLD = 262;

int FRONT_MAX = 392; 
int FRONT2_MAX = 300; 

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

void motor1Wraper(int speed){
  if(speed < 0) motor1Reverse(abs(speed));
  else motor1Forward(speed);
}

void motor2Wraper(int speed){
  if(speed < 0) motor2Reverse(abs(speed));
  else motor2Forward(speed);
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
  const int   lower         = 23;
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

void centerUntilWall(){

  if(isWallRight()&&!isWallFront()){
    int currentLeft = getCorrectedReading(3);
    int currentRight = getCorrectedReading(1); 
    applyPIDCentering(currentLeft,currentRight);
  }
  else if(!isWallFront()){
    int currentLeft = getCorrectedReading(3);
    int currentRight = getCorrectedReading(1); 
    applyPIDCentering(currentLeft,currentRight);
  }
  else return;
}


void alignToFrontWall() {
  unsigned long alignedStartTime = 0;
  bool isFullyAligned = false;

  // --- Tuning Constants ---
  // Adjust these to change how close it gets and how strict the angle is
  const int TARGET_PROXIMITY = 100; // Distance to stop at (on your 50-100 mapped scale)
  const float TARGET_ERROR = 2.0f; // Maximum allowed angle discrepancy
  const int SETTLE_TIME_MS = 100;  // How long it must hold the perfect pose

  while (!isFullyAligned) {
    // 1. Grab fresh sensor data
    int rawFront = getCorrectedReading(0);
    int rawFront2 = getCorrectedReading(5);

    int frontNormalized = map(rawFront, FRONT_WALL_THRESHOLD, FRONT_MAX, 50, 100);
    int front2Normalized = map(rawFront2, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100);

    int avgProximity = (frontNormalized + front2Normalized) / 2;
    float error = (float)(frontNormalized - front2Normalized);

    // 2. Check if we are at the target distance AND perfectly straight
    if (avgProximity >= TARGET_PROXIMITY && abs(error) <= TARGET_ERROR) {
      
      // If this is the first time we hit the target, start the timer
      if (alignedStartTime == 0) {
        alignedStartTime = millis(); 
      } 
      // If we've held this perfect alignment for 100ms, we are settled
      else if (millis() - alignedStartTime > SETTLE_TIME_MS) {
        isFullyAligned = true; 
        break; 
      }
    } 
    else {
      // If the robot slips or overshoots, reset the timer
      alignedStartTime = 0; 
    }

    // 3. Call the squaring logic to keep driving/pivoting
    approachAndSquareUp(rawFront, rawFront2);
    delay(2); 
  }
  motorStop();
  delay(150); 
}


void approachAndSquareUp(int rawFront, int rawFront2) {
  // 1. Time delta for a stable derivative
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastTime = now;

  BT.print("rawFront");BT.println(rawFront);
  BT.print("rawFront2");BT.println(rawFront2);

  // 2. Map sensor readings to your normalized 50-100 scale
  int frontNormalized = map(rawFront, FRONT_WALL_THRESHOLD, FRONT_MAX, 50, 100);
  int front2Normalized = map(rawFront2, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100);

  BT.print("front1:");BT.println(frontNormalized);
  BT.print("front2:");BT.println(front2Normalized);

  // 3. Calculate average proximity to the wall (higher = closer)
  int avgProximity = (frontNormalized + front2Normalized) / 2;
  BT.print("avgProximity:");BT.println(avgProximity);

  // 4. Calculate alignment error
  float error = (float)(frontNormalized - front2Normalized);
  BT.print("error:");BT.println(error);

  static float lastError = 0.0f; 
  float derivative = (error - lastError) / dt;
  lastError = error; 
  
  float turnAdjustment = (Kp_align * error) + (Kd_align* derivative); 
  BT.print("turnAdjustment:");BT.println(turnAdjustment);
  // 5. Determine Forward Speed based on distance
  int baseSpeed = 40; // Slow, controlled approach speed
  
  // If we reach the perfect calibrated distance (100), stop moving forward
  // and let the turnAdjustment finish squaring up in place.
  if (avgProximity >= 100) {
    baseSpeed = 0; 
  }

  // 6. Apply Differential Steering
  int leftSpeed = baseSpeed - turnAdjustment; 
  int rightSpeed = baseSpeed + turnAdjustment;
  
  // Clamp the motor outputs to prevent violent speed spikes
  // We allow negative speeds down to -50 so a wheel can reverse to pivot if needed
  leftSpeed = constrain(leftSpeed, -50, 80); 
  rightSpeed = constrain(rightSpeed, -50, 80);
  BT.print("leftSpeed:");BT.println(leftSpeed);
  BT.print("rightSpeed:");BT.println(rightSpeed);

  // 7. Safely command the motors using absolute values for reverse
  motor1Wraper(leftSpeed);
  motor2Wraper(rightSpeed);
}

// void squareUp(int rawFront, int rawFront2){
//   int frontNormalized = map(rawFront, FRONT_WALL_THRESHOLD, FRONT_MAX, 50, 100);
//   int front2Normalized = map(rawFront2, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100);

//   float error = frontNormalized - front2Normalized;
//   static float lastError = 0; 
//   float derivative  = error - lastError;
//   lastError = error; 
//   float turnAdjustment = Kp*error+ Kd*derivative; 

//   motor1Wraper(-turnAdjustment); 
//   motor2Wraper(turnAdjustment);
// }

// --- PID GAINS ---
// // Outer Loop: Position to Steering Angle (TINY values for raw ADC)
// float Kp_pos = 0.015;  
// float Kd_pos = 0.05;   

// // Inner Loop: Heading Angle to Motor PWM (LARGER values for IMU)
// float Kp_head = 4.0;
// float Kd_head = 0.5;

// void applyPIDCenteringCascaded(int rawLeft, int rawRight) {
//   static float lastPosError = 0.0f;
//   static float lastHeadingError = 0.0f;
  
//   const int BASE_SPEED = 100;
  
//   bool hasLeftWall = (rawLeft > LEFT_WALL_THRESHOLD);
//   bool hasRightWall = (rawRight > RIGHT_WALL_THRESHOLD);

//   // ==========================================
//   // 1. OUTER LOOP: Positional Error
//   // ==========================================
//   float posError = 0.0f;

//   if (hasLeftWall && hasRightWall) {
//     // Both Walls: Balance the raw ADC deviance
//     float leftDev = rawLeft - LEFT_SETPOINT;
//     float rightDev = rawRight - RIGHT_SETPOINT;
//     posError = leftDev - rightDev; // Positive = drifting left
//   } 
//   else if (hasLeftWall && !hasRightWall) {
//     // Left Wall Only
//     posError = rawLeft - LEFT_SETPOINT;
//   } 
//   else if (hasRightWall && !hasLeftWall) {
//     // Right Wall Only (Negate so drifting right yields negative error)
//     posError = -(rawRight - RIGHT_SETPOINT);
//   } 
//   else {
//     // No Walls: Trust the IMU
//     posError = 0.0f;
//   }

//   // Calculate steering correction (Degrees)
//   float posDerivative = posError - lastPosError;
//   lastPosError = posError;
  
//   float steeringCorrection = (Kp_pos * posError) + (Kd_pos * posDerivative);
//   steeringCorrection = constrain(steeringCorrection, -15.0f, 15.0f);
  
//   // Calculate dynamic target heading
//   float dynamicTargetYaw = targetYaw + steeringCorrection;

//   // ==========================================
//   // 2. INNER LOOP: Heading Error
//   // ==========================================
//   // Fast 2ms read to prevent loop stalling
//   float currentYaw = getYaw(2); 
  
//   float headingError = dynamicTargetYaw - currentYaw;
  
//   // Shortest path logic (-180 to +180)
//   if (headingError >  180.0f) headingError -= 360.0f;
//   if (headingError < -180.0f) headingError += 360.0f;

//   float headingDerivative = headingError - lastHeadingError;
//   lastHeadingError = headingError;

//   // Calculate Motor PWM Adjustment
//   float turnAdjustment = (Kp_head * headingError) + (Kd_head * headingDerivative);
//   turnAdjustment = constrain(turnAdjustment, -75, 75);

//   // ==========================================
//   // 3. APPLY TO MOTORS
//   // ==========================================
//   // Positive turnAdjustment = Steer Right (Left speeds up, Right slows down)
//   int leftSpeed = BASE_SPEED + turnAdjustment;
//   int rightSpeed = BASE_SPEED - turnAdjustment;
  
//   // Clamp to prevent stalling or PWM overflow
//   leftSpeed = constrain(leftSpeed, 40, 160);
//   rightSpeed = constrain(rightSpeed, 40, 160);

//   // Apply to TB6612FNG (handles reverse if sharply correcting)
//   if(leftSpeed > 0) motor1Forward(leftSpeed); else motor1Reverse(-leftSpeed);
//   if(rightSpeed > 0) motor2Forward(rightSpeed); else motor2Reverse(-rightSpeed);
// }