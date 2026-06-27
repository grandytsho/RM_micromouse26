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

float Kp_align =0.4;
float Kd_align = 0.01; 

float targetYaw = 0;
int TURN_THRESHOLD = 10; 
const float IMU_SCALE        = 1.0f; 
const float BOTH_WALL_BLEND  = 0.85f;
const float ONE_WALL_BLEND   = 0.60f;

static unsigned long _approach_lastTime  = 0;
static float         _approach_lastError = NAN;
static float         _approach_derivFilt = 0.0f;

//centering and wall thresholds
int LEFT_SETPOINT = 460;
int LEFT_MAX = 868;
int RIGHT_SETPOINT = 469;
int RIGHT_MAX = 905;
int LEFT_WALL_THRESHOLD = 360; 
int RIGHT_WALL_THRESHOLD = 320;
int FRONT_WALL_THRESHOLD = 333;
int FRONT2_WALL_THRESHOLD = 246;

int FRONT_MAX = 416; 
int FRONT2_MAX = 576; 

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
  // 1. Hard Brake Phase (TB6612 short brake = both IN pins HIGH)
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, HIGH);
  analogWrite(M1_PWM, 0); 
  
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, HIGH);
  analogWrite(M2_PWM, 0);
  
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

  // G. Apply adjustment to the motors
  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;
  
  if(isWallFront()) 
  { BT.println("Front wall detected in PID loop"); 
    motorStop();return;} // check for walls before giving PWM

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
    if(currentTicks < TARGET_TICKS && !isWallFront()){
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
  while(true){
    if(isWallRight()&&!isWallFront()){
      int currentLeft = getCorrectedReading(3);
      int currentRight = getCorrectedReading(1); 
      applyPIDCentering(currentLeft,currentRight);
    }
    else return;
  }
}

// void alignToFrontWall() {
//   // Reset the PID memory at the start of every alignment session
//   // This completely prevents the derivative spike bug.
//   _approach_lastTime  = 0;
//   _approach_lastError = NAN;
//   _approach_derivFilt = 0.0f;

//   unsigned long alignedStartTime = 0;
//   unsigned long alignOverallTimeout = millis(); 
//   bool isFullyAligned = false;

//   // Changed to float to prevent decimal truncation
//   const float TARGET_PROXIMITY = 57.155f; 
//   const float TARGET_ERROR = 2.0f; 
//   const int SETTLE_TIME_MS = 100;  

//   while (!isFullyAligned) {
//     // 2.5-second safety timeout prevents infinite looping
//     if (millis() - alignOverallTimeout > 2500) {
//       BT.println("Align timed out");
//       break;
//     }

//     int rawFront = getCorrectedReading(0);
//     int rawFront2 = getCorrectedReading(5);

//     int frontNormalized = map(rawFront, FRONT_WALL_THRESHOLD, FRONT_MAX, 50, 100);
//     int front2Normalized = map(rawFront2, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100);

//     int avgProximity = (frontNormalized + front2Normalized) / 2;
//     float error = (float)(frontNormalized - front2Normalized);

//     // fabsf() ensures we don't truncate the float to an int during comparison
//     if (avgProximity >= TARGET_PROXIMITY && fabsf(error) <= TARGET_ERROR) {
//       if (alignedStartTime == 0) {
//         alignedStartTime = millis(); 
//       } 
//       else if (millis() - alignedStartTime > SETTLE_TIME_MS) {
//         isFullyAligned = true; 
//         break; 
//       }
//     } 
//     else {
//       alignedStartTime = 0; 
//     }

//     approachAndSquareUp(rawFront, rawFront2);
//     delay(2); 
//   }
  
//   motorStop();
//   delay(150); 
  
//   // Snap global target heading to the newly squared orientation
//   targetYaw = getYaw(2); 
// }

// void approachAndSquareUp(int rawFront, int rawFront2) {
//   const float ALPHA = 0.2f;  // EMA weight for derivative smoothing

//   // 1. Safe dt calculation
//   unsigned long now = millis();
//   float dt = (_approach_lastTime == 0) ? 0.01f 
//              : constrain((now - _approach_lastTime) / 1000.0f, 0.001f, 0.05f);
//   _approach_lastTime = now;

//   // 2. Normalize and clamp mapping
//   int frontNormalized  = constrain(map(rawFront,  FRONT_WALL_THRESHOLD, FRONT_MAX,  50, 100), 20, 110);
//   int front2Normalized = constrain(map(rawFront2, FRONT2_WALL_THRESHOLD, FRONT2_MAX, 50, 100), 20, 110);
//   int avgProximity     = (frontNormalized + front2Normalized) / 2;

//   float error = (float)(frontNormalized - front2Normalized);

//   // 3. Seed lastError safely on the very first loop to prevent kicks
//   if (isnan(_approach_lastError)) _approach_lastError = error;
  
//   float rawDeriv = (error - _approach_lastError) / dt;
//   _approach_derivFilt = ALPHA * rawDeriv + (1.0f - ALPHA) * _approach_derivFilt; 
//   _approach_lastError = error;

//   // 4. Calculate clamped turn adjustment
//   float turnAdjustment = constrain((Kp_align * error) + (Kd_align * _approach_derivFilt), -40.0f, 40.0f);

//   // 5. Smooth linear deceleration as it approaches the wall
//   int baseSpeed = 0;
//   if (avgProximity < 85) {
//     baseSpeed = constrain((int)map(avgProximity, 20, 85, 55, 5), 5, 55);
//   }

//   // 6. Apply differential steering
//   int leftSpeed  = constrain(baseSpeed - (int)turnAdjustment, -35, 80);
//   int rightSpeed = constrain(baseSpeed + (int)turnAdjustment, -35, 80);

//   motor1Wraper(leftSpeed);
//   motor2Wraper(rightSpeed);
// }

void alignToFrontWall() {
  _approach_lastTime  = 0;
  _approach_lastError = NAN;
  _approach_derivFilt = 0.0f;

  unsigned long alignedStartTime = 0;
  unsigned long alignOverallTimeout = millis(); 
  bool isFullyAligned = false;

  // --- RAW TUNING CONSTANTS ---
  const float TARGET_PROXIMITY_RAW = 208.0f; // Target center reading
  const float TARGET_ERROR_RAW = 10.0f;      // Allowable raw difference
  const int SETTLE_TIME_MS = 100;  

  // SENSOR BALANCE OFFSET
  const int SENSOR_OFFSET = 77; 

  while (!isFullyAligned) {
    if (millis() - alignOverallTimeout > 2500) {
      BT.println("Align timed out");
      break;
    }

    int rawFront = getCorrectedReading(0);
    int rawFront2 = getCorrectedReading(5);

    int balancedFront2 = rawFront2 - SENSOR_OFFSET;

    int avgProximityRaw = (rawFront + balancedFront2) / 2;
    float errorRaw = (float)(rawFront - balancedFront2);

    if (avgProximityRaw >= TARGET_PROXIMITY_RAW && fabsf(errorRaw) <= TARGET_ERROR_RAW) {
      if (alignedStartTime == 0) {
        alignedStartTime = millis(); 
      } 
      else if (millis() - alignedStartTime > SETTLE_TIME_MS) {
        isFullyAligned = true; 
        break; 
      }
    } 
    else {
      alignedStartTime = 0; 
    }

    approachAndSquareUp(rawFront, rawFront2);
    delay(2); 
  }
  
  motorStop();
  delay(150); 
  
  targetYaw = getYaw(2); 
}

void approachAndSquareUp(int rawFront, int rawFront2) {
  const float ALPHA = 0.2f; 
  const int SENSOR_OFFSET = 77; 
  int balancedFront2 = rawFront2 - SENSOR_OFFSET;

  unsigned long now = millis();
  float dt = (_approach_lastTime == 0) ? 0.01f 
             : constrain((now - _approach_lastTime) / 1000.0f, 0.001f, 0.05f);
  _approach_lastTime = now;

  int avgProximityRaw = (rawFront + balancedFront2) / 2;
  float errorRaw = (float)(rawFront - balancedFront2);

  if (isnan(_approach_lastError)) _approach_lastError = errorRaw;
  
  float rawDeriv = (errorRaw - _approach_lastError) / dt;
  _approach_derivFilt = ALPHA * rawDeriv + (1.0f - ALPHA) * _approach_derivFilt; 
  _approach_lastError = errorRaw;

  float turnAdjustment = constrain((Kp_align * errorRaw) + (Kd_align * _approach_derivFilt), -40.0f, 40.0f);

  int baseSpeed = 0;
  const int START_BRAKE_RAW = 150;       
  const int TARGET_PROXIMITY_RAW = 208;  

  if (avgProximityRaw < TARGET_PROXIMITY_RAW) {
    baseSpeed = constrain((int)map(avgProximityRaw, START_BRAKE_RAW, TARGET_PROXIMITY_RAW, 55, 5), 5, 55);
  } 
  else if (avgProximityRaw > TARGET_PROXIMITY_RAW + 15) {
    baseSpeed = -15; 
  }

  int leftSpeed  = constrain(baseSpeed - (int)turnAdjustment, -35, 80);
  int rightSpeed = constrain(baseSpeed + (int)turnAdjustment, -35, 80);

  // Restored Print Statements
  BT.print("rawFront:"); BT.println(rawFront);
  BT.print("rawFront2:"); BT.println(rawFront2);
  BT.print("balancedFront2:"); BT.println(balancedFront2);
  BT.print("avgProximityRaw:"); BT.println(avgProximityRaw);
  BT.print("errorRaw:"); BT.println(errorRaw);
  BT.print("turnAdjustment:"); BT.println(turnAdjustment);
  BT.print("leftSpeed:"); BT.println(leftSpeed);
  BT.print("rightSpeed:"); BT.println(rightSpeed);

  motor1Wraper(leftSpeed);
  motor2Wraper(rightSpeed);
}