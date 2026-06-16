#include "one.h"
#include "two.h"

// 1. Notice there are NO "extern" keywords here!
const uint8_t EC_1A = 0; 
const uint8_t EC_1B = 1;
const uint8_t EC_2A = 30; 
const uint8_t EC_2B = 31;
const int MOTOR_SPEED_30 = 100;
const int NUM_SENSORS = 6;
const int emitterPins[NUM_SENSORS] = {38, 41, 15, 18, 20, 23};
const int sensorPins[NUM_SENSORS]  = {39, 40, 14, 19, 21, 22};
const char* sensorNames[NUM_SENSORS] = {
  "Front 2  ", 
  "Right    ", 
  "Front Rgt", 
  "Left     ", 
  "Front Lft", 
  "Front    "
};

HardwareSerial &BT = Serial2;

// 2. Notice I added the NAMES for your motor pins here!
const uint8_t M1_IN1 = 2;  //direction
const uint8_t M1_IN2 = 3;  //direction
const uint8_t M2_IN1 = 24; //PWM
const uint8_t M2_IN2 = 25; //PWM
#define AT_COMMAND_LINE 4
const double MOTOR_BIAS = (127.0+15.0)/127.0; 
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue; // Struct to hold the complex data reports

void isrLeftEncoder() {
    if (digitalRead(EC_1A) == digitalRead(EC_1B)) {
        leftEncoderTicks--;
    } else {
        leftEncoderTicks++;
    }
}

void isrRightEncoder() {
    if (digitalRead(EC_2A) != digitalRead(EC_2B)) {
        rightEncoderTicks--;
    } else {
        rightEncoderTicks++;
    }
}

float getYaw() {
  // Check if a new sensor event is ready
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      
      // 1. Extract the Quaternions
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      // 2. Convert Quaternions to Yaw (Z-axis rotation in radians)
      float yaw_rad = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (sq(qj) + sq(qk)));
      
      // 3. Convert to degrees
      currentYaw = yaw_rad * (180.0 / PI);
    }
  }
  return currentYaw; 
}
float getRight(){
  int raw = getCorrectedReading(1);
  //float out = (2494.3112 / (raw + 31.6584)) - 0.4117;
  float out =raw;
  return out;
}
float getLeft(){
  int raw = getCorrectedReading(3);
  //float out = (2850.7585 / (raw + 14.7003)) - 2.6599;
  float out =raw;
  return out;
}
float getFront1(){
  int raw = getCorrectedReading(0);
  float out =raw;
  return out;
}
float getFront2(){
  int raw = getCorrectedReading(5);
  float out =raw;
  return out;
}
float getLeftDiagonal(){
  int raw = getCorrectedReading(4);
  float out =raw;
  return out;
}
float getRightDiagonal(){
  int raw = getCorrectedReading(2);
  float out =raw;
  return out;
}
float getCorrectedReading(int sensorNum){
  digitalWrite(emitterPins[sensorNum], LOW);
  float ambient_value = analogRead(sensorPins[sensorNum]); 
  digitalWrite(emitterPins[sensorNum], HIGH); 
  float raw_value = analogRead(sensorPins[sensorNum]); 
  return (raw_value - ambient_value); 
}

void inithardware(){
  Wire1.begin();
  Serial.begin(115200); 
  BT.begin(9600);
  while (!Serial && millis() < 2000); 

  // Initialize all pins using a loop
  for (int i = 0; i < NUM_SENSORS; i++) {
    if(i==0||i==5)continue;
    pinMode(emitterPins[i], OUTPUT);
    //digitalWrite(emitterPins[i], HIGH);  // Keep all IR transmitters ON
    pinMode(sensorPins[i], INPUT); 
  }
  
  pinMode(AT_COMMAND_LINE, OUTPUT);
  digitalWrite(AT_COMMAND_LINE, LOW);

  pinMode(EC_1A, INPUT_PULLUP);
  pinMode(EC_1B, INPUT_PULLUP);
  pinMode(EC_2A, INPUT_PULLUP);
  pinMode(EC_2B, INPUT_PULLUP);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(EC_1A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EC_2A), isrRightEncoder, CHANGE);

  if(!bno08x.begin_I2C(0x4A, &Wire1)) { // Check your specific breakout's address
      Serial.println("Couldnt intialise BNO085");
      while(1); 
  }
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);

  delay(1000); 
  Serial.println("testing"); 
  bno.setExtCrystalUse(true);
  BT.print("setupdone");
}
