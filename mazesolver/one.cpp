#include "two.h"
#include "one.h"
#include <math.h>

const uint8_t EC_1A = 0; 
const uint8_t EC_1B = 1;
const uint8_t EC_2A = 30; 
const uint8_t EC_2B = 31;
const int MOTOR_SPEED_30 = 100;

const int NUM_SENSORS = 6;

const int emitterPins[NUM_SENSORS] = {23, 41, 24, 21, 15, 39};
const int sensorPins[NUM_SENSORS]  = {22, 40, 25, 20, 14, 38};

const char* sensorNames[NUM_SENSORS] = {
  "Front   ", // Sens: 22, Emit: 23
  "Right    ", // Sens: 40, Emit: 41
  "Front Rgt", // Sens: 25, Emit: 24
  "Left     ", // Sens: 20, Emit: 21
  "Front Lft", // Sens: 14, Emit: 15
  "Front 2   "  // Sens: 38, Emit: 39
};

volatile int algoButtonPressCount = -1;
volatile int runmodeButtonPressCount = 0;
volatile int currentModeButtonCount = 0;

HardwareSerial &BT = Serial2; 


const uint8_t M1_IN1 = 2;  // M1_INPUT1
const uint8_t M1_IN2 = 4;  // M1_INPUT2
const uint8_t M1_PWM = 9;  // PWMA

const uint8_t M2_IN1 = 3;  // M2_INPUT1
const uint8_t M2_IN2 = 5;  // M2_INPUT2
const uint8_t M2_PWM = 28; // PWMB

const uint8_t ALGOPIN = 29;
const uint8_t RUNMODEPIN = 12;
volatile bool stateChanged = false;

const uint8_t STATE_HC_05 = 6; 

const double MOTOR_BIAS = (127.0+15.0)/127.0;
const double MOTOR_BIAS_TURN = 0.9383; 
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
const uint8_t IMU_RESET_PIN = 11;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue; 

float currentYaw = 0.0f;
const float WHEEL_DIAMETER = 3.4f;
const float TICKS_PER_REVOLUTION_LEFT = 743.0f;
const float TICKS_PER_REVOLUTION_RIGHT = 737.0f; 
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*M_PI;
const float TICKS_PER_CM_LEFT = TICKS_PER_REVOLUTION_LEFT/WHEEL_CIRCUMFERENCE;
const float TICKS_PER_CM_RIGHT = TICKS_PER_REVOLUTION_RIGHT/WHEEL_CIRCUMFERENCE;

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

float getYaw(uint16_t timeoutMs = 15) {
    unsigned long start = millis();
    while ((millis() - start) < timeoutMs) {
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                float qr = sensorValue.un.gameRotationVector.real;
                float qi = sensorValue.un.gameRotationVector.i;
                float qj = sensorValue.un.gameRotationVector.j;
                float qk = sensorValue.un.gameRotationVector.k;

                float yaw_rad = atan2(
                    2.0f * (qr * qk + qi * qj),
                    1.0f - 2.0f * (sq(qj) + sq(qk))
                );
                currentYaw = yaw_rad * (180.0f / PI);
                return currentYaw;  
            }
        }
    }
    return currentYaw;  
}
static inline void settleMicros(uint32_t us) {
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) { /* spin */ }
}

float getRight(){
  int raw = getCorrectedReading(1);
  return raw;
}
float getLeft(){
  int raw = getCorrectedReading(3);
  float correct= ((2020.0f/(float)raw) - 0.88f);
  return correct;
}
float getFront1(){
  int raw = getCorrectedReading(0);
  return raw;
}
float getFront2(){
  int raw = getCorrectedReading(5);
  return raw;
}
float getLeftDiagonal(){
  int raw = getCorrectedReading(4);
  return raw;
}
float getRightDiagonal(){
  int raw = getCorrectedReading(2);
  return raw;
}

float getCorrectedReading(int sensorNum){
  digitalWrite(emitterPins[sensorNum], LOW);
  settleMicros(1000);
  float ambient_value = analogRead(sensorPins[sensorNum]);
  digitalWrite(emitterPins[sensorNum], HIGH); 
  settleMicros(1000);
  float raw_value = analogRead(sensorPins[sensorNum]); 
  digitalWrite(emitterPins[sensorNum], LOW);
  return (raw_value - ambient_value); 
}

float getCorrectedReadingAvg(int sensorNum){
  const int NUM_SAMPLES = 5;
  
  float ambient_sum = 0;
  float raw_sum = 0;

  for(int i = 0; i < NUM_SAMPLES; i++){
    // Ambient read
    digitalWrite(emitterPins[sensorNum], LOW);
    settleMicros(1000);
    ambient_sum += analogRead(sensorPins[sensorNum]);

    // Active read
    digitalWrite(emitterPins[sensorNum], HIGH);
    settleMicros(1000);
    raw_sum += analogRead(sensorPins[sensorNum]);
  }

  digitalWrite(emitterPins[sensorNum], LOW);
  return (raw_sum - ambient_sum) / NUM_SAMPLES;
}


void runModeInterrupt(){
  runmodeButtonPressCount++;
  BT.println(runmodeButtonPressCount);
}

void algoInterrupt() {
  algoButtonPressCount++;
  algoButtonPressCount = algoButtonPressCount%3;
  currentModeButtonCount = currentModeButtonCount;
  stateChanged=true;
  BT.println(algoButtonPressCount);
}

void inithardware(){
  Wire1.begin();
  Serial.begin(115200); 

  BT.begin(9600);
  
  while (!Serial && millis() < 2000); 

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(emitterPins[i], OUTPUT);
    pinMode(sensorPins[i], INPUT); 
  }
  
  // UPDATED: Mapped to STATE_HC_05 
  pinMode(STATE_HC_05, OUTPUT);
  digitalWrite(STATE_HC_05, LOW);

  pinMode(EC_1A, INPUT_PULLUP);
  pinMode(EC_1B, INPUT_PULLUP);
  pinMode(EC_2A, INPUT_PULLUP);
  pinMode(EC_2B, INPUT_PULLUP);
  pinMode(ALGOPIN, INPUT_PULLUP);

  // UPDATED: Included all new motor logic pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);


  pinMode(ALGOPIN, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(RUNMODEPIN), runModeInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ALGOPIN), algoInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(EC_1A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EC_2A), isrRightEncoder, CHANGE);

  if (!bno08x.begin_I2C(0x4B, &Wire1, -1)) {
    Serial2.println("Address 0x4B failed, trying 0x4A...");
    if (!bno08x.begin_I2C(0x4A, &Wire1, -1)) {
       Serial2.println("ERROR: BNO085 not detected at all!");
       return;
    }
  }
  
  Serial2.println("IMU connected. Enabling reports...");
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(800);
}