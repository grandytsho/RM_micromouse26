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

// 4. You MUST create the bno object here so bno.begin() works later!
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

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

// Your hardware setup function
void inithardware(){
  Wire1.begin();
  Serial.begin(115200); 
  BT.begin(115200);
  while (!Serial && millis() < 2000); 

  // Initialize all pins using a loop
  for (int i = 0; i < NUM_SENSORS; i++) {
    if(i==0||i==5)continue;
    pinMode(emitterPins[i], OUTPUT);
    digitalWrite(emitterPins[i], HIGH);  // Keep all IR transmitters ON
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

  if(!bno.begin()){
      Serial.println("Couldnt intialise BNO55");
      while(1); 
  }

  delay(1000); 
  Serial.println("testing"); 
  bno.setExtCrystalUse(true);
  BT.print("setupdone");
}

float getYaw(){
  sensors_event_t event; 
  bno.getEvent(&event); 
  return event.orientation.x;
}

float getRight(){
  int raw = analogRead(sensorPins[1]);
  float out = (2494.3112 / (raw + 31.6584)) - 0.4117;
  return out;
}

float getLeft(){
  int raw = analogRead(sensorPins[3]);
  float out = (2850.7585 / (raw + 14.7003)) - 2.6599;
  return out;
}

float getCorrectedReading(int sensorNum){
  digitalWrite(emitterPins[sensorNum], LOW);
  float ambient_value = analogRead(sensorPins[sensorNum]); 
  digitalWrite(emitterPins[sensorNum], HIGH); 
  float raw_value = analogRead(sensorPins[sensorNum]); 
  return (raw_value - ambient_value); 
}