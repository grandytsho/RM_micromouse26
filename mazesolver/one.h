#ifndef one_H
#define one_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BNO08x.h>

void inithardware();
float getYaw(uint16_t timeoutMs);
float getRight();
float getLeft();
float getFront1();
float getFront2();
float getLeftDiagonal();
float getRightDiagonal();
float getCorrectedReading(int sensorNum);
float getCorrectedReadingAvg(int sensorNum);
void resetIMU();

//pins
extern const int sensorPins[];
extern const uint8_t EC_1A ; 
extern const uint8_t EC_1B ;
extern const uint8_t EC_2A ; 
extern const uint8_t EC_2B ;

extern const uint8_t M1_IN1; // M2_INPUT1
extern const uint8_t M1_IN2; // M2_INPUT1
extern const uint8_t M1_PWM; // M2 PWM

extern const uint8_t M2_IN1;  // M2_INPUT1
extern const uint8_t M2_IN2;  // M2_INPUT2
extern const uint8_t M2_PWM;  // M2 PWM

//encoder ticks
extern volatile long leftEncoderTicks ;
extern volatile long rightEncoderTicks;

extern const uint8_t ALGOPIN;
extern volatile bool stateChanged;

//buttons
extern volatile int runmodeButtonPressCount;
extern volatile int algoButtonPressCount;
extern volatile int currentModeButtonCount;

//constants
extern const float WHEEL_DIAMETER;
extern const float TICKS_PER_REVOLUTION_LEFT;
extern const float TICKS_PER_REVOLUTION_RIGHT;
extern const float  WHEEL_CIRCUMFERENCE;
extern const float TICKS_PER_CM;
extern const double MOTOR_BIAS;
extern const double MOTOR_BIAS_TURN; 

//bluetooth 
extern HardwareSerial &BT;
#endif