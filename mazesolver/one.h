#ifndef one_H
#define one_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BNO08x.h>

void inithardware();
float getYaw();
float getRight();
float getLeft();
float getFront1();
float getFront2();
float getLeftDiagonal();
float getRightDiagonal();
float getCorrectedReading(int sensorNum);
float getCorrectedReadingAvg(int sensorNum);

//pins
extern const int sensorPins[];
extern const uint8_t EC_1A ; 
extern const uint8_t EC_1B ;
extern const uint8_t EC_2A ; 
extern const uint8_t EC_2B ;
extern const uint8_t M1_IN1 ;
extern const uint8_t M1_IN2 ;  
extern const uint8_t M2_IN1 ; 
extern const uint8_t M2_IN2 ;

//encoder ticks
extern volatile long leftEncoderTicks ;
extern volatile long rightEncoderTicks;

//constants
extern const double WHEEL_DIAMETER;
extern const double TICKS_PER_REVOLUTION;
extern const double  WHEEL_CIRCUMFERENCE;
extern const double TICKS_PER_CM;
extern const double MOTOR_BIAS;

//bluetooth 
extern HardwareSerial &BT;
#endif