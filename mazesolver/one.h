#ifndef one_H
#define one_H

#include <Arduino.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>


void inithardware();
float getYaw();
float getRight();
float getLeft();
float getFront1();
float getFront2();
float getLeftDiagonal();
float getRightDiagonal();
float getCorrectedReading(int sensorNum);
float getCorrectedReading();
void printCalibrationLevel();
void saveCalibrationToEEPROM();

extern const int sensorPins[];
extern const uint8_t EC_1A ; 
extern const uint8_t EC_1B ;
extern const uint8_t EC_2A ; 
extern const uint8_t EC_2B ;
extern const uint8_t M1_IN1 ;
extern const uint8_t M1_IN2 ;  
extern const uint8_t M2_IN1 ; 
extern const uint8_t M2_IN2 ;
extern volatile long leftEncoderTicks ;
extern volatile long rightEncoderTicks;
extern const double MOTOR_BIAS; 
extern HardwareSerial &BT;


#endif