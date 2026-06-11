#ifndef one_H
#define one_H

#include <Arduino.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


void inithardware();
float getYaw();
extern const int sensorPins[];
extern const uint8_t EC_1A ; 
extern const uint8_t EC_1B ;
extern const uint8_t EC_2A ; 
extern const uint8_t EC_2B ;
extern const uint8_t M1_IN1 ;
extern const uint8_t M1_IN2 ;  
extern const uint8_t M2_IN1 ; 
extern const uint8_t M2_IN2 ; 


#endif