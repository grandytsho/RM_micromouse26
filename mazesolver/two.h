#ifndef two_H
#define two_H
#include "one.h"
#include "two.h"
#include <Arduino.h>
extern float Kp;
extern  float Ki; 
extern  float Kd;
extern int upper;
extern int lower;
void motorStop();
void motor1Forward(int speed);
void motor2Forward(int speed);
void motor1Reverse(int speed);
void motor2Reverse(int speed);
bool isWallLeft();
bool isWallRight();
bool isWallFront();
void turn(float angleDeg);

#endif