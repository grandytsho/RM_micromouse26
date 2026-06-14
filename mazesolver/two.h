#ifndef two_H
#define two_H
#include "one.h"
#include "two.h"
#include <Arduino.h>

void motorStop();
void motor1Forward();
void motor2Forward();
void motor1Reverse();
void motor2Reverse();
bool isWallLeft();
bool isWallRight();
bool isWallFront();
void turn(float angleDeg);

#endif