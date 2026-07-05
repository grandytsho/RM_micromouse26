#ifndef two_H
#define two_H
#include "one.h"
#include <Arduino.h>
extern float Kp;
extern  float Ki; 
extern  float Kd;

extern float Kp_turn;
extern float Kd_turn;
extern float Ki_turn; 
extern int upper;
extern int lower;

extern float Kp_align;
extern float Kd_align;

extern int LEFT_SETPOINT;
extern int LEFT_MAX;

extern int RIGHT_SETPOINT;
extern int RIGHT_MAX;
extern int LEFT_WALL_THRESHOLD;  
extern int RIGHT_WALL_THRESHOLD;
extern int TURN_THRESHOLD;

extern int BASE_SPEED;
extern bool wallFollower;

extern volatile float targetYaw;

void motorStop();
void motor1Forward(int speed);
void motor2Forward(int speed);
void motor1Reverse(int speed);
void motor2Reverse(int speed);
bool isWallLeft();
bool isWallRight();
bool isWallFront();
bool isWallFrontCollision();
void turn(float angleDeg);
void applyPIDCentering(int rawLeft, int rawRight);
void centerUntilDistance(float dist);
void centerUntilWall();
void alignToFrontWall();
void approachAndSquareUp(int rawFront, int rawFront2);
void uTurnAndAlign(); 

#endif