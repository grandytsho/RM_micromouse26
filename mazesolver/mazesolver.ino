#include "one.h"
#include "two.h"
#include "floodFill.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>

int a=10, e =12;
bool  has_started = false; 
void rightWallFollow();
void leftWallFollow();

void setup() {
  inithardware();
  BT.print("available");
}

void loop() {
  if (BT.available() > 0) {
    char incomingChar = BT.read();
    BT.println(incomingChar);
    //Ignore stray invisible characters from the Serial Monitor
    if (incomingChar == '\n' || incomingChar == '\r') {
      return; 
    }
    
    BT.print(incomingChar);

    if (incomingChar == 'f') {
      BT.print("\nEnter distance value: ");
      while(BT.available() == 0){delay(1);}
      float dist = BT.parseFloat(); 
      centerUntilDistance(dist);
    }
    
    if (incomingChar == 'Y') {
      BT.println(getYaw(15));
    }
    
    if (incomingChar == 'p') {
      BT.print("\nEnter KP  value: ");
      while (BT.available() == 0) { delay(1); }
      Kp= BT.parseFloat(); 
      BT.print("\nKP  value: "); 
      BT.println(Kp);
    }
    
    if (incomingChar == 'd') {
      BT.print("\nEnter KD value: "); 
      while (BT.available() == 0) { delay(1); }
      Kd = BT.parseFloat(); 
      BT.print("\nKD value: "); 
      BT.println(Kd);
    }

    if (incomingChar == 'i') {
      BT.print("\nEnter KI value: "); 
      while (BT.available() == 0) { delay(1); }
      Ki = BT.parseFloat(); 
      BT.print("\nKI value: "); 
      BT.println(Ki);
    }


    if(incomingChar == 'c'){
      BT.print("\nEnter distance value: "); 
      while (BT.available() == 0) { delay(1); }
      distance = BT.parseFloat(); 
      BT.print("\ndistance value: ");
      BT.println(distance);
    }
    
    if (incomingChar == 't') {
      turn(-90);
    }
    if (incomingChar == 'z') {
      turn(90);
    }
    if(incomingChar == 'r'){
      while(true){
        wallFollower = true; 
        rightWallFollow();
      }
    }
    if(incomingChar == 'l'){
      while(true){
        wallFollower = true;
        leftWallFollow();
      }
    }
    if(incomingChar == 'x'){
      wallFollower = false; 
      BT.println("Starting floodfill"); 
      executeFloodFill(); 
    }

    if(incomingChar == 'b'){
      BT.print("\nEnter base speed: "); 
      while (BT.available() == 0) { delay(1); }
      BASE_SPEED = BT.parseInt();
      BT.print("\nBase Speed: ");
      BT.println(BASE_SPEED);
    }
    if(incomingChar == 'e'){
      BT.print("\n e value(13)"); 
      while (BT.available() == 0) { delay(1); }
      e = BT.parseInt();
      BT.print("\ne val: ");
      BT.println(e);
    }
    if(incomingChar == 'a'){
      BT.print("\n a value(25)"); 
      while (BT.available() == 0) { delay(1); }
      a = BT.parseInt();
      BT.print("\a val: ");
      BT.println(a);
    }

    //Goal setting

    if(incomingChar == 'q'){
      BT.print("\n Enter X coordinate: "); 
      while(BT.available() == 0){ delay(1); }
      goal_x = BT.parseInt(); 
      BT.print("Goal X: "); BT.println(goal_x); 
    }

    if(incomingChar == 'w'){
      BT.print("\n Enter Y coordinate: "); 
      while(BT.available() == 0){ delay(1); }
      goal_y = BT.parseInt(); 
      BT.print("Goal Y: "); BT.println(goal_y); 
    }
  } // <-- This brace closes the if(BT.available() > 0) statement
}


// static int lastRunmodePresses = runmodeButtonPressCount; 

// void loop() {
  
//   if (lastRunmodePresses != runmodeButtonPressCount) {
//     lastRunmodePresses = runmodeButtonPressCount; 
//     has_started = true; 
//     BT.println("Starting Algorithm!");
//   }

  
//   if (has_started) {
//     if (algoButtonPressCount == 0) {
//       wallFollower = false;
//       executeFloodFill(); 
      
//       has_started = false; 
//     }
//     else if (algoButtonPressCount == 1) {
//       wallFollower = true;
//       leftWallFollow(); 
//     }
//     else if (algoButtonPressCount == 2) {
//       wallFollower = true;
//       rightWallFollow();
//     }
//   }
// }

void rightWallFollow() { 
  //Turn right
  if (!isWallRight()) {
    BT.println("NO RIGHT WALL");
    // motorStop();
    // delay(1000);
    centerUntilDistance(e);
    motorStop();
    delay(1);
    turn(-90.0f);
    centerUntilDistance(a);
    delay(1);
  }
  //Drive forward using PID
  else if (!isWallFrontCollision()) {
    int currentLeft = getCorrectedReading(3);
    int currentRight = getCorrectedReading(1); 
    applyPIDCentering(currentLeft, currentRight);
  }
  // Turn left
  else if (!isWallLeft()) {
    BT.println("turning Left");
    BT.print("Front1: ");BT.println(getCorrectedReading(5));
    BT.print("Front2: ");BT.println(getCorrectedReading(0));
    //centerUntilDistance(2);
    delay(1);
    turn(90.0f);
    centerUntilDistance(a);
    delay(1);
  }
  //U-Turn
  else {
    BT.println("dead end, U-turn");
    motorStop();
    delay(1);
    turn(180.0f);
    delay(1);
  }
  
  delay(1);
  // BT.print("Right:");BT.println(isWallRight());
  // BT.print("Left:");BT.println(isWallLeft());
  // BT.print("Front:");BT.println(isWallFront());
}

void leftWallFollow() { 
  //Turn left
  if (!isWallLeft()) {
    BT.println("NO Left WALL|Left reading: "); BT.println(getCorrectedReading(3));
    // motorStop();
    // delay(1000);
    centerUntilDistance(e);
    motorStop();
    delay(1);
    turn(90.0f);
    centerUntilDistance(a);
    delay(1);
  }
  //Drive forward using PID
  else if (!isWallFrontCollision()) {
    BT.print("Front1: ");BT.println(getCorrectedReading(5));
    BT.print("Front2: ");BT.println(getCorrectedReading(0));
    int currentLeft = getCorrectedReading(3);
    int currentRight = getCorrectedReading(1);
    applyPIDCentering(currentLeft, currentRight);
  }
  // Turn right
  else if (!isWallRight()) {
    BT.println("turning Right");
    BT.print("Front1: ");BT.println(getCorrectedReading(5));
    BT.print("Front2: ");BT.println(getCorrectedReading(0));
    //centerUntilDistance(2); 
    delay(1);
    turn(-90.0f);
    centerUntilDistance(a);
    delay(1);
  }
  //U-Turn
  else {
    BT.println("dead end, U-turn");
    BT.print("Front1: ");BT.println(getCorrectedReading(5)); 
    BT.print("Front2: ");BT.println(getCorrectedReading(0)); 
    motorStop();
    delay(1);
    turn(180.0f);
    delay(1);
  }
  delay(1);
  // BT.print("Right:");BT.println(isWallRight());
  // BT.print("Left:");BT.println(isWallLeft());
  // BT.print("Front:");BT.println(isWallFront());
}