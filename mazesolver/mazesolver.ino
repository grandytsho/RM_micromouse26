#include "one.h"
#include "two.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>


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
    // Ignore stray invisible characters from the Serial Monitor
    if (incomingChar == '\n' || incomingChar == '\r') {
      return; 
    }
    
    BT.print(incomingChar);

    if (incomingChar == 'f') {
      centerUntilDistance(100);
    }

    if (incomingChar == 'a') {
      alignToFrontWall();
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
      BT.print("\nKi value: ");
      BT.println(Ki);
    }
    
    if (incomingChar == 't') {
      turn(-90);
    }
    if (incomingChar == 'z') {
      turn(90);
    }
    if(incomingChar == 'r'){
      while(true){
        rightWallFollow();
      }
    }
    if(incomingChar == 'l'){
      while(true){
        leftWallFollow();
      }
    }
  } // <-- This brace closes the if(BT.available() > 0) statement
}

void rightWallFollow(){
  if(!isWallRight()){
    BT.println("no right wall, turning right"); 
    delay(500);
    turn(-90.0f);
    delay(5);
    if(!isWallFront()){
      BT.println("moving straight after right turn");
      delay(500); 
      centerUntilDistance(25);
    }
  }
  else if(!isWallFront()){
    BT.println("no front, wall moving forward");
    delay(500); 
    centerUntilDistance(25);
    delay(5);
  }
  else if(!isWallLeft()){
    BT.println("front and right wall, turning left");
    delay(500);
    turn(90.0);
    delay(5);
    if(!isWallFront()){
      BT.println("moving forward after left turn");
      delay(500); 
      centerUntilDistance(25); 
    }
  }
  else{
    BT.println("dead end, U-turn");
    delay(500);
    turn(180.0f);
    delay(5);
    if(!isWallFront()){
      BT.println("moving forward after U-turn");
      delay(50); 
      centerUntilDistance(25); 
    }
  }

  BT.print("Right:");BT.println(isWallRight());
  BT.print("Left:");BT.println(isWallLeft());
  BT.print("Front:");BT.println(isWallFront());

}

// void rightWallFollow() { 
//  if (!isWallRight()) {
//     turn(-90.0f);
//     delay(5);
//     if(!isWallFront()){
//       centerUntilDistance(25);
//       centerUntilWall();
//     }
//   }
//   else if (!isWallFront()) {
//     centerUntilWall();
//     centerUntilDistance(15);
//   }
//   else if (!isWallLeft()) {
//     turn(90.0f);
//     delay(5);
//   }
//   else {
//     turn(180.0f);
//     delay(5);
//   }
//   delay(5);
// }
void leftWallFollow() { 
 if (!isWallLeft()) {
    turn(90.0f);
    delay(5);  
  }
  else if (!isWallFront()) {
    centerUntilWall();
  }
  else if (!isWallRight()) {
    turn(-90.0f);
    delay(5);
  }
  else {
    turn(180.0f);
    delay(5);
  }
  delay(5);
}