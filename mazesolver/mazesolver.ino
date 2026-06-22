#include "one.h"
#include "two.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>

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
      centerUntilDistance(150);
    }
    
    if (incomingChar == 'Y') {
      BT.println(getYaw());
    }
    
    if (incomingChar == 'p') {
      BT.print("\nEnter KP turn value: ");
      while (BT.available() == 0) { delay(1); }
      Kp_turn = BT.parseFloat(); 
      BT.print("\nKP turn value: "); 
      BT.println(Kp_turn);
    }
    
    if (incomingChar == 'd') {
      BT.print("\nEnter KD turn value: "); 
      while (BT.available() == 0) { delay(1); }
      Kd_turn = BT.parseFloat(); 
  BT.print("\nKD turn value: "); 
      BT.println(Kd_turn);
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

void rightWallFollow() { 
 if (!isWallRight()) {
    turn(-90.0f);
    delay(5);  
  }
  else if (!isWallFront()) {
    centerUntilDistance(21.25);
  }
  else if (!isWallLeft()) {
    turn(90.0f);
    delay(5);
  }
  else {
    turn(180.0f);
    delay(5);
  }
  delay(5);
}
void leftWallFollow() { 
 if (!isWallLeft()) {
    turn(90.0f);
    delay(5);  
  }
  else if (!isWallFront()) {
    centerUntilDistance(21.25);
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