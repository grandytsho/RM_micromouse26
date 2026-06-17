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
    
    // Ignore stray invisible characters from the Serial Monitor
    if (incomingChar == '\n' || incomingChar == '\r') {
      return; 
    }
    
    BT.print(incomingChar);

    if (incomingChar == 'f') {
      centerUntilDistance(125);
    }
    
    if (incomingChar == 'Y') {
      BT.println(getYaw());
    }
    
    if (incomingChar == 'P') {
      BT.print("\nEnter KP value: ");
      // Wait for data without deleting it
      while (BT.available() == 0) { delay(1); }
      Kp = BT.parseFloat(); 
      BT.print("\nKP value: "); 
      BT.println(Kp);
    }
    
    if (incomingChar == 'D') {
      BT.print("\nEnter KD value: "); 
      // Wait for data without deleting it
      while (BT.available() == 0) { delay(1); }
      Kd = BT.parseFloat(); 
      BT.print("\nKD value: "); 
      BT.println(Kd);
    }
    
    if (incomingChar == 't') {
      BT.print("\nEnter turn threshold value: "); 
      // Wait for data without deleting it
      while (BT.available() == 0) { delay(1); }
      TURN_THRESHOLD = BT.parseInt();
      BT.print("\nTurn threshold value: "); 
      BT.println(TURN_THRESHOLD);
    }
  } // <-- This brace closes the if(BT.available() > 0) statement
}

void leftWallFollow() { 
 if (!isWallLeft()) {
    turn(-90.0f);
    delay(5);  
  }
  else if (!isWallFront()) {
    // move forward
  }
  else if (!isWallRight()) {
    turn(90.0f);
    delay(5);
  }
  else {
    turn(180.0f);
    delay(5);
  }
  delay(5);
}