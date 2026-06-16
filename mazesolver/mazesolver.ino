#include "one.h"
#include "two.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void setup() {
  inithardware();
  BT.print("available");
}

void loop() {
  
  if (BT.available() > 0){
    char incomingChar = BT.read();
    BT.print(incomingChar);
    if(incomingChar=='T'){
      turn(90);
      BT.println(getYaw());
    }
    if(incomingChar=='Y'){
      BT.println(getYaw());
    }
    if (incomingChar == 'P') {
  BT.print("\nEnter KP value: ");

  // 1. Flush the buffer: Read and discard any leftover characters 
  // (like the 'Enter' key you pressed after typing 'P')
  while (BT.available() > 0) {
    BT.read();
  }

  // 2. Wait indefinitely until the user types the new value
  while (BT.available() == 0) {
    // The code is trapped in this empty loop until data arrives
  }

  // 3. Now that data has arrived, parse it
  Kp = BT.parseFloat(); 

  BT.print("\nKP value: "); 
  BT.print(Kp); 
  BT.println();
}
    if (incomingChar == 'D') {
  BT.print("\nEnter KD value: "); 

  // 1. Flush the buffer: Read and discard any leftover characters 
  // (like the 'Enter' key you pressed after typing 'P')
  while (BT.available() > 0) {
    BT.read();
  }

  // 2. Wait indefinitely until the user types the new value
  while (BT.available() == 0) {
    // The code is trapped in this empty loop until data arrives
  }

  // 3. Now that data has arrived, parse it
  Kd = BT.parseFloat(); 

  BT.print("\nKD value: "); 
  BT.print(Kd); 
  BT.println();
}
if (incomingChar == 'U') {
  BT.print("\nEnter upper value: "); 

  // 1. Flush the buffer: Read and discard any leftover characters 
  // (like the 'Enter' key you pressed after typing 'P')
  while (BT.available() > 0) {
    BT.read();
  }

  // 2. Wait indefinitely until the user types the new value
  while (BT.available() == 0) {
    // The code is trapped in this empty loop until data arrives
  }

  // 3. Now that data has arrived, parse it
  upper = BT.parseInt(); 

  BT.print("\nUpper value: "); 
  BT.print(upper); 
  BT.println();
}
if (incomingChar == 'L') {
  BT.print("\nEnter lower value: "); 

  // 1. Flush the buffer: Read and discard any leftover characters 
  // (like the 'Enter' key you pressed after typing 'P')
  while (BT.available() > 0) {
    BT.read();
  }

  // 2. Wait indefinitely until the user types the new value
  while (BT.available() == 0) {
    // The code is trapped in this empty loop until data arrives
  }

  // 3. Now that data has arrived, parse it
  lower = BT.parseInt(); 

  BT.print("\nLower value: "); 
  BT.print(lower); 
  BT.println();
}
  }
}

void leftWallFollow() { 
 if (!isWallLeft()) {// turn left
    turn(-90.0f);
    
    delay(5);  // small delay to settle
    //centerUntil25cm();
  }
  else if (!isWallFront()) {//move forward 
    // BT.println("Going front");
   // centerUntil25cm();
  }
  else if (!isWallRight()) {//turn right
        // BT.println("Going right");
    turn(90.0f);
    delay(5);
    //centerUntil25cm();
  }
  else {
        // BT.println("Going around");
    turn(180.0f);
    delay(5);
  }
  //bool a = alignToNearest90();
  delay(5);
}