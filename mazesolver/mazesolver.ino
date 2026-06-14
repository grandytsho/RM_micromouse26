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