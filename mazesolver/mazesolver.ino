#include "one.h"
#include "two.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


void setup() {
  inithardware();

}

void loop() {
  Serial.print("Sensors: ");
  // Loop through all 6 sensors
  for (int i = 0; i < 6; i++) {
    // Assuming these are digital IR sensors. Use analogRead() if they are analog.
    int reading = digitalRead(sensorPins[i]); 
    Serial.print(reading);
    Serial.print(" | ");
  }
  Serial.println();
  delay(200);
}
