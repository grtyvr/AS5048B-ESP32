#include <Arduino.h>
#include "AS5048.hpp"
#include <SPI.h>

const uint8_t azPin = 5;

AS5048A azSensor(azPin);



void setup() {
  Serial.begin(9600);
  azSensor.init();
  azSensor.printDiagnostics();
  delay(1000);
}

void loop() {
  Serial.print("Azimuth angle: ");
  Serial.println(azSensor.getAverageAngle(50));
//  Serial.print("Azimuth angle: ");
//  Serial.println(azSensor.getExpSmoothAngle(0.15));
//  azSensor.printDiagnostics();
//  Serial.print("         Gain: ");
//  Serial.println(azSensor.getGain());
//  azSensor.printDiagnostics();
//  if (azSensor.error()){
//    Serial.print("Error flag set: ");
//    uint8_t errors = azSensor.getErrors();
//    Serial.println(errors);
//  }

  delay(100);
}
