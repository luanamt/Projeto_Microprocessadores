#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Serial connected 250Kbps...");

  delay(500);
}