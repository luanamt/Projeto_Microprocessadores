#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
}
 //--------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Serial connected 250Kbps...");
  float sum=0;
  for(int i=0;i<100;i++){
    sum += analogRead(A0);
  }
  Serial.print("ADC Value: ");
  Serial.print(((sum/102300)*3.3*1000)-32);
  Serial.print(" mV.");
  Serial.print(" - Temperature: ");
  Serial.print((((sum/102300)*3.3*1000)-32)/10);
  Serial.print(" oC.\n");
  delay(50);
}

