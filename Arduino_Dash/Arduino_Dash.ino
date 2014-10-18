#include <Wire.h>

#include <Adafruit_GFX.h>

#include <Adafruit_LEDBackpack.h>

//UK Solar Car Arduion Dash Program

Adafruit_7segment sseg1 = Adafruit_7segment();
void setup()
{
  Serial.begin(115200);
  Serial.println("Arduino_Dash");
  
  sseg1.begin(0x70);
  
}




void loop()
{
 for(int i = 0; i < 100; i++)
 {
   sseg1.print(i, DEC);
   delay(500);
 }
}
