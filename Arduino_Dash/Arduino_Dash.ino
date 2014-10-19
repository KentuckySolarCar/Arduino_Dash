//UK Solar Car Arduion Dash Program

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

//Definitions
#define  sseg1_addr  0x70
#define  sseg2_addr  0x71


//Set up 7 segment displays
Adafruit_7segment sseg1 = Adafruit_7segment();
Adafruit_7segment sseg2 = Adafruit_7segment();

//Identification delay
long identification_delay = 500;
long last_identification= 0; 



int val1 = 3.45678;
int val2 = 103.45;



boolean pi_good = false; 


void writeDashes(Adafruit_7segment); 

void setup()
{

  //Open serial communications
  Serial.begin(115200);
  
  //begin ssegs
  sseg1.begin(sseg1_addr);
  sseg2.begin(sseg2_addr);
  
}




void loop()
{
  
  if(!pi_good)
  {
    //Wait for motor controller to send data
    writeDashes(sseg1);
    writeDashes(sseg2);
    sseg1.blinkRate(1);
    sseg2.blinkRate(1);
    
    
   //Identify every identification_delay ms
   if(millis() - last_identification > identification_delay)
   {
     Serial.println("Arduino_Dash");
     last_identification = millis();
   }
  }
  
  else
  {
    sseg1.print(val1, DEC);
    sseg2.print(val2, DEC);
    sseg1.writeDisplay();
    sseg2.writeDisplay();
    sseg1.blinkRate(0);
    sseg2.blinkRate(0);
   
   
  }
  
  //When pi responds, set pi_good true
  if(!pi_good && Serial.available() > 0)
  {
    pi_good = true;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
}











void writeDashes(Adafruit_7segment sseg)
{
  sseg.writeDigitRaw(0x00, 0b1000000);
  sseg.writeDigitRaw(0x01, 0b1000000);
  sseg.writeDigitRaw(0x03, 0b1000000);
  sseg.writeDigitRaw(0x04, 0b1000000);
  sseg.writeDisplay();
  
}
