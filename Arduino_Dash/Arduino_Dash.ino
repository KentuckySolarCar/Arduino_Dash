#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GPS.h>
//the GPS library needs this event though it isn't used.
#include <SoftwareSerial.h>

//Definitions
#define  sseg1_addr    0x70
#define  sseg2_addr    0x71
#define  ID_DELAY      500
#define  DATA_TIMEOUT  3000
#define  GPS_serial    Serial1

//Set up 7 segment displays
Adafruit_7segment sseg1 = Adafruit_7segment();
Adafruit_7segment sseg2 = Adafruit_7segment();

//Set up GPS
Adafruit_GPS GPS(&GPS_serial);

//Identification delay
long last_identification= 0; 
long last_data = 0;

float val1 = 999.1; //values written to the displays
float val2 = 999.2;

String serial_line = ""; //buffer for serial data
boolean serial_line_flag = false;

boolean pi_good = false; 


void writeDashes(Adafruit_7segment); 

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (c) UDR0 = c; 
  //char c = input_serial.read();
  //Serial.print(c);
}

void setup()
{

  //Open serial communications
  Serial.begin(115200);
  
  //begin ssegs
  sseg1.begin(sseg1_addr);
  sseg2.begin(sseg2_addr);
  
  //initialize input buffer
  serial_line.reserve(200);
  
  //GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PGCMD_NOANTENNA);
  
  //set up timer interrupt for GPS serial data
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
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
   if(millis() - last_identification > ID_DELAY)
   {
     Serial.println("Arduino_Dash");
     last_identification = millis();
   }
  }
  
  else
  {
    sseg1.print(val1);
    sseg2.print(val2);
    sseg1.writeDisplay();
    sseg2.writeDisplay();
    sseg1.blinkRate(0);
    sseg2.blinkRate(0);
  }
  
  //When pi responds, set pi_good true
  if(!pi_good && Serial.available() > 0)
  {
    pi_good = true;
    serial_line = "";
    last_data=millis();
  }
  
  parseSerial();
  
  if(millis()-last_data > DATA_TIMEOUT)
  {
    pi_good = false;
  }
  

  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
}







void serialEvent() //special built-in arduino function
{
  //read serial data into buffer, set flag when EOL
  while (Serial.available()) 
  {
    char inChar = (char)Serial.read(); 
    serial_line += inChar;
    
    if (inChar == '\n') 
    {
      serial_line_flag = true;
      //Serial.println("newline");
    } 
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

void parseSerial() 
{
  
  if(serial_line_flag)
  {
    char buf[200];
    last_data = millis();
    serial_line.trim();
    serial_line.toCharArray(buf,200);
   //parse serial data
   if(serial_line[0]=='S')
   {
     //sscanf(buf+2,"%f",&val1);
     val1 = (float) atof(buf+2);
   }
   else if (serial_line[0]=='C')
   {
     val2 = (float) atof(buf+2);
   } 
  }
    serial_line_flag = false;
    serial_line = "";
}
