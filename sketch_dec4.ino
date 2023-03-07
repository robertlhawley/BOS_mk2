#include "AS726X.h"
#include <SoftwareSerial.h>

// Ok, some ideas.  We are using A4 and A5 (wierd interior pins) of the mini to communicate with the 
// AS7263.  We need 2 more pins for the serial connection.  We can use any other pins to attach a DIP
// switch for configuration bits.  
// ideas for configuration:  
// - measurement mode- 2 bits
// - gain- 2 bits
// - baud rate- 2-3 bits 115200, 57600, 19200, 9600
// - calibrated or not? 1 bit?
// 
// DIP switch, 4 bits, will be connected to Arduino pins 10-13.  
//
// Other pieces to this puzzle:  concept:
// - arduino pro mini talks to the AS7623
// - need a light bulb (incandescent is impt)
// - sends data up a serial line through a RS485 transciever (1484 probably)
// - arduino wants 5 volts, AS wants 3.3 (tried working with 3.3v arduino but not ideal (garbled serial) FIXED THAT!  was compiling with the WRONG paramgerts!

AS726X sensor;
SoftwareSerial CampbellSerial(8, 7); // RX, TX

byte GAIN = 3;
byte MEASUREMENT_MODE = 2;

float R;
float S;
float T;
float U;
float V;
float W;

// config bits
int bit0 = 0;
int bit1 = 0;
int bit2 = 0;
int bit3 = 0;

void setup() {
  // set our transmitter pin as output
  pinMode(3, OUTPUT);

  // these 4 pins read the DIP switches for configuration on power up
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  bit0 = digitalRead(10);
  bit1 = digitalRead(11);
  bit2 = digitalRead(12);
  bit3 = digitalRead(13);
  // bits 0 and 1 (Switch 1 & 2) control the serial port (RS485) baud rate- defaults back to 9600 if failure  
  if (bit0 == bit1 && bit0 == 0) // 00 (11 on the DIP switch since the switch pulls it low)
    { 
      CampbellSerial.begin(115200);
    } 
  else if (bit0 < bit1 && bit0 == 0) // 01 (10 on the DIP switch)
    {
      CampbellSerial.begin(57600);    
    }
  else if (bit0 > bit1 && bit0 == 1) // 10 (01 on the DIP switch)
    {
      CampbellSerial.begin(19200);    
    }
  else  // 11 (00 on the DIP switch)
    {
      CampbellSerial.begin(9600);    
    }

  // bits 2 and 3 (Switch 3 & 4) control the gain- defaults back to 3 if failure  
  if (bit2 == bit3 && bit2 == 0) // 00 (11 on the DIP switch since the switch pulls it low)
    { 
      GAIN = 0;
    } 
  else if (bit2 < bit3 && bit2 == 0) // 01 (10 on the DIP switch)
    {
      GAIN = 1;    
    }
  else if (bit2 > bit3 && bit2 == 1) // 10 (01 on the DIP switch)
    {
      GAIN = 2;    
    }
  else  // 11 (00 on the DIP switch)
    {
      GAIN = 3;    
    }

  sensor.begin();
  sensor.setGain(GAIN);
  delay(200);
  sensor.setMeasurementMode(MEASUREMENT_MODE);
  // "key the mike" to transmit using the LTC1487
  digitalWrite(3, HIGH);
  delay(200);
}

void loop() {

    R = sensor.getCalibratedR();
    S = sensor.getCalibratedS();
    T = sensor.getCalibratedT();
    U = sensor.getCalibratedU();
    V = sensor.getCalibratedV();
    W = sensor.getCalibratedW();
  
    CampbellSerial.print(R);
    CampbellSerial.print(" ");
    CampbellSerial.print(S);
    CampbellSerial.print(" ");
    CampbellSerial.print(T);
    CampbellSerial.print(" ");
    CampbellSerial.print(U);
    CampbellSerial.print(" ");
    CampbellSerial.print(V);
    CampbellSerial.print(" ");
    CampbellSerial.println(W);

    Serial.print(R);
    Serial.print(" ");
    Serial.print(S);
    Serial.print(" ");
    Serial.print(T);
    Serial.print(" ");
    Serial.print(U);
    Serial.print(" ");
    Serial.print(V);
    Serial.print(" ");
    Serial.println(W);

    //  sensor.printMeasurements();//Prints out all measurements (calibrated)
    //  sensor.printUncalibratedMeasurements();//Prints out all measurements (calibrated)
}

