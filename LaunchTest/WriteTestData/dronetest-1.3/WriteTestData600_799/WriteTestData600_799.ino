/*
04/12/2021: v4.1: More user interface is added
*/

#include <Arduino.h>
#include <Wire.h>
#include "PSI.h"
#include <SparkFun_External_EEPROM.h> //download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git
#include "TestData.h"

//define relevant pins
#define Drogue_Release 2   //set Drogue_Release pin
#define Main_Release 3   //set Maine_Release pin
#define Ematch_Check 6
#define Sensor_Check 7
#define Buzzer_Set 8
#define Main_Connect A0   //set Maine_Release pin
#define Battery_Check A6
#define Drogue_Connect A7   //set Drogue_Release pin

#define disk1 0x50    //Address of 24LC256 eeprom chip
// These are the variables that could be changed based on real life situation.
int battery_threshold = 400; //902;// 7.4/(8.4/1024)=902.095

float relativeAltitude;
int address;
float readAltitude;

PSI psi;
ExternalEEPROM myMem;
TestData test;

int size_TestData;

void setup() {
  //--- Serial Debugging ---
  Serial.begin(9600);
  Serial.println("REBOOT");
  //--- Establish Pin Modes and turn off all LEDs ---
  pinMode(Drogue_Release, OUTPUT);
  pinMode(Main_Release, OUTPUT);
  pinMode(Drogue_Connect, INPUT);
  pinMode(Main_Connect, INPUT);
  pinMode(Battery_Check, INPUT);
  pinMode(Buzzer_Set, OUTPUT);
  pinMode(Ematch_Check, OUTPUT);
  pinMode(Sensor_Check, OUTPUT);
  digitalWrite(Ematch_Check, LOW);
  digitalWrite(Sensor_Check, LOW);
  Wire.begin();
  Wire.setClock(400000); //Most EEPROMs can run 400kHz and higher
  // Get reference pressure for relative altitude
  address = 2400;
  relativeAltitude = 0;
  readAltitude;

  myMem.setMemorySize(512000/8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time


  Serial.println("adding memory");


  size_TestData = sizeof(test.test_data)/4;

  
  for(int j = 0; j < size_TestData;j++){
    myMem.put(address,test.test_data[j]);
    myMem.get(address,readAltitude);
    Serial.print("address ->" ); Serial.println(address);
    Serial.print(readAltitude, DEC); Serial.println(" s/m -> ");
    address = psi.EEPROM_Check(address);
  }
  psi.buzzer_powerOn(Buzzer_Set);
  Serial.println("Memory added.");
}


void loop() {
 
}
