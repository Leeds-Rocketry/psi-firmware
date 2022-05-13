wi#include <Wire.h>
#include "SparkFun_External_EEPROM.h"                                 // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
ExternalEEPROM myMem;

#define disk1 0x50                                                    //Address of 24LC256 eeprom chip

uint16_t EEPROM_storge = 64000;
int reading_rate = 0;

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);                                              //Most EEPROMs can run 400kHz and higher
  Serial.print("Mem size in bytes: ");
  Serial.println(myMem.length());
  if (myMem.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
    while (1)
      ;
  }
  Serial.println("Memory detected!");
  myMem.setMemorySize(512000 / 8);                                    //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128);                                             //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete();                                 //Supports I2C polling of write completion
  myMem.setPageWriteTime(3);                                          //3 ms max write time
  uint16_t address = 0;
  Serial.print("Time/s     "); Serial.println("Height/m     ");
  EEPROM_storge = myMem.length();
  float myRead_time;
  float myRead_altitude;
  uint16_t i = 0;
  for ( i; i < EEPROM_storge; i += sizeof(float)) {

    //Serial.print(i/4, DEC);
    //Serial.print(", ");
    myMem.get(i, myRead_time);
    Serial.print(myRead_time, 4);
    Serial.print(", ");
    i += sizeof(float);

    //Serial.print(i, DEC);
    //Serial.print(", ");
    myMem.get(i, myRead_altitude);
    Serial.println(myRead_altitude, 4);

    //delay(reading_rate);
  }
  //Serial.print(myRead_time, 4);
  //Serial.print(", ");
  //Serial.print(i, DEC);
  //Serial.print(", ");
  //Serial.println(myRead_altitude, 4);

}

void loop() {}
