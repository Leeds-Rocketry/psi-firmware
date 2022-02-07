#include <Wire.h>
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
ExternalEEPROM myMem;

#define disk1 0x50    //Address of 24LC256 eeprom chip

uint64_t EEPROM_storge=64000;
int reading_rate = 10;

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); //Most EEPROMs can run 400kHz and higher
  if (myMem.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
    while (1)
      ;
  }
  Serial.println("Memory detected!");
  myMem.setMemorySize(512000/8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time
   int address=0;
  Serial.print("Time/s     ");Serial.println("Height/m     ");


  for ( int i = 0; i < EEPROM_storge;i+=sizeof(float)) {
    float myRead_time;
    //Serial.print(i/4, DEC);
    //Serial.print(", ");
    myMem.get(i, myRead_time);
    Serial.print(myRead_time, 4);
    Serial.print(", ");
    i+=sizeof(float);
    float myRead_altitude;
    //Serial.print(i, DEC);
    //Serial.print(", ");
    myMem.get(i, myRead_altitude);
    Serial.println(myRead_altitude, 4);

    delay(reading_rate);
  }

}

void loop() {}
