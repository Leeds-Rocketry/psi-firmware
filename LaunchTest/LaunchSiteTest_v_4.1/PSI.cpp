#include "PSI.h"


PSI::PSI(){

}


int PSI::EEPROM_Check(int current_address){
  current_address+= sizeof(float);
  if (current_address == EEPORM_storage){
        current_address = 0;
    }
return current_address;
}


void PSI::writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
 
byte PSI::readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
}

float PSI::ms2s(int ms){
  float s_float = (float)ms/1000;
  return s_float;
}

void PSI::buzzer_EEPROM(int pin_address){
    tone(pin_address,200);
    delay(500);
    noTone(pin_address);
    delay(500);
    tone(pin_address,200);
    delay(500);
    noTone(pin_address);
    delay(500);
    tone(pin_address,200);
    delay(500);
    noTone(pin_address);
    delay(1500);
    }
void PSI::buzzer_sensor(int pin_address){
      
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(500);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(500);
    noTone(pin_address);
    delay(1500);
    }

void PSI::buzzer_ematch(int pin_address){
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(1500);
    }

void PSI::buzzer_powerLow(int pin_address){
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(500);
    tone(pin_address, 200);
    delay(1000);
    noTone(pin_address);
    delay(1500);
    }

void PSI::buzzer_powerOn(int pin_address){
    digitalWrite(pin_address, HIGH);
    delay(500);
    digitalWrite(pin_address, LOW);
    delay(500);
    }
/*
int PSI::Serial_transfer(void){
  int value = 0;
  int relativeAltitude = 0;
  float referenceTime, time_ms, time_s_float, myRead_altitude, myRead_time;

    if ( Serial.available()) {
      //Interpret ASCII code from laptop to real input numerical number
      char ch = Serial.read();
      if (ch >= '0' && ch <= '9') // is this an ascii digit between 0 and 9?
      {
        value = (value * 10) + (ch - '0'); // yes, accumulate the value

      }
      else if (ch == 10) { // is the character the newline character
        relativeAltitude = value;  // set accumulated value
        value = 0; // reset val to 0 ready for the next sequence of digits
        time_ms = millis() - referenceTime;
        time_s_float = ms2s(time_ms);
        Serial.print("address = ");
        Serial.print(address, DEC);
        Serial.print(", ");
        myMem.put(address, time_s_float);
        myMem.get(address, myRead_time);
        Serial.print("time = ");
        Serial.print(myRead_time, DEC);
        Serial.print("s, ");
        address = psi.EEPROM_Check(address);
        Serial.print("address = ");
        Serial.print(address, DEC);
        Serial.print(", ");
        myMem.put(address, relativeAltitude);
        myMem.get(address, myRead_altitude);
        Serial.println(myRead_altitude, DEC);
        address = EEPROM_Check(address);
        Serial.print("The altitude is now: ");
        Serial.print(relativeAltitude, DEC);
        Serial.println(" meters");
      }  
  return relativeAltitude;
}
*/
