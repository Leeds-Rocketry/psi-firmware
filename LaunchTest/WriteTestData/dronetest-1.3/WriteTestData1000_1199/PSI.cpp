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
