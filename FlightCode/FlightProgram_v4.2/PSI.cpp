#include "PSI.h"


PSI::PSI() {
  
}




int PSI::EEPROM_Check(uint16_t current_address,uint16_t EEPORM_storage) {
  current_address += sizeof(float);
  if (current_address >= EEPORM_storage) {
    current_address = EEPORM_storage-4;
  }
  return current_address;
}

float PSI::simplp (float *x,  float *y,
               int M, float xm1)
{
  int n;
  y[0] = x[0] + xm1;
  for (n=1; n < M ; n++) {
    y[n] =  x[n]  + x[n-1];
  }
  return x[M-1];
}



float PSI::ms2s(unsigned long ms) {
  float s_float = (float)ms / 1000;
  return s_float;
}

void PSI::buzzer_EEPROM(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(1500);
}
void PSI::buzzer_sensor(int pin_address) {

  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(1500);
}

void PSI::buzzer_ematch(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(1500);
}

void PSI::buzzer_powerLow(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(500);
  digitalWrite(pin_address, HIGH);
  delay(1000);
  digitalWrite(pin_address, LOW);
  delay(1500);
}

void PSI::buzzer_powerOn(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(250);
  digitalWrite(pin_address, LOW);

}

void PSI::buzzer_EEPROMEreased(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(250);
  digitalWrite(pin_address, LOW);
  delay(250);
  digitalWrite(pin_address, LOW);
}
