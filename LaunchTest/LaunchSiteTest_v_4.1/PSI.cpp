#include "PSI.h"


PSI::PSI() {
  address_store = 0;
  last_stored = 15;
}



void PSI::init_last_ten_a(double ten[]) {
  for (int i = 0; i < last_stored ; i++) {
    ten[i] = 3048;
  }
}

int PSI::store_ten(double a, int j) {
  last_ten_a [j] = a;
  j++;
  if (j == last_stored) j = 0;
  return j;
}

bool PSI::calculate_ten_a(double ten[]) {
  double sum, ave_pro_a;
  double pro_a[14];
  for (int i = 0; i < last_stored; i++) {
    pro_a[i] = (ten[i + 1] - ten[0]) / (i + 1);
    sum += pro_a[i];
  }
  ave_pro_a = sum / last_stored;
  return ave_pro_a;
}

int PSI::EEPROM_Check(int current_address) {
  current_address += sizeof(float);
  if (current_address == EEPORM_storage) {
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

  Wire.requestFrom(deviceaddress, 1);

  if (Wire.available()) rdata = Wire.read();

  return rdata;
}

float PSI::ms2s(unsigned long ms) {
  float s_float = (float)ms / 1000;
  return s_float;
}

void PSI::buzzer_EEPROM(int pin_address) {
  tone(pin_address, 200);
  delay(500);
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
void PSI::buzzer_sensor(int pin_address) {

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

void PSI::buzzer_ematch(int pin_address) {
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

void PSI::buzzer_powerLow(int pin_address) {
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

void PSI::buzzer_powerOn(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(500);
  digitalWrite(pin_address, LOW);
  delay(500);
}

void PSI::buzzer_EEPROMEreased(int pin_address) {
  digitalWrite(pin_address, HIGH);
  delay(250);
  digitalWrite(pin_address, LOW);
  delay(250);
  digitalWrite(pin_address, LOW);
  delay(250);
}