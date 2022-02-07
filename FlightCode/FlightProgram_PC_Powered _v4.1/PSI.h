#ifndef PSI_h
#define PSI_h   //create token
#include "Arduino.h"
#include "MS5611.h"
#include <Wire.h>  



class PSI{
public:
    PSI();
    //void checkSettings();  
    int EEPROM_Check(int current_address);
    void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data);
    byte readEEPROM(int deviceaddress, unsigned int eeaddress);
    float ms2s(unsigned long ms);
    void buzzer_powerOn(int pin_address);
    void buzzer_EEPROM(int pin_address);
    void buzzer_sensor(int pin_address);
    void buzzer_ematch(int pin_address);
    void buzzer_powerLow(int pin_address);
    //int Serial_transfer(void);

private:
    int buttonState = 0;
    int MODE = 0;       //initialize mode to zero
    int t = 0;          //array count number for last_ten_a
    int address = 0;
    bool ascending = true ;
    double drogue_state = 0;
    double main_state = 0;
    double current_a = 0;  // The last altitude. Used to determine relsase drouge

    double main_release_a = 243.84;
    int EEPORM_storage = 64000;

};

#endif
