#ifndef PSI_h
#define PSI_h 	//create token
#include "Arduino.h"
#include "MS5611.h"
#include <Wire.h>  



class PSI{
public:
    PSI();
    //void checkSettings();  

    int EEPROM_Check(uint16_t current_address,uint16_t EEPORM_storage); // address+4 every update.
 
    float ms2s(unsigned long ms);
    void buzzer_powerOn(int pin_address);
    void buzzer_EEPROM(int pin_address);
    void buzzer_sensor(int pin_address);
    void buzzer_ematch(int pin_address);
    void buzzer_powerLow(int pin_address);
    void buzzer_EEPROMEreased(int pin_address);
    float simplp (float *x,  float *y,
               int M, float xm1);

//private:

    //uint16_t EEPORM_storage = 64000;

};

#endif
