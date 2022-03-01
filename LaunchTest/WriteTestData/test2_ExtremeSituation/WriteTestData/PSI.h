#ifndef PSI_h
#define PSI_h 	//create token
#include "Arduino.h"

class PSI{
  
private:
    int address = 0;
    int EEPORM_storage = 65535;
public:
    PSI();
    //void checkSettings();  
    int EEPROM_Check(int current_address);
    float ms2s(int ms);
    void buzzer_powerOn(int pin_address);
    void buzzer_EEPROM(int pin_address);
    void buzzer_sensor(int pin_address);
    void buzzer_ematch(int pin_address);
    void buzzer_powerLow(int pin_address);
};

#endif
