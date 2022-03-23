
/*===============================================
  Yudong Xiao, Leeds University Rocket Association(LURA), University of Leeds

  This code is meant to only record pressure data.

  22/03/2022 V4.1: Tips to check battery_threshold is added.
  ==============================================
*/


#include <Arduino.h>
#include <Wire.h>

#include "MS5611.h"
#include "PSI.h"

#include <SparkFun_External_EEPROM.h>                       //External EEPROM library. download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git
#include <cppQueue.h>
//---Define relevant pins---
#define Drogue_Release 3                                    //E-match for drogue parachute
#define Main_Release 2                                      //E-match for main parachute
#define Ematch_Check 6
#define Sensor_Check 7
#define Battery_Check A6
#define Buzzer_Set 8
#define Drogue_Check A0                                      //Pin for Check the status of drogue ematch                                      
#define Main_Check A7                                        //Pin for Check the status of main ematch

#define  IMPLEMENTATION  FIFO

//---Others---
#define disk1 0x50                                              //Address of 24LC256 eeprom chip    
#define battery_threshold  902                                 // 7.4/(8.4/1024)=902.095

#define ignition_duration  1500                                //Time period for ematches to be ignited. In ms
#define reading_rate  60                                      //The time delayed for each iteration. 100 gives about 10 Hz reading rate from the Drone test 

#define sample_size  15                                        //The number of data used to determine the apogee point. There are two continues samples together to determine the apogee.


#define main_release_a  243.8f                                 //The altitude for main parachute to be deployed. 800ft for SAC

#define launch_threshold  20.0f                               // The threshold altitude when EEPROM starts storing data
#define land_threshold  10.0f                               // The threshold altitude when EEPROM stop saving data
#define apogee_threshold 1.5f                              // apogee gradient threshold should be less than gradient_init
#define gradient_init 2.0f

bool enableEmatchCheck = false;
bool enableBatteryCheck = false;

typedef struct strData {
  float time;
  float altitude;
} Data;

//---Declare general variables. Can be changed according to different flight condition.---


//Variables used for sensor
double referencePressure = 0.0;
float  absoluteAltitude = 0.0f;
float relativeAltitude = 0.0f;
int32_t realPressure = 0;

//---Variables for timing data---
unsigned long referenceTime = 0;
unsigned long time_ms = 0;                                       //Time in milisecond. Unsigned long has 4 bytes
float time_s_float = 0.0;                                        //Time in second.

float drogue_start_time = 0.0;                                   //Time for Drogue e match starts to be ignited
float main_start_time = 0.0;                                     //Time for Drogue e match starts to be ignited

//---Initialization---
volatile int MODE = 0;                                           //Initialize flight mode to mode 0;
uint16_t EEPORM_storage = 64000;                                 //512k 24LC256 eeprom chip has 64k bytes
uint16_t address = 0;                                            //EEPROM starting address
int address_count = 0;
int shift_size = 0;
//---Initialization for apogee detection algorithm---
float sum_gradient = 0.0f;                                       //Sum of gradients in each sample

//---Others---
int battery_val = 0;
unsigned long beepStart = 0;
Data time_altitude = {0.0, 0.0};
float buffer_sum_gradient[2] = {0.0};                             //Up to 3 samples's gradients together determine the apogee point when all the gradients < 0
Data readTimeAltitude = {0.0, 0.0};
Data forPop = {0.0, 0.0};



cppQueue  OuterSample(sizeof(Data), sample_size + shift_size, IMPLEMENTATION);  // Instantiate queue
//---Create objects---
MS5611 BMP;
PSI psi;
ExternalEEPROM myMem;

void setup() {
  //--- Establish Pin Modes and turn off Checking LEDs ---
  pinMode(Drogue_Release, OUTPUT);
  pinMode(Main_Release, OUTPUT);
  pinMode(Drogue_Check, INPUT);
  pinMode(Main_Check, INPUT);
  pinMode(Battery_Check, INPUT);
  pinMode(Buzzer_Set, OUTPUT);
  pinMode(Ematch_Check, OUTPUT);
  pinMode(Sensor_Check, OUTPUT);

  digitalWrite(Ematch_Check, LOW);
  digitalWrite(Sensor_Check, LOW);
  digitalWrite(Drogue_Release, LOW);
  digitalWrite(Main_Release, LOW);

  psi.buzzer_powerOn(Buzzer_Set);

  Wire.begin();
  Wire.setClock(400000); //Most EEPROMs can run 400kHz and higher

  //---clear eeprom--- (not used now)
  //  myMem.erase();
  //  psi.buzzer_EEPROMEreased(Buzzer_Set);
  //Serial.begin(9600);
  //Serial.println("REBOOT");
  //---initialize BMP sensor---

  //---Detect status of EEPROM, Sensor, ematches and power.---
  //---Tone: EEPROM(...), Sensor(_..), ematches(_ _.), powerLow(_ _ _)---


  battery_val = analogRead(Battery_Check);
  if (enableBatteryCheck == true) {
    while (battery_val <= battery_threshold) psi.buzzer_powerLow(Buzzer_Set);
  }

  while (myMem.begin() == false)psi.buzzer_EEPROM(Buzzer_Set);
  while (!BMP.begin())psi.buzzer_sensor(Buzzer_Set);
  digitalWrite(Sensor_Check, HIGH);                                     //LED for sensors

  if (enableEmatchCheck == true) {
    while (analogRead(Drogue_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    while (analogRead(Main_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    digitalWrite(Ematch_Check, HIGH);                                     //LED for ematches
  }
  delay(5000);
  psi.buzzer_powerOn(Buzzer_Set);

  //---Settings for EEPROM---
  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time
  int32_t readP = 0;
  float readA = 0.0;
  Serial.begin(9600);
  beepStart = millis();
  while (1) {

    realPressure =   BMP.readPressure();
    myMem.put(address, realPressure);

    myMem.get(address, readP);
    Serial.print("relative pressure = "); Serial.print(readP);

    address = psi.EEPROM_Check(address, EEPORM_storage);
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    myMem.put(address, relativeAltitude);

    myMem.get(address, readA);
    Serial.print(", relative altitude = "); Serial.println(readA);

    address = psi.EEPROM_Check(address, EEPORM_storage);


    delay(50);
  }

}

void loop() {
}
