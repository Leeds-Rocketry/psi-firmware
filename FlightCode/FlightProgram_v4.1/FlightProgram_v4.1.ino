
/*===============================================
  Yudong Xiao, Leeds University Rocket Association(LURA), University of Leeds

  |This code is used for PSI(Pressure Measure Unit) that will be deployed in LURA's rocket, G1 and future rockets.
  PSI is the redundent componets for Easymini. It is supposed to complete tha task of: Reading altitude data from altimeter sensor(MS5611), store the data to EEPORM(24LC512) and detemine when to deploy two parachutes.

  04/12/2021 V4.1: Tips to check battery_threshold is added.
  ==============================================
*/
/*===============================================
  Update 13/2/2022
  Change on PSI.h:
  1.One Buzzer Buzzing function of EEPROM ereased is added
  ==============================================
*/
/*===============================================
  Update 28/2/2022
  Change on PSI.h:
  1.Tuneing the active buzzer is now using digitalWrite function instead of tune() which used PWM.
  2.Removed all serial functions
  3.Removed last dealy in PSI.h Buzzer_powerOn();
  ==============================================
*/

#include <Arduino.h>

#include "MS5611.h"
#include "PSI.h"

#include <SparkFun_External_EEPROM.h>                       //External EEPORM library. download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git

//---Define relevant pins---
#define Drogue_Release 3                                    //E-match for drogue parachute
#define Main_Release 2                                      //E-match for main parachute
#define Ematch_Check 6
#define Sensor_Check 7
#define Battery_Check A6
#define Buzzer_Set 8
#define Drogue_Check A0                                      //Pin for Check the status of drogue ematch                                      
#define Main_Check A7                                        //Pin for Check the status of main ematch

//---Others---
#define disk1 0x50                                           //Address of 24LC256 eeprom chip

//---Declare general variables. Can be changed according to different flight condition.---
//Can these not be #define as well? Saves runtime memory
int battery_threshold = 902;                                 // 7.4/(8.4/1024)=902.095
unsigned int EEPORM_storage = 64000;                         //512k 24LC256 eeprom chip has 64k bytes
unsigned long ignition_duration = 1500;                      // Time period for ematches to be ignited. In ms
float main_release_a = 80;                                   //The altitude for main parachute to be deployed. 800ft for SAC
float launch_threshold = 0.5;                                // The threshold altitude when EEPROM starts storing data
float land_threshold = -3;                                   // The threshold altitude when EEPROM stop saving data

float EEPROM_DrogueMarK = 11111.0;                           //The mark put in EEPROM marking the time when apogee is detected and drogue parachute is deployed
float EEPROM_MainMarK = 22222.0;                             //The mark put in EEPROM marking the time when main parachute is deployed
int reading_rate = 100;                                      //The time delayed for each iteration. 100 gives about 10 Hz reading rate from the Drone test
int sample_size = 20;                                        //The number of data used to determine the apogee point
int shift_size = 10;                                         //The numerr of data skiped until next sample starts
bool enableEmatchCheck = false;

//Variables used for sensor
//Would recommend asigning a value on variable intialization
double referencePressure;
float  absoluteAltitude, relativeAltitude;
long realPressure;

//---Variables for timing data---
unsigned long referenceTime;
unsigned long time_ms;                                        //Time in milisecond. Unsigned long has 4 bytes
float time_s_float;                                           //Time in second.

float drogue_start_time;                                     //Time for Drogue e match starts to be ignited
float main_start_time;                                       //Time for Drogue e match starts to be ignited

//---Initialization---
int MODE = 0;                                                 //Initialize flight mode to mode 0;
int address = 0;                                              //EEPROM starting address
bool ascending = true ;                                       //Initial flight condition is ascending
//---Initialization for apogee detection algorithm---
float sum_gradient = 0;                                       //Sum of gradients in each sample
float buffer_sum_gradient[3] = {0};                           //Up to 3 samples's gradients together determine the apogee point when all the gradients < 0
//---Others---
int last_address = 0;
int battery_val = 0;
float base_count = 0;                                           //Starting time for storing time data.

//---Create objects---
MS5611 BMP;
PSI psi;
ExternalEEPROM myMem;

// Why are these here and not in a header file? Based on a lotof global variables so extremely hard to read - what goes in and what are results of the function?
void EEPORM_Data_Store() {
  realPressure =   BMP.readPressure();
  relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);                             // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address);
  myMem.put(address, relativeAltitude);
  address = psi.EEPROM_Check(address);
}

void EEPORM_Mark_Store(float Mark) {
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);                             // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address);
  myMem.put(address, Mark);
  address = psi.EEPROM_Check(address);
}

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(BMP.getOversampling());
}

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
  //---clear EEPORM--- (not used now)
  //  myMem.erase();
  //  psi.buzzer_EEPROMEreased(Buzzer_Set);

  //---initialize BMP sensor---
  Wire.begin();
  Wire.setClock(400000);                                                //Most EEPROMs can run 400kHz and higher

  //---Detect status of EEPROM, Sensor, ematches and power.---
  //---Tone: EEPROM(...), Sensor(_..), ematches(_ _.), powerLow(_ _ _)---
  while (myMem.begin() == false)psi.buzzer_EEPROM(Buzzer_Set);
  while (!BMP.begin())psi.buzzer_sensor(Buzzer_Set);
  digitalWrite(Sensor_Check, HIGH);                                     //LED for sensors
  if (enableEmatchCheck == true) {
    while (analogRead(Drogue_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    while (analogRead(Main_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    digitalWrite(Ematch_Check, HIGH);                                     //LED for ematches
  }
  battery_val = analogRead(Battery_Check);
  while (battery_val <= battery_threshold) psi.buzzer_powerLow(Buzzer_Set);
  //---Settings for EEPROM---
  //Why have serial comms (checkSettings()) when on launchpad? Not connected to anything anyways
  checkSettings();
  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time

  referencePressure = BMP.readPressure();
  relativeAltitude = 0;
  MODE = 1;
}

void loop() {
  // Describe MODE 1 here
  if (MODE == 1) {
    delay(reading_rate * 10);                                           //Lower beeping rate
    psi.buzzer_powerOn(Buzzer_Set);                                     //Keeps beeping indicating PSI's working status
    realPressure =   BMP.readPressure();
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    //absoluteAltitude =   BMP.getAltitude(realPressure);

    if (relativeAltitude > launch_threshold) {
      referenceTime = millis();
      myMem.put(address, base_count);
      address = psi.EEPROM_Check(address);
      myMem.put(address, relativeAltitude);
      address = psi.EEPROM_Check(address);
      MODE++;
    }
  }
  //Describe MODE 2 here
  if (MODE == 2 && ascending == 1) {
    EEPORM_Data_Store();
    float readAltitude[sample_size] = {0};                        //Read form last altitude data
    float gradient[sample_size - 1] = {0};
    // When addresss >=160, it shows there are 20 altitude data already. Put them in a window and calculate the overall linear approximation  to determine the appeage.
    // If not. the window shifts to next 10 altitude data (data within 0.5 second).
    // Window size 20 data. Next sample is shifted by 10 data
    if (address >= sample_size * 8 && address >= last_address + shift_size * 8 ) {                                    //Time+altitdue has 8 bytes in total
      for (int i = 0; i < sample_size; i++) {
        myMem.get(address - 4 - 8 * (sample_size - 1 - i), readAltitude[i]);
      }                                        //Allocating the right address
      for (int i = 0; i < sample_size; i++) {
        if (i < sample_size - 1) {
          //Your approximation does not inclue the time domain? 1 dimensional data here, is this right? Does this work?
          gradient[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);                                             //Calculating every linear approximation.
          sum_gradient += gradient[i];
        }
      }
      buffer_sum_gradient[1] = buffer_sum_gradient[0];
      buffer_sum_gradient[0] = sum_gradient;
      sum_gradient = 0;
      last_address = address;
      //Shifting
      if (buffer_sum_gradient[0] < 0 && buffer_sum_gradient[1] < 0 ) {
        digitalWrite(Drogue_Release, HIGH);
        //psi.buzzer_powerOn(Buzzer_Set);
        EEPORM_Mark_Store(EEPROM_DrogueMarK);
        drogue_start_time = millis();
        while (millis() - drogue_start_time  < ignition_duration) {
          EEPORM_Data_Store();
          delay(reading_rate);
        }
        digitalWrite(Drogue_Release, LOW);
        ascending = false;
        MODE++;
      }
    }
    delay(reading_rate);
  }
  if (MODE == 3 && ascending == false) {
    EEPORM_Data_Store();
    if (relativeAltitude <= main_release_a) {
      digitalWrite(Main_Release, HIGH);
      EEPORM_Mark_Store(EEPROM_MainMarK);
      main_start_time = millis();
      while (millis() - main_start_time  < ignition_duration) {
        EEPORM_Data_Store();
        delay(reading_rate);
      }
      digitalWrite(Main_Release, LOW);
      MODE++;
    }
    delay(reading_rate);
  }

  if (MODE == 4 && ascending == false) {
    if (address != 0 && relativeAltitude >= land_threshold) {                                                             //Detection of landing and overlap in EEPROM
      EEPORM_Data_Store();
    } else MODE = 5;
    delay(reading_rate);
  }
  if (MODE == 5) {
    while (1) {
      psi.buzzer_powerOn(Buzzer_Set);
      delay(reading_rate);
    }
  }

}
