
/*===============================================
  Yudong Xiao, Leeds University Rocket Association(LURA), University of Leeds

  |This code is used for PSI(Pressure Measure Unit) that will be deployed in LURA's rocket, G1 and future rockets.
  PSI is the redundent componets for Easymini. It is supposed to complete tha task of: Reading altitude data from altimeter sensor(MS5611), store the data to eeprom(24LC512) and detemine when to deploy two parachutes.

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
  /*===============================================
  Update 5/3/2022
  Change on PSI.h:
  1.Add CppQueue.h
  2.Updated new apogee detection algorithm. Apogee dection buffer does not read data from eeprom now.

  ==============================================*/
/*===============================================
  Update 16/3/2022
  Change on PSI.h:
  1.Deleted unused code
  Change on FlightProgram:
  1.shift_size = sample_size. Two samples have no overlapped data.
  ==============================================*/
/*===============================================
  Update 23/3/2022
  Change on FlightProgram:
  1.Deleted some unnecessary variables
  2.Battery threshold is changeed to 8.2V(/8.4V) and battery only indicate its low states without stopping the program
  ==============================================*/
/*===============================================
  Update 13/5/2022
  Update on FlightProgram:
  1. Add one more landing condition. When a set of altitude in a duration of land_timelimit has less variation within land_constant_threshold, it is considered as constant and the rocket has landed.
  ==============================================*/

#include <Arduino.h>
#include <Wire.h>

#include "MS5611.h"
#include "PSI.h"

#include <SparkFun_External_EEPROM.h>                       //External EEPROM library. download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git
#include <cppQueue.h>                                       //https://github.com/SMFSW/Queue.git 
//---Define relevant pins---
#define Drogue_Release  3                                    //E-match for drogue parachute
#define Main_Release    2                                    //E-match for main parachute
#define Ematch_Check    6
#define Sensor_Check    7
#define Battery_Check   A6
#define Buzzer_Set      8
#define Drogue_Check    A0                                    //Pin for Check the status of drogue ematch                                      
#define Main_Check      A7                                    //Pin for Check the status of main ematch

#define	IMPLEMENTATION	FIFO

//---Editable Area---

#define ignition_duration         2000                               //Time period for ematches to be ignited. In ms
#define reading_rate              60                                 //The time delayed for each iteration. 100 gives about 10 Hz reading rate from the Drone test 
#define sample_size               15                                 //The number of data used to determine the apogee point. There are two continues samples together to determine the apogee.
#define main_release_a            304.8f                             //The altitude for main parachute to be deployed. 800ft for SAC
#define launch_threshold          15.0f                              // The threshold altitude when EEPROM starts storing data
#define land_threshold            15.0f                              // The threshold altitude when EEPROM stop saving data
#define land_timelimit            5.0f
#define land_constant_threshold   1.0f
#define apogee_threshold          1.5f                               // apogee gradient threshold should be less than gradient_init


//---Non Editable Area---

#define disk1                     0x50                               //Address of 24LC256 eeprom chip    
#define battery_threshold         999                                // 8.2/(8.4/1024)=999
#define gradient_init             300
typedef struct strData {
  float	time;
  float	altitude;
} Data;

//---Declare general variables. Can be changed according to different flight condition.---
bool enableEmatchCheck          = true;
bool enableBatteryCheck         = true;

//Variables used for sensor
int32_t referencePressure       = 0;
int32_t realPressure            = 0;
float  absoluteAltitude         = 0.0f;
float relative_altitude         = 0.0f;

//---Variables for timing&altitude data---
unsigned long referenceTime     = 0;
unsigned long time_ms           = 0;                              //Time in milisecond. Unsigned long has 4 bytes
float time_s_float              = 0.0f;                           //Time in second.

float drogue_start_time         = 0.0f;                           //Time for Drogue e match starts to be ignited
float main_start_time           = 0.0f;                           //Time for Drogue e match starts to be ignited
float land_timelimit_start      = 0.0f;
float land_timelimit_stop       = 0.0f;
//---Initialization---
int MODE                         = 0;                              //Initialize flight mode to mode 0;
uint16_t EEPORM_length           = 64000;                          //512k 24LC256 eeprom chip has 64k bytes
uint16_t address                 = 0;                              //EEPROM starting address
int shift_size                   = 0;
//---Initialization for apogee detection algorithm---
float sum_gradient               = 0.0f;                           //Sum of gradients in each sample
uint16_t size_of_landSample      = 0;
float land_max_altitude          = 0.0f;
float land_min_altitude          = 0.0f;

//---Others---
unsigned long beep_start         = 0;                                //The beep starting time in MODE 1, lanuch ready mode. Below lanuchThreshold altitude.
Data time_altitude               = {0.0f, 0.0f};                     //The pair of time and altitude that is pushed to the queue(stack)
float buffer_sum_gradient[2]     = {0.0f};                           //The buffer that contains up to 2 samples's gradients together determine the apogee point when all the gradients < 0
Data read_time_altitude          = {0.0f, 0.0f};                     //Read data from the queue(stack) for apogee detection algorithm.
Data forPop                      = {0.0f, 0.0f};                     //A place to take the number that is poped out.

void eepromStoreData();
void eepromStoreMark();
float getSampleGradient(float x[], float y[], int samplesize);

cppQueue	OuterSample(sizeof(Data), sample_size + sample_size, IMPLEMENTATION);	// Instantiate queue
cppQueue	landSample(sizeof(Data), round(land_timelimit / ((reading_rate + 10) / 1000)), IMPLEMENTATION);	// Instantiate queue

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
  Wire.setClock(400000);                                                          //Most EEPROMs can run 400kHz and higher

  //---clear eeprom--- (not used now)
  //  myMem.erase();
  //  psi.buzzer_EEPROMEreased(Buzzer_Set);
  //Serial.begin(9600);
  //Serial.println("REBOOT");
  //---initialize BMP sensor---

  //---Detect status of EEPROM, Sensor, ematches and power.---
  //---Tone: EEPROM(...), Sensor(_..), ematches(_ _.), powerLow(_ _ _)---


  if (enableBatteryCheck == true) {
    if (analogRead(Battery_Check) <= battery_threshold)psi.buzzer_powerLow(Buzzer_Set);
    delay(2000);
    if (analogRead(Battery_Check) <= battery_threshold)psi.buzzer_powerLow(Buzzer_Set);
  }

  while (myMem.begin() == false)psi.buzzer_EEPROM(Buzzer_Set);
  while (!BMP.begin())psi.buzzer_sensor(Buzzer_Set);
  digitalWrite(Sensor_Check, HIGH);                                       //LED for sensors

  if (enableEmatchCheck == true) {
    while (analogRead(Drogue_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    while (analogRead(Main_Check) == 0) psi.buzzer_ematch(Buzzer_Set);
    digitalWrite(Ematch_Check, HIGH);                                     //LED for ematches
  }

  delay(5000);                                                            //Wait for 5 seconds and beep to indicate all checks are done.
  psi.buzzer_powerOn(Buzzer_Set);

  //---Settings for EEPROM---
  myMem.setMemorySize(512000 / 8);                                        //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128);                                                 //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete();                                     //Supports I2C polling of write completion
  myMem.setPageWriteTime(3);                                              //3 ms max write time

  referencePressure   = BMP.readPressure();
  MODE                = 1;
  EEPORM_length       = myMem.length();
  shift_size          = sample_size;                                      //No shared data between two data.
  beep_start          = millis();
  size_of_landSample  = round(land_timelimit / ((reading_rate + 10) / 1000));

}

void loop() {
  //Rocket is in ready to lanuch status in mode 1, waiting for current altitude beyond the lanuch_thershold. There is constantly beeping indicating the mode's working condition.
  if (MODE == 1) {
    if (millis() - beep_start >= 3000) {
      psi.buzzer_powerOn(Buzzer_Set);                               //Keeps beeping indicating PSI's working status
      beep_start = millis();
    }
    realPressure =   BMP.readPressure();
    relative_altitude =   BMP.getAltitude(realPressure, referencePressure);
    //absoluteAltitude =   BMP.getAltitude(realPressure);


    if (relative_altitude > launch_threshold) {
      referenceTime = millis();
      myMem.put(address, 0.0f);
      address = psi.EEPROM_Check(address, EEPORM_length);
      myMem.put(address, relative_altitude);
      address = psi.EEPROM_Check(address, EEPORM_length);
      MODE++;
    }
    delay(reading_rate);
  }
  //Mode 2 is flight ascending mode where the aopgee detection runs.
  //apogee detection: There are two samples average greident are taken as the determiniation for apogee. Those tow deteminations have to be both negative to determine the apogee. The first sample has 20 altitude data and the next sample starts from the 11th data of next sample.
  //For each two samples there are 30 data in total. The size of two samples together is called biggerSampleSize. The biggerSampleSize shifts by 1 data each time.
  if (MODE == 2 ) {
    float read_alltitude[sample_size] = {0.0f};
    float read_time[sample_size] = {0.0f};
    read_time_altitude = {0.0f, 0.0f};

    //Reset determinations
    buffer_sum_gradient[0] = gradient_init;
    buffer_sum_gradient[1] = gradient_init;
    sum_gradient = 0.0f;

    eepromStoreData();

    time_altitude.time = time_s_float;
    time_altitude.altitude = relative_altitude;
    OuterSample.push(&time_altitude);

    if (OuterSample.isFull()) {
      for (int i = 0; i < sample_size; i++) {
        OuterSample.peekIdx(&read_time_altitude, i);
        read_alltitude[i] = read_time_altitude.altitude;
        read_time[i] = read_time_altitude.time;
        /*
          Serial.print(i);
          Serial.print(" th,  ");
          Serial.print("time ->");
          Serial.print(read_alltitude[i]);
          Serial.print(", altitude ->  ");
          Serial.println(read_time[i]);
        */
      }
      sum_gradient = getSampleGradient(read_time, read_alltitude, sample_size);
      buffer_sum_gradient[0] = sum_gradient;
      //Serial.print(" 1th sum_gradient = "); Serial.println(sum_gradient);

      for (int i = sample_size; i < sample_size + shift_size; i++) {
        OuterSample.peekIdx(&read_time_altitude, i);
        read_alltitude[i - shift_size] = read_time_altitude.altitude;
        read_time[i - shift_size] = read_time_altitude.time;
        /*
          Serial.print(i);
          Serial.print(" th,  ");
          Serial.print("time ->");
          Serial.print(read_time_altitude.time);
          Serial.print(", altitude ->  ");
          Serial.println(read_time_altitude.altitude);
        */
      }
      sum_gradient = getSampleGradient(read_time, read_alltitude, sample_size);
      buffer_sum_gradient[1] = sum_gradient;
      //Serial.print(" 2nd sum_gradient = "); Serial.println(sum_gradient);
      OuterSample.pop(&forPop);                                                                             //Shifting
    }

    if (buffer_sum_gradient[0] < apogee_threshold && buffer_sum_gradient[1] < apogee_threshold ) {
      digitalWrite(Drogue_Release, HIGH);
      //psi.buzzer_powerOn(Buzzer_Set);
      eepromStoreMark();
      drogue_start_time = millis();
      while (millis() - drogue_start_time  < ignition_duration) {
        eepromStoreData();
        delay(reading_rate);
      }
      digitalWrite(Drogue_Release, LOW);
      MODE++;
    }
    delay(reading_rate / 2);                         //reading_rate of 60 has 15Hz, 30 has 20Hz
  }
  //Mode 3 is the flight descending mode where PSI only looks for the altitude where main prarachute deploys.
  if (MODE == 3 ) {
    eepromStoreData();                               // ms is transferred to second(float)
    if (relative_altitude <= main_release_a) {
      digitalWrite(Main_Release, HIGH);
      eepromStoreMark();
      main_start_time = millis();
      while (millis() - main_start_time  < ignition_duration) {
        eepromStoreData();
        delay(reading_rate);
      }
      digitalWrite(Main_Release, LOW);
      MODE++;
      land_timelimit_start = time_ms;
    }

    delay(reading_rate);
  }

  //Mode 4 is the flight descending mode where PSI only stores altitude data and time.
  if ( MODE == 4 ) {
    float land_altitude[size_of_landSample] = {0.0f};

    if (address < EEPORM_length - 5 && relative_altitude >= land_threshold) {                                                           //Detection of landing and overlap in EEPROM
      eepromStoreData();
    } else MODE = 5;
    land_timelimit_stop = time_ms;

    if (land_timelimit_stop - land_timelimit_start > land_timelimit && landSample.isFull() == 1 ) {
      for (int i = 0; i < size_of_landSample ; i++) {
        landSample.peekIdx(&land_altitude, i);
        land_max_altitude = max(land_altitude[i], land_max_altitude);
        land_min_altitude = min(land_altitude[i], land_min_altitude);
      }
      if (land_max_altitude - land_min_altitude <= land_constant_threshold) {
        MODE = 5;
      }
      land_timelimit_start = land_timelimit_stop;
    }
    landSample.push(&relative_altitude);
    delay(reading_rate);
  }


  //Mode 5 is the flight descending mode where PSI stop storeing anything when either the space for the eeprom runs out or the landing_threshold is reached.
  //Mode 5 has a faster beeping , distinguishing from mode 1
  if (MODE == 5) {
    while (1) {
      psi.buzzer_powerOn(Buzzer_Set);
      delay(reading_rate);
    }
  }
}

void eepromStoreData() {
  realPressure =   BMP.readPressure();
  relative_altitude =   BMP.getAltitude(realPressure, referencePressure);
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);                                              // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address, EEPORM_length);
  myMem.put(address, relative_altitude);
  address = psi.EEPROM_Check(address, EEPORM_length);
}

void eepromStoreMark() {
  realPressure =   BMP.readPressure();
  relative_altitude =   BMP.getAltitude(realPressure, referencePressure);
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);                                               // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address, EEPORM_length);
  if (relative_altitude < 0.00005 && relative_altitude > -0.00005) relative_altitude = 11.111f;
  myMem.put(address, relative_altitude * 1000);
  address = psi.EEPROM_Check(address, EEPORM_length);
}

float getSampleGradient(float x[], float y[], int samplesize) {                                 //Least Squares method to get linear regression
  int SizeofSample = sample_size;
  double sum1 = 0.0, sum2 = 0.0, x_avg = 0.0, y_avg = 0.0;
  double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  for (int i = 0; i < SizeofSample; ++i)
  {
    t1 += x[i] * x[i];
    t2 += x[i];
    t3 += x[i] * y[i];
    t4 += y[i];
  }
  float a = (t3 * SizeofSample - t2 * t4) / (t1 * SizeofSample - t2 * t2);
  float b = (t1 * t4 - t2 * t3) / (t1 * SizeofSample - t2 * t2);
  return a;

}
