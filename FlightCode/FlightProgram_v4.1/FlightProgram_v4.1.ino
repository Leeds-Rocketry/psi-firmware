
/*===============================================
  Yudong Xiao, Leeds University Rocket Association(LURA), University of Leeds

  |This code is used for PSI(Pressure Measure Unit) that will be deployed in LURA's rocket, G1 and future rockets.
  PSI is the redundent componets for Easymini. It is supposed to complete tha task of: Reading altitude data from altimeter sensor(MS5611), store the data to EEPORM(24LC512) and detemine when to deploy two parachutes.

  04/12/2021 V4.1: Tips to check battery_threshold is added.
  ==============================================
*/
#include <Arduino.h>
#include <Wire.h>         //includes Wire.h library for I2C interface
#include "MS5611.h"
#include "PSI.h"         // Specifical LURA PSI library.
#include <SparkFun_External_EEPROM.h>   //External EEPORM library. download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git

//define relevant pins
#define Drogue_Release 2   //set Drogue_Release pin
#define Main_Release 3   //set Maine_Release pin
#define Ematch_Check 6
#define Sensor_Check 7
#define Buzzer_Set 8
#define Main_Connect A0   //set Maine_Release pin
#define Battery_Check A6
#define Drogue_Connect A7   //set Drogue_Release pin

#define disk1 0x50    //Address of 24LC256 eeprom chip

//declare general use variables
int battery_threshold = 902;// 7.4/(8.4/1024)=902.095
unsigned int EEPORM_storage = 64000;
unsigned long ignition_duration = 1500; // ematch ingnition time period
float main_release_a = 80;

float launch_threshold = 0.5; // The threshold altitude when EEPROM start to store data.
float land_threshold = -3;  // the threshold altitude when EEPROM decides not to save data

float EEPROM_DrogueMarK = 11111.0;
float EEPROM_MainMarK = 22222.0;
int reading_rate = 100;
float drogue_start_t;
float main_start_t;

double referencePressure;
float  absoluteAltitude, relativeAltitude;
unsigned long referenceTime;
unsigned long time_ms;
float time_s_float; //changes
float base_count;
long realPressure;
int last_address; // used for moving into next 10 altitude of



//initialization
int MODE = 0;       //initialize mode to zero
int address = 0;
bool ascending = true ;
float readAltitude[20] = {0};   //Has 20 altitude data to determine if rocket reaches appoage
float propotional[19] = {0};
float sum_propotional = 0;
float buffer_sum_propotional[3] = {0};

//create objects
MS5611 BMP;
PSI psi;
ExternalEEPROM myMem;

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(BMP.getOversampling());
}

void setup() {
  //--- Serial Debugging ---
  Serial.begin(9600);
  Serial.println("REBOOT");
  //--- Establish Pin Modes and turn off all LEDs ---
  pinMode(Drogue_Release, OUTPUT);
  pinMode(Main_Release, OUTPUT);
  pinMode(Drogue_Connect, INPUT);
  pinMode(Main_Connect, INPUT);
  pinMode(Battery_Check, INPUT);
  pinMode(Buzzer_Set, OUTPUT);
  pinMode(Ematch_Check, OUTPUT);
  pinMode(Sensor_Check, OUTPUT);
  digitalWrite(Ematch_Check, LOW);
  digitalWrite(Sensor_Check, LOW);
  psi.buzzer_powerOn(Buzzer_Set);
  //clear EEPORM
  //initialize BMP sensor
  Wire.begin();
  Wire.setClock(400000); //Most EEPROMs can run 400kHz and higher
  //Detect status of EEPROM, Sensor, ematches and powerLow. Each has repective tone for buzzing when it goes wrong.
  // Tone: EEPROM(...), Sensor(_..), ematches(_ _.), powerLow(_ _ _)
  //Check EEPROM
  while (myMem.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
    psi.buzzer_EEPROM(Buzzer_Set);
  }
  Serial.println("Memory detected!");
  // Check Sensor
  Serial.println("Initialize MS5611 Sensor");
  while (!BMP.begin())
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    psi.buzzer_sensor(Buzzer_Set);
  }
  digitalWrite(Sensor_Check, HIGH); // Sensor_Check_LED on when BMP inti right
  /*Check Ematches connection
    if (analogRead(Drogue_Connect) == 0) {
    Serial.println("Drogue Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }
    if (analogRead(Main_Connect) == 0) {
    Serial.println("Main Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }
  */
  digitalWrite(Ematch_Check, HIGH); // Ematches_Check_LED on when ematches connected right
  //Check battery voltage
  int battery_val = 0;
  battery_val = analogRead(Battery_Check);
  while (battery_val <= battery_threshold) {
    Serial.println("Battery voltage too low, Charge!");
    Serial.println("Or Check the variable Battery_threshold");
    psi.buzzer_powerLow(Buzzer_Set);
  }

  // Get reference pressure for relative altitude
  referencePressure = BMP.readPressure();
  // Check settings for EEPROM
  checkSettings();
  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time
  ascending = true;
  address = 0;
  MODE = 1;
  relativeAltitude = 0;
  base_count = 0; // Start to store time data when altitude reach 2m. Start to count time from 0, base_count.
}

void loop() {
  if (MODE == 1) {
    delay(reading_rate*10);
    //Read pressure and real altitude;
    // Calculate absolute altitude
    realPressure =   BMP.readPressure();// Read true Pressure
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    absoluteAltitude =   BMP.getAltitude(realPressure);
    psi.buzzer_powerOn(Buzzer_Set);
    if (relativeAltitude > launch_threshold) {
      MODE++;                      // Move to mode 2. Ascending flight mode
      //myMem.erase();              //takes huge time. Dont use it.
      referenceTime = millis();   // get referenceTime. So later counter data starts from 0 second.
      myMem.put(address, base_count);
      address = psi.EEPROM_Check(address);  //update address for EEPROM
      myMem.put(address, relativeAltitude);
      address = psi.EEPROM_Check(address);
    }

  }
  if (MODE == 2 && ascending == 1) { //ACTIVE FLIGHT ascending mode
    //Get time and altitude and put them to EEPROM
    realPressure =   BMP.readPressure();// Read true Pressure
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    time_ms = millis() - referenceTime;
    time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float)
    myMem.put(address, time_s_float);
    address = psi.EEPROM_Check(address);
    myMem.put(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
    /*
    // When addresss >=160, it shows there are 20 altitude data already. Put them in a window and calculate the overall linear approximation  to determine the appeage.
    // If not. the window shifts to next 10 altitude data (data within 0.5 second).
    //window size 20 data. Shift by 10 data
    if (address >= 160 && address >= last_address + 80 ) { // Allocating address. 8*20 =160.  8(address increment of two altitude data ) = 2 * sizeof(float). 80 = 8*10; shift the window to the next 10 altitude.
      for (int i = 0; i < 20; i++) {
        myMem.get(address - 4 - 8 * (19 - i), readAltitude[i]); //just get altitude at allocated address and put them in to the 20 number size window
      }
      for (int i = 0; i < 20; i++) {
        if (i + 1 <= 19) {
          propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1); //Calculating every data's linear approximation compared to the first data.
        }
        if (i < 19)sum_propotional += propotional[i]; //Sum up every data's linear approximation to get this window's whole linear approximation 
      }
      */
      /*
//    window size 30 data. Shift by 10 data
      if (address >= 240 && address >= last_address + 80 ) { // Start with first 30 data. The address start from 8*30 =240. 8(address increment of two altitude data ) = 2 * sizeof(float). 80 = 8*10; detecte next round of next 10 altitude.
        for (int i = 0; i < 30; i++) {
          myMem.get(address - 4 - 8 * (29 - i), readAltitude[i]);
        }
        for (int i = 0; i < 30; i++) {
          if (i + 1 <= 29) {
            propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);
          }
          if (i < 29)sum_propotional += propotional[i];
        }
*/
   // window size 10 data. Shift by 5 data
      if (address >= 160 && address >= last_address + 8 ) { //Start with first 20 data. The address start from 8*20 =160. 8(address increment of two altitude data ) = 2 * sizeof(float). 80 = 8*10; detecte next round of next 10 altitude.
        for (int i = 0; i < 10; i++) {
          myMem.get(address - 4 - 8 * (9 - i), readAltitude[i]);
        }
        for (int i = 0; i < 10; i++) {
          if (i + 1 <= 9) {
            propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);
          }
          if (i < 9)sum_propotional += propotional[i];
        }

/*    window size 20 data. Shift by 1 data
      if (address >= 160 && address >= last_address + 8 ) { // Start with first 20 data. The address start from 8*20 =160.  8(address increment of two altitude data ) = 2 * sizeof(float). 80 = 8*10; detecte next round of next 10 altitude.
        for (int i = 0; i < 20; i++) {
          myMem.get(address - 4 - 8 * (19 - i), readAltitude[i]);
        }
        for (int i = 0; i < 20; i++) {
          if (i + 1 <= 19) {
            propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);
          }
          if (i < 19)sum_propotional += propotional[i];
        }
*/
        buffer_sum_propotional[1] = buffer_sum_propotional[0];
        buffer_sum_propotional[0] = sum_propotional;
        last_address = address;//detecte next round of next 10 altitude.
      }
      if (buffer_sum_propotional[0] < 0 && buffer_sum_propotional[1] < 0 ) { //apogee detection method now
        digitalWrite(Drogue_Release, HIGH);
        psi.buzzer_powerOn(Buzzer_Set);
        time_ms = millis() - referenceTime;
        time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float)
        myMem.put(address, time_s_float);
        address = psi.EEPROM_Check(address);
        myMem.put(address, EEPROM_DrogueMarK);  //mark in EEPROM
        address = psi.EEPROM_Check(address);
        drogue_start_t = millis();
        while (millis() - drogue_start_t  < ignition_duration) {
          realPressure = BMP.readPressure();// Read true Pressure
          relativeAltitude = BMP.getAltitude(realPressure, referencePressure);
          time_ms = millis() - referenceTime;
          time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float
          myMem.put(address, time_s_float);
          address = psi.EEPROM_Check(address);
          myMem.put(address, relativeAltitude);
          address = psi.EEPROM_Check(address);
          delay(reading_rate);
        }
        digitalWrite(Drogue_Release, LOW);
        ascending = false;
        MODE++;                                  // Move to mode 3, flight descdending mode;
      }
      delay(reading_rate);
    }
    if (MODE == 3 && ascending == false) { //RECOVERY MODE
      realPressure =   BMP.readPressure();// Read true Pressure
      relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
      time_ms = millis() - referenceTime;
      time_s_float = psi.ms2s(time_ms);
      myMem.put(address, time_s_float);
      address = psi.EEPROM_Check(address);
      myMem.put(address, relativeAltitude);
      address = psi.EEPROM_Check(address);
      Serial.print(relativeAltitude);
      Serial.println(" m");
      if (relativeAltitude < main_release_a) {    // Determine when to deploy main parachute
        digitalWrite(Main_Release, HIGH);
        time_ms = millis() - referenceTime;
        time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float)
        myMem.put(address, time_s_float);
        address = psi.EEPROM_Check(address);
        myMem.put(address, EEPROM_MainMarK);
        address = psi.EEPROM_Check(address);
        main_start_t = millis();
        while (millis() - main_start_t  < ignition_duration) {
          realPressure = BMP.readPressure();// Read true Pressure
          relativeAltitude = BMP.getAltitude(realPressure, referencePressure);
          time_ms = millis() - referenceTime;
          time_s_float = psi.ms2s(time_ms);
          myMem.put(address, time_s_float);
          address = psi.EEPROM_Check(address);
          myMem.put(address, relativeAltitude);
          address = psi.EEPROM_Check(address);
          delay(reading_rate);
        }
        digitalWrite(Main_Release, LOW);
        MODE++; // Move to MODE 4, recording descdending MODE;
      }
      delay(reading_rate);
    }

    if (MODE == 4 && ascending == false) { //RECOVERY  RECORDING MODE
      realPressure =   BMP.readPressure();// Read true Pressure
      relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
      time_ms = millis() - referenceTime;
      time_s_float = psi.ms2s(time_ms);
      if (address == 0) MODE = 5;
      myMem.put(address, time_s_float);
      address = psi.EEPROM_Check(address);
      if (address == 0) MODE = 5;
      myMem.put(address, relativeAltitude);
      address = psi.EEPROM_Check(address);
      if (address == 0) MODE = 5;
      if (relativeAltitude < land_threshold || address == 0 ) {    //After landed. Stop transferring data to EEPROM.
        MODE = 5;
      }
      delay(reading_rate);
    }
    if (MODE == 5) {    // When EEPORM full, freeze it here.
      while (1){    
        psi.buzzer_powerOn(Buzzer_Set);
        delay(reading_rate*10);
        }
    }

  }
