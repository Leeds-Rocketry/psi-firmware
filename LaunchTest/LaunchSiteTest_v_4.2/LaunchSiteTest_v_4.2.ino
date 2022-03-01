//04/12/2021: v4.1: More user interface is added


#include <Arduino.h>
#include <Wire.h>
#include "PSI.h"
#include "MS5611.h"
#include <SparkFun_External_EEPROM.h> //download link: https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library.git


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
// These are the variables that could be changed based on real life situation.
int battery_threshold = 400; //902;// 7.4/(8.4/1024)=902.095
unsigned int EEPORM_storage = 63999;
unsigned long ignition_duration = 1500;
float main_release_a = 243.84;
float launch_threshold = 2; // The threshold altitude when EEPROM start to store data.
float land_threshold = -3;
int reading_rate = 10;
int sample_size = 20;
int shift_size = 5;
int block_size = 10; // block size is normally half of sample size

double referencePressure;
float  absoluteAltitude, relativeAltitude;
unsigned long referenceTime;
unsigned long time_ms;
float time_s_float;
float base_count;
long realPressure;
int value;
float myRead_time;
float myRead_altitude;
int last_address;

//initialization
int MODE = 0;       //initialize mode to zero
int address = 0;
bool ascending = true ;
float  last_a = -5;  //  Initial last altitude. EEPROM statrs to wrok when altitude read is above 2 meters.Used to determine appoage
float error_margin = 1;
float readAltitude[20] = {0};
float readAltitude_LPfiltered[20] = {0};
float propotional[19] = {0};
float sum_propotional = 0;
float buffer_sum_propotional[3] = {0};

MS5611 BMP;
PSI psi;
ExternalEEPROM myMem;

int size_TestData;

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
  //psi.buzzer_powerOn(Buzzer_Set);
  //clear EEPORM
  //initialize BMP sensor
  Wire.begin();
  Wire.setClock(400000); //Most EEPROMs can run 400kHz and higher
  //Detect status of EEPROM, Sensor, ematches and powerLow. Each has repective tone for buzzing when it goes wrong.
  // Tone: EEPROM(...), Sensor(_..), ematches(_ _.), powerLow(_ _ _)
  //Check EEPROM
  /*
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
    Serial.println("Sensor detected!");
    Serial.println("Ematch is asummed not being connected. Not cheking Ematch connection. If you want to, check the code line 90");
    //Check Ematches connection

    while (analogRead(Drogue_Connect)==0){
    Serial.println("Drogue Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }
    while (analogRead(Main_Connect)==0){
    Serial.println("Main Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }

    digitalWrite(Ematch_Check,HIGH); // Ematches_Check_LED on when ematches connected right

    //Check battery voltage
    int battery_val = 0;
    battery_val = analogRead(Battery_Check);
    while (battery_val <= battery_threshold) {
    Serial.println("Battery voltage too low, Charge!");
    psi.buzzer_powerLow(Buzzer_Set);
    }
    // Get reference pressure for relative altitude

  */
  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time
  ascending = true;
  address = 0;
  MODE = 1;
  relativeAltitude = 0;
  base_count = 0;



  Serial.print(" launch threshold: ");
  Serial.print(launch_threshold);
  Serial.println(" m");

  Serial.print(" land threshold: ");
  Serial.print(land_threshold);
  Serial.println(" m");

  Serial.print("Reading rate ");
  Serial.print(reading_rate);
  Serial.println(" ms");
  /*
    for (int i =0;i<6400;){
     myMem.get(i, myRead_time);
     Serial.print(i, DEC); Serial.print("-> ");Serial.print(myRead_time, DEC); Serial.print("-> ");
     i = psi.EEPROM_Check(i);
     myMem.get(i, myRead_time);
     Serial.println(myRead_time, DEC);
     i = psi.EEPROM_Check(i);
    }
    while(1){}
  */
}


void loop() {
  if (MODE == 1) {
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
    Serial.print(relativeAltitude);
    Serial.println(" m");
    if (relativeAltitude > launch_threshold) {
      MODE++;
    }
    delay(reading_rate);
  }

  if (MODE == 2 && ascending == true) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now ascending.");
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
    Serial.print(relativeAltitude);
    Serial.println(" m");
    sum_propotional = 0; // initial sum
    if (address >= sample_size * 8 && address >= last_address + shift_size * 4 ) {
      Serial.println("Apogee detecting going on.");
      // 4+8*19 =156.  8(address increment of two altitude data ) = 2 * sizeof(float).
      //80 = 8*10; detecte next round of next 10 altitude.
      for (int i = 0; i < sample_size; i++) {
        myMem.get(address - 4 - 8 * (sample_size - 1 - i), readAltitude[i]);

      }
      float xm1 = 0;
      xm1 = psi.simplp(readAltitude, readAltitude_LPfiltered, block_size, xm1);
      xm1 = psi.simplp(&readAltitude[block_size], &readAltitude_LPfiltered[block_size], block_size, xm1);
      readAltitude_LPfiltered[0] = 2 * readAltitude_LPfiltered[0];
      /*
      for (int i = 0; i < sample_size; i++) {
        if (readAltitude[i] == 0) {
          Serial.print("address is"); Serial.println(address);
          Serial.print("readAltitude["); Serial.print(i); Serial.print("] = "); Serial.println(readAltitude[i]);
          Serial.print("readAltitude_LPfiltered["); Serial.print(i); Serial.print("] = "); Serial.println(readAltitude_LPfiltered[i]);
        }
      }
       */
      for (int i = 0; i < sample_size; i++) {

        /*
          Serial.print(readAltitude[i]);
          Serial.print("m, ");
        */
        if (i  <= sample_size - 1) {
          //propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);
          propotional[i] = (readAltitude_LPfiltered[i + 1] - readAltitude_LPfiltered[0]) / (i + 1);
          sum_propotional += propotional[i];
          /*
            Serial.print(propotional[i]);
            Serial.print("gradient = " );
            Serial.print(", ");
          */
        }
        /*
          Serial.print("sum of gradient = " );
          Serial.print(sum_propotional);
          Serial.print(", ");
 /*
          Serial.print(i);
          Serial.println("th data," );
        */
      }
      buffer_sum_propotional[1] = buffer_sum_propotional[0];
      buffer_sum_propotional[0] = sum_propotional;
      last_address = address;//detecte next round of next 10 altitude.
    }
    if (buffer_sum_propotional[0] < 0 && buffer_sum_propotional[1] < 0) { //apogee detection method now
      digitalWrite(Drogue_Release, HIGH);
      Serial.println("Apogee detected.");
      Serial.println(" Drogue released  ");     // For tesing. Can be deleted for real flight
      delay(ignition_duration);
      digitalWrite(Drogue_Release, LOW);
      ascending = false;
      MODE++;                                  // Move to mode 3, flight descdending mode;
    }
    delay(reading_rate);
  }
  if (MODE == 3 && ascending == false) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now Descending.");
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
    Serial.print(relativeAltitude);
    Serial.println(" m");
    if (relativeAltitude < main_release_a) {
      digitalWrite(Main_Release, HIGH);
      Serial.println("Main deployed");
      delay(ignition_duration);
      digitalWrite(Main_Release, LOW);
      MODE++; // Move to mode 3, flight descdending mode;
    }
    delay(reading_rate);
  }
  if (MODE == 4 && ascending == false) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now Descending");
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
    Serial.print(relativeAltitude);
    Serial.println(" m");
    if (address == 0) MODE = 5;
    if (relativeAltitude < land_threshold || address == 0 ) {    //After landed. Stop transferring data to EEPROM.
      MODE = 5;
    }
    delay(reading_rate);
  }
  if (MODE == 5) {// When EEPORM full, freeze it here.
    Serial.print(" Altitude lower than Land Threshold Altitude: ");
    Serial.print(land_threshold);
    Serial.println(" m");
    Serial.println("EEPROM Writing Inactivated");
    while (1);
  }
}
