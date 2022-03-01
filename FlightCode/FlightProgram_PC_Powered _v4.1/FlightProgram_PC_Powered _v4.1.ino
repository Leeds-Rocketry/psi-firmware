
/*===============================================
  Yudong Xiao, Leeds University Rocket Association(LURA), University of Leeds

  |This code is used for PSI(Pressure Measure Unit) that will be deployed in LURA's rocket, G1 and future rockets.
  PSI is the redundent componets for Easymini. It is supposed to complete tha task of: Reading altitude data from altimeter sensor(MS5611), store the data to EEPORM(24LC512) and detemine when to deploy two parachutes.
  The inteeface protocol is I2c.

  04/12/2021: v4.1: More user interface is added
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
// These are the variables that could be changed based on real life situation.
int battery_threshold = 400; //902;// 7.4/(8.4/1024)=902.095
unsigned int EEPORM_storage = 64000;
unsigned long ignition_duration = 1500; // ematch ingnition time period
float main_release_a = 243.84;
float launch_threshold = 0; // The threshold altitude when EEPROM start to store data.
float land_threshold = -10;  // the threshold altitude when EEPROM decides not to save data
float EEPROM_DrogueMarK = 11111.0;
float EEPROM_MainMarK = 22222.0;
int reading_rate = 10;
float drogue_start_t, drogue_stop_t;
float main_start_t, main_stop_t;

double referencePressure;
float  absoluteAltitude, relativeAltitude;
unsigned long referenceTime;
unsigned long time_ms;
float time_s_float;
float base_count;
long realPressure;
int last_address; // used for moving into next 10 altitude of



//initialization
int MODE = 0;       //initialize mode to zero
int address = 0;
bool ascending = true ;
float  last_a = -5;  //  Initial last altitude. EEPROM statrs to wrok when altitude read is above 2 meters.Used to determine apogee
float error_margin = 1;   // didnt use this
float readAltitude[20] = {0};
float propotional[19] = {0};
float sum_propotional = 0;
float buffer_sum_propotional[2] = {0};

//create objects
MS5611 BMP;
PSI psi;
ExternalEEPROM myMem;

void EEPORM_Data_Store() {
  realPressure =   BMP.readPressure();// Read true Pressure
  relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address);
  myMem.put(address, relativeAltitude);  //mark in EEPROM
  address = psi.EEPROM_Check(address);
}

void EEPORM_Mark_Store(float Mark) {
  time_ms = millis() - referenceTime;
  time_s_float = psi.ms2s(time_ms);   // ms is transferred to second(float)
  myMem.put(address, time_s_float);
  address = psi.EEPROM_Check(address);
  myMem.put(address, Mark);  //mark in EEPROM
  address = psi.EEPROM_Check(address);
}
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
  Serial.println("Sensor detected!");
  digitalWrite(Sensor_Check, HIGH); // Sensor_Check_LED on when BMP inti right
  Serial.println("Ematch is asummed not being connected. Not cheking Ematch connection. If you want to, check the code line 111");
  //Check Ematches connection
  /*
    while (analogRead(Drogue_Connect)==0){
    Serial.println("Drogue Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }
    while (analogRead(Main_Connect)==0){
    Serial.println("Main Ematches not connected, Check!");
    psi.buzzer_ematch(Buzzer_Set);
    }
    digitalWrite(Ematch_Check,HIGH); // Ematches_Check_LED on when ematches connected right
  */
  //Check battery voltage
  int battery_val = 0;
  battery_val = analogRead(Battery_Check);
  while (battery_val <= battery_threshold) {
    Serial.println("Battery voltage too low, Charge!");
    psi.buzzer_powerLow(Buzzer_Set);
  }
  Serial.println("Battery voltage high enough, no need to Charge");

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
  //Read pressure and real altitude;
  if (MODE == 1) {
    // Calculate absolute altitude
    realPressure =   BMP.readPressure();// Read true Pressure
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    absoluteAltitude =   BMP.getAltitude(realPressure);
    Serial.print(" absoluteAltitude = ");
    Serial.print(absoluteAltitude);
    Serial.print(" m, relativeAltitude = ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    Serial.print(" Altitude now lower than ");
    Serial.print(launch_threshold);
    Serial.println(" m");
    Serial.println("Blow air to sensor to get higher altitude and enable EEPROM storing data");

    if (relativeAltitude > launch_threshold) {
      MODE++;                      // Move to mode 2. Ascending flight mode
      //myMem.erase();              //takes huge time. Dont use it.
      referenceTime = millis();   // get referenceTime. So later counter data starts from 0 second.
      myMem.put(address, base_count);
      address = psi.EEPROM_Check(address);  //update address for EEPROM
      myMem.put(address, relativeAltitude);
      address = psi.EEPROM_Check(address);
      Serial.println("EEPROM data writing activated");
      last_a = relativeAltitude;  // Update last altitude
    }
    delay(reading_rate);
  }
  if (MODE == 2 && ascending == 1) { //ACTIVE FLIGHT ascending mode
    EEPORM_Data_Store();
    Serial.print("relativeAltitude = ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    // When addresss >=160, it shows there are 20 altitude data already.
    //Store them to an array and do the caculation to determine if rocket reach the apogee.
    // If not. the array moves to next 10 altitude data (data within 1 second).
    if (address >= 160 && address >= last_address + 80 ) {
      Serial.println("Apogee detecting starts.");
      // 4+8*19 =156.  8(address increment of two altitude data ) = 2 * sizeof(float).
      //80 = 8*10; detecte next round of next 10 altitude.
      for (int i = 0; i < 20; i++) {
        myMem.get(address - 4 - 8 * (19 - i), readAltitude[i]);
      }
      for (int i = 0; i < 20; i++) {

        Serial.print(readAltitude[i]);
        Serial.print("m, ");
        if (i + 1 <= 19) {
          propotional[i] = (readAltitude[i + 1] - readAltitude[0]) / (i + 1);
          Serial.print("gradient = " );
          Serial.print(propotional[i]);
          Serial.print(", ");
        }
        if (i < 19)sum_propotional += propotional[i];
        Serial.print("sum of gradient = " );
        Serial.print(sum_propotional);
        Serial.print(", ");

        Serial.print("order: " );
        Serial.print(i);
        Serial.println(", ");
      }
      last_address = address;
      buffer_sum_propotional[1] = buffer_sum_propotional[0];
      buffer_sum_propotional[0] = sum_propotional;
      sum_propotional = 0;
      if (buffer_sum_propotional[0] < 0 && buffer_sum_propotional[1] < 0) { //apogee detection method now
        digitalWrite(Drogue_Release, HIGH);
        EEPORM_Mark_Store(EEPROM_DrogueMarK);
        Serial.println("Apogee detected.");
        Serial.println(" Drogue released  ");     // For tesing. Can be deleted for real flight
        while (millis() - drogue_start_t < ignition_duration) {
          //Serial.println(drogue_start_t - millis());
          //Serial.println(ignition_duration/1000);
          EEPORM_Data_Store();
          delay(reading_rate);
        }
      }
      digitalWrite(Drogue_Release, LOW);
      ascending = false;
      MODE++;                                  // Move to mode 3, flight descdending mode;
    }
    delay(reading_rate);
  }
  if (MODE == 3 && ascending == false) { //RECOVERY MODE
    EEPORM_Data_Store();
    Serial.print("relativeAltitude = ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    if (relativeAltitude < main_release_a) {    // Determine when to deploy main parachute
      digitalWrite(Main_Release, HIGH);

      EEPORM_Mark_Store(EEPROM_MainMarK);
      Serial.print(" Altitude lower than Main Release Altitude: ");
      Serial.print(main_release_a);
      Serial.println(" m");

      main_start_t = millis();
      while (millis() - main_start_t  < ignition_duration) {
        EEPORM_Data_Store();
        delay(reading_rate);
      }
      Serial.println(" Main released!  ");
      digitalWrite(Main_Release, LOW);
      MODE++; // Move to MODE 4, recording descdending MODE;
    }
    delay(reading_rate);
  }

  if (MODE == 4 && ascending == false) { //RECOVERY  RECORDING MODE
    realPressure =   BMP.readPressure();// Read true Pressure
    relativeAltitude =   BMP.getAltitude(realPressure, referencePressure);
    Serial.print("relativeAltitude = ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    time_ms = millis() - referenceTime;
    Serial.print("millis = ");
    Serial.print(millis());
    Serial.println(" ms");
    Serial.print("time_ms = ");
    Serial.print(time_ms);
    Serial.println(" ms");
    time_s_float = psi.ms2s(time_ms);
    Serial.print("time_s_float = ");
    Serial.print(time_s_float);
    Serial.println(" s");
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
  if (MODE == 5) {// When EEPORM full, freeze it here.
    Serial.print(" Altitude lower than Land Threshold Altitude: ");
    Serial.print(land_threshold);
    Serial.println(" m");
    Serial.println("EEPROM Writing Inactivated");
    while (1);
  }

}
