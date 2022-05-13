//04/12/2021: v4.1: More user interface is added

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
  //--- Serial Debugging ---
  Serial.begin(9600);
  Serial.println("REBOOT");
  //--- Establish Pin Modes and turn off all LEDs ---
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

  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time

  Serial.print(" launch threshold: ");
  Serial.print(launch_threshold);
  Serial.println(" m");

  Serial.print(" land threshold: ");
  Serial.print(land_threshold);
  Serial.println(" m");

  Serial.print("Reading rate ");
  Serial.print(reading_rate);
  Serial.println(" ms");

  MODE                = 1;
  EEPORM_length       = myMem.length();
  shift_size          = sample_size;                                      
  beep_start          = millis();
  size_of_landSample  = round(land_timelimit / ((reading_rate + 10) / 1000));
  
}


void loop() {
  if (MODE == 1) {
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    if (millis() - beepStart >= 3000) {
      psi.buzzer_powerOn(Buzzer_Set);                               //Keeps beeping indicating PSI's working status
      beepStart = millis();
    }
    Serial.print(time_s_float);
    Serial.print(" s, ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    if (relativeAltitude > launch_threshold) {
      MODE++;
    }
    delay(reading_rate);
  }

  if (MODE == 2 ) { //ACTIVE FLIGHT_TEST mode
    //Serial.println("It is now ascending.");

    float readAltitude[sample_size] = {0.0f};
    float readTime[sample_size] = {0.0f};
    readTimeAltitude = {0.0f, 0.0f};
    buffer_sum_gradient[0] = gradient_init;
    buffer_sum_gradient[1] = gradient_init;
    sum_gradient = 0.0f;
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    Serial.print("Time -> "); Serial.print(time_s_float); Serial.print(", Altitude -> "); Serial.println(relativeAltitude);
    /*
        Serial.print(time_s_float);
        Serial.print(" s, ");
        Serial.print(relativeAltitude);
        Serial.println(" m");
    */
    time_altitude.time = time_s_float;
    time_altitude.altitude = relativeAltitude;

    OuterSample.push(&time_altitude);

    if (OuterSample.isFull()) {
      //Serial.println("apogee detection going on");
      Serial.print(" At address ");
      Serial.print(address / 8);
      Serial.println(" ");

      for (int i = 0; i < sample_size; i++) {
        OuterSample.peekIdx(&readTimeAltitude, i);
        readAltitude[i] = readTimeAltitude.altitude;
        readTime[i] = readTimeAltitude.time;
        /*
          Serial.print(i);
          Serial.print(" th,  ");
          Serial.print("time ->");
          Serial.print(readTimeAltitude.time);
          Serial.print(", altitude ->  ");
          Serial.println(readTimeAltitude.altitude);
        */
      }
      sum_gradient = getSampleGradient(readTime, readAltitude, sample_size);
      Serial.print("1 sum_gradient = "); Serial.println(sum_gradient);
      /*Serial.print("For the first sample, the data are\n");
      for (int i = 0; i < sample_size; i++) { //i == 0 -19
        Serial.print(readTime[i]);Serial.println(", " );
      }
      for (int i = 0; i < sample_size; i++) { //
        Serial.print(readAltitude[i]);Serial.println("," );
      }*/
      buffer_sum_gradient[0] = sum_gradient;
      for (int i = sample_size; i < sample_size + sample_size; i++) {
        OuterSample.peekIdx(&readTimeAltitude, i);
        readAltitude[i - sample_size] = readTimeAltitude.altitude;
        readTime[i - sample_size] = readTimeAltitude.time;
        /*
          Serial.print(i);
          Serial.print(" th,  ");
          Serial.print("time ->");
          Serial.print(readTimeAltitude.time);
          Serial.print(", altitude ->  ");
          Serial.println(readTimeAltitude.altitude);
        */
      }
      sum_gradient = getSampleGradient(readTime, readAltitude, sample_size);
      Serial.print("2 sum_gradient = "); Serial.println(sum_gradient);
      buffer_sum_gradient[1] = sum_gradient;

      /*Serial.print("For the second sample, the data are\n");
      for (int i = 0; i < sample_size; i++) { //i == 0 -19
        Serial.print(readTime[i]);Serial.println(", " );
      }
      for (int i = 0; i < sample_size; i++) { //i == 0 -19
        Serial.print(readAltitude[i]);Serial.println(", " );
      }*/
      OuterSample.pop(&forPop);
    }

    if (buffer_sum_gradient[0] < apogee_threshold && buffer_sum_gradient[1] < apogee_threshold) { //apogee detection method now
      digitalWrite(Drogue_Release, HIGH);
      Serial.println("Apogee detected.");
      Serial.print("At address "); Serial.print(address / 8); Serial.print(" Time -> "); Serial.println(time_s_float); Serial.print(", Altitude -> "); Serial.println(relativeAltitude);

      Serial.println(" Drogue released  ");     // For tesing. Can be deleted for real flight
      delay(ignition_duration);
      digitalWrite(Drogue_Release, LOW);


      MODE++;                                  // Move to mode 3, flight descdending mode;
    }
    delay(reading_rate);
  }
  if (MODE == 3 ) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now Descending.");
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address, EEPROM_storage);

    Serial.print(time_s_float);
    Serial.print(" s, ");
    Serial.print(relativeAltitude);
    Serial.println(" m");

    if (relativeAltitude < main_release_a) {
      digitalWrite(Main_Release, HIGH);
      Serial.println("Main deployed");
      delay(ignition_duration);
      digitalWrite(Main_Release, LOW);
      MODE++; // Move to mode 3, flight descdending mode;
      land_timelimit_start = time_ms;
    }
    delay(reading_rate);
  }
  if (MODE == 4 ) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now Descending");
float land_altitude[size_of_landSample] = {0.0f};
if (address < EEPORM_storage - 5 && relativeAltitude >= land_threshold) {                                                           //Detection of landing and overlap in EEPROM
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address, EEPROM_storage);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address, EEPROM_storage);

    Serial.print(time_s_float);
    Serial.print(" s, ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
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
  if (MODE == 5) {// When EEPORM full, freeze it here.
    Serial.print(" Altitude lower than Land Threshold Altitude: ");
    Serial.print(land_threshold);
    Serial.println(" m");
    Serial.println("EEPROM Writing Inactivated");
    while (1);
  }
}

float getSampleGradient(float x[], float y[], int samplesize) {                                 //Least Squares method to get linear regression
  int SizeofSample = samplesize;
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

  /*
    x_avg = accumulate(x) / SizeofSample;
    y_avg = accumulate(y) / SizeofSample;
    for (int i = 0; i < SizeofSample; ++i){
    sum1 += (x[i] * y[i] - x_avg * y_avg);
    sum2 += (x[i] * x[i] - x_avg * x_avg);
    }

    return y_avg - sum1 / sum2 * x_avg;
  */
}
