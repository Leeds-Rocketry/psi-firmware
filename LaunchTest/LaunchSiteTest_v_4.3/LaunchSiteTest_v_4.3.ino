//04/12/2021: v4.1: More user interface is added


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
#define EEPROM_storage  64000                                  //512k 24LC256 eeprom chip has 64k bytes
#define ignition_duration  1500                                //Time period for ematches to be ignited. In ms
#define reading_rate  1                                      //The time delayed for each iteration. 100 gives about 10 Hz reading rate from the Drone test 

#define sample_size  15                                       //The number of data used to determine the apogee point
#define shift_size  5                                         //The numerr of data skiped until next sample starts

#define main_release_a  80.0f                                 //The altitude for main parachute to be deployed. 800ft for SAC
#define launch_threshold  0.5f                                // The threshold altitude when EEPROM starts storing data
#define land_threshold  60.0f                                 // The threshold altitude when EEPROM stop saving data
#define apogee_threshold 0.05f
#define gradient_init 1.0f

typedef struct strData {
  float time;
  float altitude;
} Data;

//---Declare general variables. Can be changed according to different flight condition.---
bool enableEmatchCheck = false;

//Variables used for sensor
double referencePressure = 0.0;
float  absoluteAltitude = 0.0;
float relativeAltitude = 0.0;
long realPressure = 0;

//---Variables for timing data---
unsigned long referenceTime = 0;
unsigned long time_ms = 0;                                       //Time in milisecond. Unsigned long has 4 bytes
float time_s_float = 0.0;                                        //Time in second.

float drogue_start_time = 0.0;                                   //Time for Drogue e match starts to be ignited
float main_start_time = 0.0;                                     //Time for Drogue e match starts to be ignited

//---Initialization---
int MODE = 0;                                                 //Initialize flight mode to mode 0;
int address = 0;                                              //EEPROM starting address
int address_count = 0;
bool ascending = true ;                                       //Initial flight condition is ascending
//---Initialization for apogee detection algorithm---
float sum_gradient = 0.0;                                       //Sum of gradients in each sample
//float buffer_sum_gradient[3] = {0.0};                           //Up to 3 samples's gradients together determine the apogee point when all the gradients < 0
//---Others---
int last_address = 0;
int battery_val = 0;

//float accumulate(float sample[]);
float getSampleGradient(float x[], float y[], int samplesize);

Data time_altitude = {0.0, 0.0};
float buffer_sum_gradient[2] = {0.0};
Data readTimeAltitude = {0.0, 0.0};
Data forPop = {0.0, 0.0};

cppQueue  OuterSample(sizeof(Data), sample_size + shift_size, IMPLEMENTATION); // Instantiate queue
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
  myMem.setMemorySize(512000 / 8); //In bytes. 512kbit = 64kbyte
  myMem.setPageSize(128); //In bytes. Has 128 byte page size.
  myMem.enablePollForWriteComplete(); //Supports I2C polling of write completion
  myMem.setPageWriteTime(3); //3 ms max write time
  referencePressure = BMP.readPressure();
  MODE = 1;


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
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
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

    float readAltitude[sample_size] = {0.0};
    float readTime[sample_size] = {0.0};
    buffer_sum_gradient[2] = {0.0};
    readTimeAltitude = {0.0, 0.0};
    buffer_sum_gradient[0] = gradient_init;
    buffer_sum_gradient[1] = gradient_init;
    sum_gradient = 0.0;

    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);
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
      buffer_sum_gradient[0] = sum_gradient;
      for (int i = shift_size; i < sample_size + shift_size; i++) {
        OuterSample.peekIdx(&readTimeAltitude, i);
        readAltitude[i - shift_size] = readTimeAltitude.altitude;
        readTime[i - shift_size] = readTimeAltitude.time;
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
      OuterSample.pop(&forPop);
    }

    if (buffer_sum_gradient[0] < 0 && buffer_sum_gradient[1] < 0) { //apogee detection method now
      digitalWrite(Drogue_Release, HIGH);
      Serial.println("Apogee detected.");
      Serial.print("At address "); Serial.println(address / 8); Serial.print("Time -> "); Serial.println(time_s_float); Serial.print(", Altitude -> "); Serial.println(relativeAltitude);
      while (1);
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
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);

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
    }
    delay(reading_rate);
  }
  if (MODE == 4 && ascending == false) { //ACTIVE FLIGHT_TEST mode
    Serial.println("It is now Descending");
    myMem.get(address, time_s_float);
    address = psi.EEPROM_Check(address);
    myMem.get(address, relativeAltitude);
    address = psi.EEPROM_Check(address);

    Serial.print(time_s_float);
    Serial.print(" s, ");
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
