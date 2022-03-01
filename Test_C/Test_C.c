#include <stdio.h>
#include <stdbool.h>
float test_data[400]={
6.124791145,
11.14775372,
11.65771294,
12.50852489,
12.84937763,
13.87193775,
14.12559605,
14.63819695,
15.48900986,
16.34246445,
16.76786995,
17.61868286,
17.87498283,
18.64124298,
18.81034851,
19.74835587,
20.17376137,
20.60181046,
20.94002151,
21.28351593,
22.04977608,
22.64692879,
23.41318893,
23.66948891,
24.77660179,
25.11745644,
25.63005638,
26.6526165,
27.16257477,
27.50342941,
28.52863121,
29.37944412,
29.8074913,
30.91460419,
32.02436066,
32.44976425,
33.39041519,
33.64671707,
34.07212067,
35.2664299,
35.43553543,
36.5452919,
10,
37.39874649,
37.74224091,
38.84935379,
39.87455368,
40.55890274,
40.47171021,
41.24061203,
42.43491745,
43.03471375,
43.46012115,
44.65707016,
44.91337204,
46.79203033,
47.56092834,
47.73003387,
47.64548492,
48.84243393,
48.58613205,
49.95218658,
50.1239357,
50.38023758,
52.08978653,
51.74629211,
52.00259399,
51.91804123,
52.85868835,
53.11499023,
53.37129211,
54.14019394,
54.90909576,
55.42169571,
56.1932373,
56.61864471,
56.96213913,
57.47474289,
58.33083725,
58.15909195,
59.44059372,
60.12494278,
60.80929184,
61.58083725,
62.00888443,
62.60603714,
63.11863708,
64.31822968,
64.57453156,
65.00257874,
65.34343719,
66.28672791,
67.05562592,
67.73997498,
68.08347321,
10,
10,
10,
10,
10,
10,
72.7048111,
73.30460358,
74.16069794,
74.50419617,
75.27574158,
75.87553406,
76.21639252,
77.07512665,
77.41598511,
78.18753052,
78.70277405,
79.30256653,
130,
131,
132,
132,
131,
130,
129,
128,
85.0442276,
85.30052948,
85.81577301,
10,
87.27430725,
87.61780548,
87.87409973,
200,
200,
200,
91.04747772,
91.30641937,
10,
92.67775726,
93.53649902,
94.05174255,
94.56698608,
200,
200,
200,
96.88161469,
97.56860352,
98.68628693,
10,
99.71677399,
100.1448212,
100.3165741,
100.9190063,
101.6060028,
10,
103.2362823,
103.6669769,
104.5257111,
10,
105.727951,
105.9842453,
106.4994888,
107.1891251,
107.7889252,
10,
108.9911575,
109.5935974,
110.0242844,
130,
131,
132,
132,
131,
130,
129,
128,
114.7513123,
115.5254974,
115.5254974,
116.2996826,
116.2996826,
116.9866791,
116.8149261,
117.0738754,
117.589119,
118.1915512,
118.5350494,
118.7067947,
118.8785477,
118.8785477,
119.3092346,
119.5681763,
119.3092346,
119.9116745,
119.7399292,
119.9962234,
119.8244781,
119.4809799,
119.5681763,
119.9962234,
120.6858597,
120.5141144,
120.5986633,
120.2551727,
119.9116745,
119.7399292,
119.2220383,
119.2220383,
118.4478531,
118.6222458,
118.4478531,
117.417366,
117.1584244,
116.4714355,
116.1279373,
115.4383011,
115.6972504,
115.3537521,
114.9230576,
115.0102539,
114.6667633,
113.9771271,
113.8925781,
112.3442001,
112.3442001,
111.9135132,
110.9675751,
110.7112732,
110.2805862,
109.8525391,
109.5090408,
108.3913574,
108.3913574,
107.8761139,
107.3608704,
106.7584381,
106.1559982,
105.727951,
105.4690018,
105.1255112,
104.3539658,
104.1822205,
103.6669769,
103.3208389,
102.8927917,
102.3775482,
101.7777481,
101.8623047,
101.7777481,
101.3470612,
101.3470612,
100.747261,
100.6600647,
100.4883194,
99.88851929,
99.71677399,
99.28608704,
98.94258881,
98.77084351,
98.94258881,
99.02978516,
98.77084351,
98.77084351,
98.3427887,
97.82754517,
97.65579987,
97.05335999,
96.62531281,
95.93832397,
95.33852386,
94.65153503,
93.87998962,
93.70824432,
93.27754974,
93.19300079,
92.50601196,
93.02125549,
93.27754974,
92.93405914,
93.27754974,
93.02125549,
92.16251373,
92.16251373,
91.6472702,
91.73446655,
91.56272125,
91.90621185,
92.33425903,
91.4755249,
90.53223419,
91.56272125,
91.13466644,
91.13466644,
90.01963043,
90.18873596,
89.6761322,
90.44767761,
89.41719055,
89.3326416,
89.16088867,
88.47389984,
88.64564514,
87.78955078,
87.53060913,
87.01800537,
86.75906372,
86.84626007,
86.15927124,
85.55947113,
85.47492218,
85.47492218,
85.47492218,
85.0442276,
84.78792572,
84.18813324,
83.84463501,
84.10093689,
83.75743866,
83.67288971,
83.41658783,
82.55784607,
82.04524994,
82.55784607,
82.04524994,
80.9302063,
80.33041382,
79.73061371,
79.98691559,
79.55886841,
78.10297394,
77.84667206,
76.90338135,
77.15968323,
77.15968323,
76.21639252,
76.73163605,
76.04728699,
75.191185,
74.8476944,
74.41964722,
73.47634888,
73.56354523,
73.22005463,
72.27940369,
72.02046204,
71.59241486,
71.2515564,
71.16436005,
70.13652039,
70.13652039,
70.05196381,
69.2804184,
68.76782227,
67.73997498,
68.93956757,
67.91172028,
67.57086945,
66.28672791,
65.85868073,
65.43062592,
65.6869278,
64.57453156,
64.57453156,
63.71843338,
63.03408813,
62.69323349,
61.32453537,
61.23733902,
60.89648819,
60.55298996,
60.29669189,
58.92799377,
58.84344101,
58.67169189,
57.39019012,
57.04669189,
57.04669189,
56.79039383,
56.79039383,
55.76519012,
55.76519012,
54.9962883,
54.56824112,
53.88389206,
53.54303741,
52.68694305,
52.25889206,
51.83348846,
51.57718658,
51.06458664,
49.61133575,
49.78308105,
49.01153946,
48.92698669,
48.41438293,
48.41438293,
48.0735321,
48.15808487,
47.73003387,
47.56092834,
47.38918304,
46.8765831,
46.62028122,
46.27942657,

};

float main_release_a = 43.84;
float launch_threshold = 2; // The threshold altitude when EEPROM start to store 
float land_threshold = -3;
int reading_rate = 10;
int sample_size = 20;
int shift_size = 5;
int block_size = 200; // block size is normally half of sample size

double referencePressure;
float  absoluteAltitude, relativeAltitude;
unsigned long referenceTime;
unsigned long time_ms;
float time_s_float;
int value;
float myRead_time;
float myRead_altitude;
int last_address;

//initialization
int MODE = 0;       //initialize mode to zero
int address = 0;
bool ascending = true ;

float readAltitude[20] = {0};
float readAltitude_LPfiltered[400] = {0};
float propotional[19] = {0};
float sum_propotional = 0;
float buffer_sum_propotional[3] = {0};
float simplp (float *x,  float *y, int M, float xm1)
{
  int n;
  y[0] = x[0] + xm1;
  for (n=1; n < M ; n++) {
    y[n] =  x[n]  + x[n-1];
  }
  return x[M-1];
}

int main(void) {
MODE = 1;

      float xm1 = 0;
      xm1 = simplp(test_data, readAltitude_LPfiltered, block_size, xm1);
      xm1 = simplp(&test_data[block_size], &readAltitude_LPfiltered[block_size], block_size, xm1);
  /*
      printf("readAltitude_LPfiltered[%d] = %f \n", 21,readAltitude_LPfiltered[21]);
        for (int i = 0; i < 340; i++) {
          printf("address is %d \n",i);
          printf("test_data[%d] = %f \n", i,test_data[i]);
          printf("readAltitude_LPfiltered[%d] = %f \n", i,readAltitude_LPfiltered[i]);
      }
while(1){};
*/

  while(1){

  if (MODE == 1) {
printf("readAltitude_LPfiltered[%d] = %f \n", 24,readAltitude_LPfiltered[24]);
    printf("address is %d \n",address);
    printf("readAltitude = %f \n",readAltitude);
    relativeAltitude = readAltitude_LPfiltered[address];
    //relativeAltitude = test_data[address];
    address += 1;
    printf(" relativeAltitude = %f ",relativeAltitude);
    printf(" m \n");
    if (relativeAltitude > launch_threshold) {
      MODE++;
    }
    last_address = 0;
  }

  if (MODE == 2 && ascending == true) { //ACTIVE FLIGHT_TEST mode
printf("1readAltitude_LPfiltered[%d] = %f \n", 24,readAltitude_LPfiltered[24]);
    printf("It is now ascending. ");
    printf("address is %d ",address);
    relativeAltitude = readAltitude_LPfiltered[address];
    
    //relativeAltitude = test_data[address];
    address += 1;
    //printf("address is %d ",address);
    printf(" relativeAltitude = %f m \n ",relativeAltitude);
    //printf("readAltitude_LPfiltered[%d] = %f \n", address,readAltitude_LPfiltered[address]);
    //if(address == 21){printf("readAltitude_LPfiltered[%d] = %f \n", address,readAltitude_LPfiltered[address]);while(1){};}
    sum_propotional = 0; // initial sum
    //printf("address is %d ",address);
    //printf("sample_size = %d \n",sample_size);
    
  //printf("last_address + shift_size * 1 is %d \n",last_address  + shift_size * 1 );
  //while(1){};
    if (address >= sample_size * 1 && address >= last_address + shift_size * 1 ) {
      
      printf("Apogee detecting going on. \n");
      printf("address is %d ",address);
      for (int i = 0; i < sample_size; i++) { //i == 0 -19
        //readAltitude[i] = test_data[address - 1 - 2 * (sample_size - 1 - i)];
        readAltitude[i] = readAltitude_LPfiltered[address - (sample_size - i)];
        printf("2readAltitude_LPfiltered[%d] = %f \n", 24,readAltitude_LPfiltered[24]);
//printf("readAltitude_LPfiltered[%d] = %f \n", address - (sample_size - i),readAltitude_LPfiltered[address - (sample_size - i)]);
      }

      //readAltitude_LPfiltered[0] = 2 * readAltitude_LPfiltered[0];
      /*
        for (int i = 0; i < sample_size; i++) {
          printf("address is %d \n",address);
          printf("readAltitude[%d] = %f \n", i,readAltitude[i]);
          printf("readAltitude_LPfiltered[%d] = %f \n", i,readAltitude_LPfiltered[i]);
      }
      */
printf("4readAltitude_LPfiltered[%d] = %f \n", 24,readAltitude_LPfiltered[24]);
      for (int i = 0; i < sample_size; i++) {

        if (i  <= sample_size - 1) {
          printf("3readAltitude_LPfiltered[%d] = %f \n", 24,readAltitude_LPfiltered[24]); 
          propotional[i] = (readAltitude[i + 1]-readAltitude[0]) / (i + 1);
          sum_propotional += propotional[i];
          /*
          propotional[i] = (readAltitude_LPfiltered[i + 1]-readAltitude_LPfiltered[0]) / (i + 1);
          sum_propotional += propotional[i];
           */
        }

      }
      
      buffer_sum_propotional[1] = buffer_sum_propotional[0];
      buffer_sum_propotional[0] = sum_propotional;
      last_address = address;//detecte next round of next 10 altitude.
      
    }
    if (buffer_sum_propotional[0] < 0 && buffer_sum_propotional[1] < 0) { //apogee detection method now

      printf("Apogee detected. \n");
      printf(" Drogue released  \n");     // For tesing. Can be deleted for real flight
      while(1){};
      ascending = false;
      MODE++;                                  // Move to mode 3, flight descdending mode;
    }

  }
  if (MODE == 3 && ascending == false) { //ACTIVE FLIGHT_TEST mode
    printf("It is now Descending.\n");
    relativeAltitude = test_data[address];
    address +=1;
    printf(" relativeAltitude = %f  m \n ",relativeAltitude);
    if (relativeAltitude < main_release_a) {
      printf("Main deployed \n");
      MODE++; // Move to mode 3, flight descdending mode;
    }
  }
  if (MODE == 4 && ascending == false) { //ACTIVE FLIGHT_TEST mode
    printf("It is now Descending \n");
    relativeAltitude = test_data[address];
    address += 1;
    printf(" relativeAltitude = %f m\n ",relativeAltitude);
    if (address == 0) MODE = 5;
    if (relativeAltitude < land_threshold || address == 0 ) {
      MODE = 5;
    }
 
  }
  if (MODE == 5) {// When EEPORM full, freeze it here.
    printf(" Altitude lower than Land Threshold Altitude: ");
    printf(" land_threshold = %f m\n ",land_threshold);
    printf("EEPROM Writing Inactivated \n");
    while (1);
  }    
  }
}
