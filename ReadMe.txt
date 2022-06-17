1. The normal sequence for running the PSI program is
EEPROMERASE(Open the serial monitor and type "yes" to ACTUALLY erase all data)
->
FlightPrograme v4.2(under file "Flight Code")
->
 EEPROMREAD(Copy and Paste data in the serial monitor to an excel file. )

2. To do the ground launch test, the sequence is :

LaunchTest/WriteTestData/G1C/WriteTestData(0-1199)(Upload every file sepeartely and you should hear a clear beep after 
each writing is done)
->
LaunchTest/LaunchSiteTestV4.3

3. To run the flight code without checking if the EEPROMs and Power are connected, just upload 
FlightCode/FlightPorgram_PC_powered.
