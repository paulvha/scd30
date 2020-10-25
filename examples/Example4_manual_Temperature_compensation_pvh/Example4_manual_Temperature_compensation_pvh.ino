/*
  Based on :
  
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751
  
  *************************************************************************************************
  
  This example prints the current CO2 level, relative humidity, and temperature in C from the SCD30
  You can test / set the impact of the temperature offset on temperature reading and humidity.
  
  !! It will NOT impact CO2 measurement. Only the Temperature and Humidity output results  !!!

  The higher the temperature offset, the lower the temperature output and increase in humidity.
 
  At any moment you can change the temperature offset but just entering the desired change in the 
  serial monitor to measured value. (e.g. 6 <enter> for 6 degrees). It might be 5 seconds before you 
  see that it has been applied, but you will get confirmation whether it was applied or failed.

  It will then take about 10 minutes before the full impact can be seen as the SCD30 applies 
  the offset in small steps.

  The temperature offset is stored on the SCD30 and used after re-powering, 
  to disable set it to zero (0 <enter>)

  *********************************************************************
  * use for temperature Calibration (will NOT impact CO2 measurement)
  *********************************************************************

  You can use this sketch to calibrate and store the temperature offset
  Try to create a stable environment with 1 or more external temperature measurement devices.
  
  1. Start the sketch and enter 0 to disable any current temperature offset 
  2. Let the sketch run for some time and watch that the SCD30 temperature reading is
     more or less stable for couple of minutes. This can take up to 30 minutes.
  3. Watch the difference between the SCD30 and external temperature measurement device.
  4. Now enter the number that is different (e.g. SCD30 is 6.2 degrees higher, then enter 6.2)
     You can NOT enter a negative number !
  5. It will take up to 10 minutes before you see the full effect. (so > 600 seconds AFTER you entered)
  6. If still to much difference after 20 minutes repeat from step 3. But remember your kost recent 
     change entered, say it was 6.2. If the SCD30 output is still 1 degree to high, you must enter 7.2.
     If the SCD30 reading is now 0.5 degrees to low, you must enter 5.7 !!!

  The temperature offset is stored on the SCD30 and used after re-powering, 

  In my case :
  After entering 0 (step 1) it took 730 seconds before more or less stable (step2). The SCD30 was
  showing 24.40, where a good quality other temperature meter was indicating 22.9C. (step 3) So
  I have entered 2.52 for correction (step 4). 
  
  After 1720 the SCD30 was indicating 23,46, and the temperature meter 23.1.(Step 5)  
  So I had to offset  23.36 - 23.1 = 0.26 more than 2.52 => 2.52 + 0.36 = 2.78 (step6)
  
  After 2715 the SCD30 was showing 22.9 where the temperature meter was showing 23.0. Now I enter  2.78 - 0.1 = 2.68

  Repeated the above a couple of times everytime a smaller down change, ending on 2.6 as an offset. 
  BUT then I let it run for hours.. and had to make a final adjustment to 3.1
  ********************************************************************** 
  * Versioning:
  **********************************************************************
  October 2020 / paulvha
    * Added this example
    
  October 2020 / paulvha
    * you can now enter in float (e.g. 6.2 instead of only 6)
    
  **********************************************************************
  pin layout SCD30
  VDD       1 Supply Voltage ( !!! ON THE CORNER OF THE BOARD !!)
  GND       2 Ground
  TX/SCL    3 Transmission line Modbus / Serial clock I2C
  RX/SDA    4 Receive line Modbus / Serial data I2C
  RDY       5 Data ready. High when data is ready for read-out  (*1)
  PWM       6 PWM output of CO2 concentration measurement  (*1)
              (May2020 : supported  BUT not implemented)
  SEL       7 Interface select pin. Pull to VDD for selecting Modbus,
              leave floating or connect to GND for selecting I2C. (*1)

  Note *1 : none of these lines are connected or used

  CONNECT TO ARDUINO

  SCD30 :
    VCC to 3V3 or 5V
    GND to GND
    SCL to SCL ( Arduino UNO A5)
    SDA to SDA ( Arduino UNO A4)

  CONNECT TO ESP8266-THING

  Make sure to cut the link and have a jumper on the DTR/reset.
  Include the jumper for programming, remove before starting serial monitor

  SCD30    ESP8266
    GND --- GND
    VCC --- 3V3
    SCL --- SCL
    SDA --- SDA

  Given that SCD30 is using clock stretching the driver has been modified to deal with that.

  CONNECT TO ESP32-THING

  SCD30    ESP32
    GND --- GND
    VCC --- 3V3
    SCL --- 22
    SDA --- 21

  Given that SCD30 is using clock stretching SoftWire is selected by the driver to deal with that.
  Make sure to press the GPIO0 button for connect / upload
*/

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (ONLY NEEDED CASE OF SCD30 ERRORS)      //
//                                                                      //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
//                  NO CHANGES NEEDED BEYOND THIS POINT                 //
////////////////////////////////////////////////////////////////////////// 

#include "paulvha_SCD30.h"
SCD30 airSensor;

void setup()
{
  Wire.begin();

  Serial.begin(9600);
  Serial.println(F("\nSCD30 + Manual temperature offset"));

  // set driver debug level
  airSensor.setDebug(scd_debug);

  // This will init, but not start measurement
  // on an ESP8266 must called last to set the clock stretching correct for SCD30
  if (! airSensor.begin(Wire,false))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }

  DeviceInfo();

  // This will cause SCD30 readings to occur every two seconds
  if (!airSensor.beginMeasuring())
  {
    Serial.println(F("The SCD30 did not start. Please check wiring."));
    while(1);
  }

  airSensor.setAutoSelfCalibration(0);          // stop ASC as that is set automatically during airSensor.begin()
}

void loop()
{
  float temps,hums;
  static uint16_t  secc = 0;   // count seconds

  // handle any keyboard input
  if (Serial.available()) {
    while (Serial.available()) handle_input(Serial.read());
  }
    
  if (airSensor.dataAvailable())
  {
    Serial.print(F("seconds\t"));
    Serial.print(secc);

    Serial.print(F("\tco2(ppm)\t"));
    Serial.print(airSensor.getCO2());

    Serial.print(F("\ttemp(C)\t"));
    temps = airSensor.getTemperature();
    Serial.print(temps, 2);
   
    Serial.print(F("\thumidity(%)\t"));
    hums = airSensor.getHumidity();
    Serial.println(hums, 1);
  }
  else
    Serial.println(F("No SCD30 data"));

  // wait every 5 seconds is data available
  delay(5000);

  secc += 5;
}

void handle_input(char c)
{
  static char input[10];    // keyboard input buffer
  static int inpcnt = 0;    // keyboard input buffer length
  
  if (c == '\r') return;    // skip CR
        
  if (c != '\n') {          // act on linefeed
    input[inpcnt++] = c;
    if (inpcnt < 10) return;// prevent overrun
  }
  
  input[inpcnt] = 0x0;

  float tempAdjust = asciitof(input);
  
  if (! airSensor.setTemperatureOffset(tempAdjust)) {
    Serial.println(F("Could not set temperature offset"));
  }
  else {
    Serial.print(F("Temperature offset : "));
    Serial.println(tempAdjust);
  }
  
  inpcnt = 0;               // reset pointer
}

void DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) +1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)
  if (airSensor.getSerialNumber(buf))
  {
   Serial.print(F("SCD30 serial number : "));
   Serial.println(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val) ) {
    Serial.print("SCD30 Firmware level: Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
  }
  else {
    Serial.println("Could not obtain firmware level");
  }
}

// October 2020
// Added code below as atof() on Artemis / Apollo3 is causing compile errors
// lib version 2.0.2 :  undefined reference to `__wrap__calloc_r'
// Expect that this will be fixed in the future, but for now include an alternative
// taken from https://github.com/GaloisInc/minlibc/blob/master/atof.c
#define isdigit(c) (c >= '0' && c <= '9')

double asciitof(const char *s)
{
  // This function stolen from either Rolf Neugebauer or Andrew Tolmach. 
  // Probably Rolf.
  double a = 0.0;
  int e = 0;
  int c;
  while ((c = *s++) != '\0' && isdigit(c)) {
    a = a*10.0 + (c - '0');
  }
  if (c == '.') {
    while ((c = *s++) != '\0' && isdigit(c)) {
      a = a*10.0 + (c - '0');
      e = e-1;
    }
  }
  if (c == 'e' || c == 'E') {
    int sign = 1;
    int i = 0;
    c = *s++;
    if (c == '+')
      c = *s++;
    else if (c == '-') {
      c = *s++;
      sign = -1;
    }
    while (isdigit(c)) {
      i = i*10 + (c - '0');
      c = *s++;
    }
    e += i*sign;
  }
  while (e > 0) {
    a *= 10.0;
    e--;
  }
  while (e < 0) {
    a *= 0.1;
    e++;
  }
  return a;
}
