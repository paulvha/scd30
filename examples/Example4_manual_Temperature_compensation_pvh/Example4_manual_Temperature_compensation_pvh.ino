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
  You can test the impact of the temperature offset on temperature reading and humidity.
  
  !! It will NOT impact CO2 measurement. !!!
  
  At any moment you can change the temperature offset but just entering the desired change to 
  measured value. (e.g. 6 <enter> for 6 degrees). It might be 5 seconds before you see that it 
  has been applied, but you will get confirmation whether it was applied or failed.

  It will then take minutes before the full impact can be seen as the SCD30 applies the offset 
  in small steps

  The temperature offset is stored on the SCD30 and used after re-powering, 
  to disable set it to zero (0 <enter>)

  **********************************************************************
  * Versioning:
  **********************************************************************
  October 2020 / paulvha
    Added this example
    
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
  Make sure to press the GPIO0 button for connect /upload

  NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE
  In case of ESP32, given the SoftWire library, you have to make a change in SparkfunBME280.h.
  Line 39 states:    #include <Wire.h>
  Comment that line out : //#include <Wire.h>

  NOW include :
   #if defined(ARDUINO_ARCH_ESP32)
   #include <SoftWire/SoftWire.h>
   #else
   #include <Wire.h>
   #endif

  BME280
   VIN  --- 3V3 or 5V
   3v3  --- Not Connected
   GND  --- GND
   SCK  --- SCL
   SDO  --- Not Connected
   SDI  --- SDA
   CS   --- Not Connected
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
  float temps,tempb,hums;
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
  uint16_t tempAdjust = (uint16_t) atoi(input);

  if (! airSensor.setTemperatureOffset(tempAdjust*100)) {
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
