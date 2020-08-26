/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751

  This example prints the current CO2 level, relative humidity, and temperature in C from the SCD30
  as well as BME280 temperature information

  **********************************************************************
  * Versioning:
  **********************************************************************
  august 2018 / paulvha:
    Support ESP8266-Thing
    include option to debug driver
    Added support from BME280 (Combo board: https://www.sparkfun.com/products/14348)
    Pressure, relative humidity and temperature in C from BME280

  January 2019 / Paulvha
    Added option to set BME280 I2C addr. (some use 0x76 instead of 0x77)
    Added SoftWire support for ESP32
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
    SCL to SCL ( Arduino UNO A4)
    SDA to SDA ( Arduino UNO A5)

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

#include "paulvha_SCD30.h"
#include "SparkFunBME280.h"

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (ONLY NEEDED CASE OF SCD30 ERRORS)      //
//                                                                      //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0


///////////////////////////////////////////////////////////////////////
// define the BME280 address.                                        //
// Use if address jumper is closed : 0x76.                           //
///////////////////////////////////////////////////////////////////////
#define I2CADDR 0x77

BME280 mySensor; //Global sensor object
SCD30 airSensor;

// status
int detect_BME280 = 0;

// count seconds
uint16_t  secc = 0;

void setup()
{
  char buf[10];

  Wire.begin();

  Serial.begin(9600);
  Serial.println(F("\nSCD30 + BME280 Example"));

  // set driver debug level
  airSensor.setDebug(scd_debug);

  // set I2C address. default is 0x77
  mySensor.setI2CAddress(I2CADDR);

  if (mySensor.beginI2C() == false) // Begin communication over I2C
  {
    Serial.println(F("The BME280 did not respond. Please check wiring."));
  }
  else
  {
    Serial.println(F("The BME280 detected"));
    detect_BME280 = 1;
  }

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

  //////////////////////////////////////////////////
  // change this for testing                      //
  //////////////////////////////////////////////////
  airSensor.setAutoSelfCalibration(0);          // stop ASC as that is set automatically during airSensor.begin()
  airSensor.setTemperatureOffset((float) 0);    // set for x degrees Temperature offset
}

void loop()
{
  float temps,tempb,hums;

  if (airSensor.dataAvailable())
  {
    // serialTrigger();                // option to wait for user enter instead of constant loop

    Serial.print(F("seconds\t"));
    Serial.print(secc);

    Serial.print(F("\tco2(ppm)\t"));
    Serial.print(airSensor.getCO2());

    Serial.print(F("\ttemp(C)\t"));
    temps = airSensor.getTemperature();
    Serial.print(temps, 2);

    // only if BME280 was detected
    if (detect_BME280 == 1)
    {
      Serial.print(F("\tBCM:\t"));
      tempb = mySensor.readTempC();
      //tempb = mySensor.readTempF();
      Serial.print(tempb, 2);
    }

    Serial.print(F("\thumidity(%)\t"));
    hums = airSensor.getHumidity();
    Serial.print(hums, 1);

    Serial.println();

  }
  else
    Serial.println(F("No SCD30 data"));

  // wait every 5 seconds is data available
  delay(5000);

  secc += 5;
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

/* serialTrigger prints a message, then waits for something
 * to come in from the serial port.
 */
void serialTrigger()
{
  Serial.println();
  Serial.println(F("press enter"));
  Serial.println();

  while (!Serial.available());

  while (Serial.available())
    Serial.read();
}
