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
  as well as the BME280 information.

  **********************************************************************
  * Versioning:
  **********************************************************************
  august 2018 / paulvha:
    Support ESP8266-Thing
    include option to debug driver
    Added support from BME280 (Combo board: https://www.sparkfun.com/products/14348)
    Pressure, relative humidity and temperature in C from BME280
    Perform pressure compensation on the SCD30
    differences and maximum delta of humidity and temperature

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

///////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (ONLY NEEDED CASE OF SCD30 ERRORS)   //
//                                                                   //
// 0 : no messages                                                   //
// 1 : request sending and receiving                                 //
// 2 : request sending and receiving + show protocol errors          //
///////////////////////////////////////////////////////////////////////
#define scd_debug 0

///////////////////////////////////////////////////////////////////////
// define the BME280 address.                                        //
// Use if address jumper is closed : 0x76.                           //
///////////////////////////////////////////////////////////////////////
#define I2CADDR 0x77

//////////////////////////////////////////////////////////////////////////
//                SELECT THE WIRE INTERFACE                             //
//////////////////////////////////////////////////////////////////////////
#define SCD30WIRE Wire

///////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED //////////////////
///////////////////////////////////////////////////////////////////////

BME280 mySensor; //Global sensor object
SCD30 airSensor;

// hold statistics
float temp_tot = 0;
float temp_cnt = 0;
float hum_tot = 0;
float hum_cnt = 0;
float temp_max = 0;
float hum_max = 0;

// status
int detect_BME280 = 0;

void setup()
{
  SCD30WIRE.begin();

  Serial.begin(115200);
  Serial.println(F("\nSCD30 + BME280 Example"));

  // set driver debug level
  // 0 : no messages
  // 1 : request sending and receiving
  // 2 : request sending and receiving + show protocol errors
  airSensor.setDebug(scd_debug);

  // set I2C address. default is 0x77
  mySensor.setI2CAddress(I2CADDR);

  if (! mySensor.beginI2C() ) // Begin communication over I2C
  {
    Serial.println(F("The BME280 did not respond. Please check wiring."));
  }
  else
  {
    Serial.println(F("The BME280 detected"));
    detect_BME280 = 1;
  }

  // This will init the hardware start automatic reading
  // on an ESP8266 must called last to set the clock stretching correct for SCD30
  if (! airSensor.begin(SCD30WIRE) )
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }

  // get Device information
  DeviceInfo();
}

void loop()
{
  float temps,hums, tempb, humb, pressure;

  if (airSensor.dataAvailable())
  {

    //serialTrigger();                // option to wait for user enter instead of constant loop

    Serial.print(F("SCD30 : co2(ppm) : "));
    Serial.print(airSensor.getCO2());

    Serial.print(F("\ttemp(C): "));
    temps = airSensor.getTemperature();
    Serial.print(temps, 2);

    Serial.print(F("\thumidity(%): "));
    hums = airSensor.getHumidity();
    Serial.print(hums, 1);

    Serial.println();

    // if BME280 was detected include the information & statistics
    if (detect_BME280 == 1)
    {
      Serial.print(F("BME280: Pressure: "));
      pressure = mySensor.readFloatPressure()/100;
      Serial.print(pressure, 0);

      Serial.print(F("\ttemp(C): "));
      tempb = mySensor.readTempC();
      //tempb = mySensor.readTempF();
      Serial.print(tempb, 2);

      Serial.print(F("\thumidity(%): "));
      humb =mySensor.readFloatHumidity();
      Serial.print(humb, 1);

      Serial.print(F("\tAlt(m): "));
      Serial.print(mySensor.readFloatAltitudeMeters(), 1);
      //Serial.print(mySensor.readFloatAltitudeFeet(), 1);

      Serial.println();

      // count TOTAL delta
      temp_tot += temps - tempb;
      hum_tot += hums - humb;

      // count samples
      temp_cnt++;
      hum_cnt++;

      // obtain maximum delta
      if (temps > tempb)
      {
        if (temp_max < temps - tempb) temp_max = temps - tempb;
      }
      else
      {
        if (temp_max < tempb - temps) temp_max - tempb- temps;
      }

      if (hums > humb)
      {
        if (hum_max < hums-humb) hum_max = hums - humb;
      }
      else
      {
        if (hum_max < humb-hums) hum_max = humb - hums;
      }

      Serial.print(F("Delta avg temp : "));
      Serial.print(temp_tot/temp_cnt, 2);
      Serial.print(F(" Delta avg humidity : "));
      Serial.print(hum_tot/hum_cnt,2);
      Serial.print(F(" Biggest delta temp : "));
      Serial.print(temp_max);
      Serial.print(F(" Biggest delta humidity : "));
      Serial.print(hum_max);
      Serial.println("\n");

      // Pressure adjustment, current ambient pressure in mBar: 700 to 1200 read from BME80
      airSensor.setAmbientPressure(pressure);
    }
  }
  else
    Serial.println(F("No SCD30 data"));

  // only every 2 seconds is data available
  delay(2000);
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
