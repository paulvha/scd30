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

  **********************************************************************
  * modified paulvha : august 2018
  **********************************************************************
  Added support from BME280 (Combo board: https://www.sparkfun.com/products/14348)
  Pressure, relative humidity and temperature in C from BME280
  Perform pressure compensation on the SCD30
  differences and maximum delta of humidity and temperature
  **********************************************************************

  Hardware Connections:
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the device into an available Qwiic port
  Open the serial monitor at 115200 baud to see the output

  ELSE CONNECT TO ARDUINO UNO

  SCD30 :
    VCC to 3.3V
    GND to GND
    SCL to SCL ( Arduino UNO A4)
    SDA to SDA ( Arduino UNO A5)

  Combo board:
    VCC to 3.3V
    GND to GND
    SCL to SCL ( Arduino UNO A4)
    SDA to SDA ( Arduino UNO A5)


  ELSE CONNECT TO ESP8266-THING

  Make sure to cut the link and have a jumper on the DTR/reset. include the jumper
  for programming, remove before starting serial monitor

  SCD30
    GND TO GND
    Connect VCC to 3V3
    SCL to SCL
    SDA to SDA

  Combo board:
    connect VCC to 3.3V
    GND to GND
    SCL to SCL
    SDA to SDA
    (wake, rst and int CONNECTIONS ARE NOT USED)

  Given that SCD30 is using clock stretching the driver has been modified to deal with that.
*/

#include <Wire.h>
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

//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

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
  char buf[10];

  Wire.begin();

  Serial.begin(115200);
  Serial.println("\nSCD30 + BME280 Example");

  // set driver debug level
  // 0 : no messages
  // 1 : request sending and receiving
  // 2 : request sending and receiving + show protocol errors
  airSensor.setDebug(scd_debug);

  if (mySensor.beginI2C() == false) // Begin communication over I2C
  {
    Serial.println("The BME280 did not respond. Please check wiring.");
  }
  else
  {
    detect_BME280 = 1;
  }

  // This will cause readings to occur every two seconds and automatic calibration
  // on an ESP8266 must called last to set the clock stretching correct for SCD30
  airSensor.begin(Wire);
  //This will cause SCD30 readings to occur every two seconds
  if (airSensor.begin() == false)
  {
    Serial.println("The SCD30 did not respond. Please check wiring.");
    while(1);
  }
    
  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 7 digits (6 serial + 0x0)
  airSensor.getSerialNumber(buf);
  Serial.print("serial number: ");
  Serial.println(buf);
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

