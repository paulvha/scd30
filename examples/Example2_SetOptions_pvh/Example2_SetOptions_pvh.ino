/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751

  This example demonstrates the various settings available on the SCD30.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.

  **********************************************************************
  * Versioning:
  **********************************************************************
  august 2018 / paulvha:
    Support ESP8266-Thing
    include option to debug driver
    obtain SCD30 serial number
    obtain temperature in Fahrenheit

  January 2019 / Paulvha
    Added SoftWire support for ESP32
  **********************************************************************
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

  Note: All settings (interval, altitude, etc) are saved to non-volatile memory and are
  loaded by the SCD30 at power on. There's no damage in sending that at each power on.

  Note: 100kHz I2C is fine, but according to the datasheet 400kHz I2C is not supported by the SCD30
*/

//////////////////////////////////////////////////////////////////////////
//////////// Change to the pressure in mbar on your location /////////////
/////// for better SCD30 - CO2 results (between 700 and 1200 mbar)  //////
//////////////////////////////////////////////////////////////////////////
#define pressure 1018

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (only NEEDED case of errors)            //
// Requires serial monitor (remove DTR-jumper before starting monitor)  //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
// Change number of seconds between measurements: 2 to 1800 (30 minutes)//
// setting to 4 will cause 3 dots before reading, due to call delay(1000)/
//////////////////////////////////////////////////////////////////////////
#define read_interval 4

//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

#include "paulvha_SCD30.h"

SCD30 airSensor;

void setup()
{
  char buf[10];
  Wire.begin();

  Serial.begin(9600);
  Serial.println("SCD30 Example 2");

  // set driver debug level
  airSensor.setDebug(scd_debug);

  //This will init setting but NOT readings
  airSensor.begin(Wire,false);

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 7 digits (6 serial + 0x0)
  airSensor.getSerialNumber(buf);
  Serial.print("serial number: ");
  Serial.println(buf);

  //This will cause readings to occur every two seconds
  airSensor.begin();

  // Change number of seconds between measurements: 2 to 1800 (30 minutes)
  // setting to 4 will cause 3 dots before reading, due to call delay(1000)
  airSensor.setMeasurementInterval(read_interval);

/* paulvha : you can set EITHER the Altitude compensation of the pressure.
 * Setting both does not make sense as both overrule each other, but it is included for demonstration
 *
 * see Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
 *
 *    The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in mBar to the sensor.
 *    Setting the ambient pressure will overwrite previous and future settings of altitude compensation. Setting the argument to zero
 *    will deactivate the ambient pressure compensation. For setting a new ambient pressure when continuous measurement is running
 *    the whole command has to be written to SCD30.
 *
 *    Setting altitude is disregarded when an ambient pressure is given to the sensor
 */

  // My desk is ~10m above sealevel
  airSensor.setAltitudeCompensation(10); //Set altitude of the sensor in meter

  // Pressure in Boulder
  airSensor.setAmbientPressure(pressure); //Current ambient pressure in mBar: 700 to 1200
}

void loop()
{
  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");
    Serial.print(airSensor.getCO2());

    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" temp(F):");
    Serial.print(airSensor.getTemperatureF(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
  }
  else
    Serial.print(".");

  delay(1000);
}
