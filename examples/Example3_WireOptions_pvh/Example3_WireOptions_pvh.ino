/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751

  This example demonstrates how to start the library using other Wire ports.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.

  NOTE: THIS EXAMPLE WILL FAIL ON A DEVICE THAT ONLY HAS ONE WIRE INTERFACE

  Note: 100kHz I2C is fine, but according to the datasheet 400kHz I2C is not supported by the SCD30
*/

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (only NEEDED case of errors)            //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
//                SELECT THE WIRE INTERFACE                             //
//////////////////////////////////////////////////////////////////////////
#define SCD30WIRE Wire

#include "paulvha_SCD30.h"

SCD30 airSensor;

void setup()
{
  Serial.begin(115200);
  Serial.println("SCD30 Example 3");

  // set driver debug level
  airSensor.setDebug(scd_debug);

  // Start the wire hardware that may be supported by your platform
  SCD30WIRE.begin();

  // This will cause readings to occur every two seconds
  if (! airSensor.begin(SCD30WIRE))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }

  // display device information
  DeviceInfo();

  //The library will now use SCD30WIRE for all communication
}

void loop()
{
  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");
    Serial.print(airSensor.getCO2());

    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
  }
  else
    Serial.print(".");

  delay(1000);
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

