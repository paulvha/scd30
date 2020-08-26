/*

  This example demonstrates the new functions that becam available
  known with the datasheet of the SCD30 May 2020

  Hardware Connections:
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the device into an available Qwiic port
  Open the serial monitor at 115200 baud to see the output

  **********************************************************************
  * Versioning:
  **********************************************************************
  August 2020 / paulvha
    new functions from the datasheet May 2020
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

  Note *1 : none of these lines are connected or used.

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
*/

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (only NEEDED case of errors)            //
// Requires serial monitor (remove DTR-jumper before starting monitor)  //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
//                SELECT THE WIRE INTERFACE                             //
//////////////////////////////////////////////////////////////////////////
#define SCD30WIRE Wire

//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

#include "paulvha_SCD30.h"

SCD30 airSensor;

void setup()
{
  SCD30WIRE.begin();

  Serial.begin(115200);
  Serial.println("SCD30 Example 14: new functions 2020");

  airSensor.setDebug(scd_debug);

  airSensor.begin(SCD30WIRE); // This will cause readings to occur every two seconds
}

void loop()
{
  DeviceInfo();

  Do_temperature_offset();

  Do_Measurement_interval();

  Do_FRC();

  Do_Altitude_Comp();

  delay(5000);
}


void DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) +1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 32 digits (32 serial + 0x0)
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

void Do_temperature_offset()
{
  uint16_t val, val1;

  if ( airSensor.getTemperatureOffset(&val) ) {
    Serial.print("\nReading Temperature offset before making change: ");
    Serial.println(val);

    val1 = val + 1;

    Serial.print("Setting new Temperature offset to : ");
    Serial.println(val1);

    if ( airSensor.setTemperatureOffset(val1) ) {

        if ( airSensor.getTemperatureOffset(&val1) ) {

          Serial.print("Reading Temperature offset after making change : ");
          Serial.println(val1);

          if ( airSensor.setTemperatureOffset(val) ) {
            Serial.print("Resetting Temperature offset to : ");
            Serial.println(val);
          }
          else {
            Serial.print("Could not reset Temperature offset to ");
            Serial.println(val);
          }
        }
        else {
          Serial.println("Could not obtain Temperature offset");
        }
    }
    else {
      Serial.println("Could not set new Temperature offset");
    }
  }
  else {
    Serial.println("Could not obtain Temperature offset");
  }
}

void Do_Measurement_interval()
{
  uint16_t val, val1;

  if ( airSensor.getMeasurementInterval(&val) ) {
    Serial.print("\nReading measurement interval before making change: ");
    Serial.println(val);

    val1 = val + 1;

    Serial.print("Setting new measurement interval to : ");
    Serial.println(val1);

    if ( airSensor.setMeasurementInterval(val1) ) {

        if ( airSensor.getMeasurementInterval(&val1) ) {

          Serial.print("Reading measurement interval after making change : ");
          Serial.println(val1);

          if ( airSensor.setMeasurementInterval(val) ) {
            Serial.print("Reset measurement interval to : ");
            Serial.println(val);
          }
          else {
            Serial.print("Could not reset measurement interval to ");
            Serial.println(val);
          }
        }
        else {
          Serial.println("Could not obtain measurement interval");
        }
    }
    else {
      Serial.println("Could not set new measurement interval");
    }
  }
  else {
    Serial.println("Could not obtain measurement interval");
  }
}

void Do_FRC()
{
  uint16_t val, val1;

  if ( airSensor.getForceRecalibration(&val) ) {
    Serial.print("\nReading forced calibration factor before making change: ");
    Serial.println(val);

    val1 = val + 1;

    Serial.print("Setting new forced calibration factor to : ");
    Serial.println(val1);

    if ( airSensor.setForceRecalibration(val1) ) {

        if ( airSensor.getForceRecalibration(&val1) ) {

          Serial.print("Reading forced calibration factor after making change : ");
          Serial.println(val1);

          if ( airSensor.setForceRecalibration(val) ) {
            Serial.print("Resetting forced calibration factor to : ");
            Serial.println(val);
          }
          else {
            Serial.print("Could not reset forced calibration factor to ");
            Serial.println(val);
          }
        }
        else {
          Serial.println("Could not obtain forced calibration factor");
        }
    }
    else {
      Serial.println("Could not set forced calibration factor");
    }
  }
  else {
    Serial.println("Could not obtain forced calibration factor");
  }
}

void Do_Altitude_Comp()
{
  uint16_t val, val1;

  if ( airSensor.getAltitudeCompensation(&val) ) {
    Serial.print("\nReading Altitude compensation before making change: ");
    Serial.println(val);

    val1 = val + 1;

    Serial.print("Setting new Altitude compensation to : ");
    Serial.println(val1);

    if ( airSensor.setAltitudeCompensation(val1) ) {

        if ( airSensor.getAltitudeCompensation(&val1) ) {

          Serial.print("Reading Altitude compensation after making change : ");
          Serial.println(val1);

          if ( airSensor.setAltitudeCompensation(val) ) {
            Serial.print("Resetting Altitude compensation to : ");
            Serial.println(val);
          }
          else {
            Serial.print("Could not reset Altitude compensation to ");
            Serial.println(val);
          }
        }
        else {
          Serial.println("Could not obtain Altitude compensation");
        }
    }
    else {
      Serial.println("Could not set new Altitude compensation");
    }
  }
  else {
    Serial.println("Could not obtain Altitude compensation");
  }
}
