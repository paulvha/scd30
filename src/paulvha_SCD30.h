/*
  This is a library written for the SCD30
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14751

  Written by Nathan Seidle @ SparkFun Electronics, May 22nd, 2018

  The SCD30 measures CO2 with accuracy of +/- 30ppm.

  This library handles the initialization of the SCD30 and outputs
  CO2 levels, relative humidty, and temperature.

  https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  **********************************************************************
  modified by paulvh version 10 August, 2018

  Changes:
  * Added ESP8266 board detection in begin() to support the ESP8266 clockstretching
  * Added settting and display debugging
  * Added obtaining serial number of SCD-30
  * Added StopMeasurement
  * Added StartSingleMeasurement
  * Added CRC checks on different places
  * Added getTemperatureF (Fahrenheit)

  Modified by Paulvha version February 2019

 Changes:
 * Added option in examples 10 and 13 to set BME280 I2C address. (some use 0x76 instead of 0x77)
 * Added SoftWire (a port of the ESP8266 I2C library) for ESP32 (which does NOT support clockstretching)
 * Removed StartSingleMeasurement as that is not working in the SCD30 as it should.
 * Added option to begin() to disable starting measurement. (needed in case one wants to read serial number)
 * updated the keywords.txt file
 * updated sketches and library where needed
 *
  Modified by Paulvha version August 2020

 changes based on Datasheet May 2020
 * added functions : getForceRecalibration, getMeasurementInterval, getTemperatureOffset, getAltitudeCompensation, getFirmwareLevel
 * updated the keywords.txt file
 * added example14 to demonstrate the new functions
 * updated sketches and library where needed
 *
 Change October 2020
  * Update in readmeasurement to translate byte to float. did not work on Arduino. Tested on Uno, Artemis Apollo3, ESP32
  * Added example5 to work with Sparkfun LCD
  * update to example4
  *
 Change November 2020
  * solved a conflict with ByteToFloat when using SPS30 at the same time (rename to ByteToFl)
  *********************************************************************
*/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/** ESP32 hardware I2C does NOT support clock stretching
 * while that is necessary for the SDSP30
 *
 * A port of the ESP8266 I2C has been done to support this
 */
#if defined(ARDUINO_ARCH_ESP32)
#include <SoftWire/SoftWire.h>
#else
#include <Wire.h>
#endif

//The default I2C address for the SCD30 is 0x61.
#define SCD30_ADDRESS 0x61

//Available commands
#define COMMAND_CONTINUOUS_MEASUREMENT  0x0010
#define COMMAND_SET_MEASUREMENT_INTERVAL 0x4600
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_AUTOMATIC_SELF_CALIBRATION 0x5306
#define COMMAND_SET_FORCED_RECALIBRATION_FACTOR 0x5204
#define COMMAND_SET_TEMPERATURE_OFFSET 0x5403
#define COMMAND_SET_ALTITUDE_COMPENSATION 0x5102
#define CMD_READ_SERIALNBR 0xD033
//#define CMD_START_SINGLE_MEAS 0x0006              // removed was not stable in SCD30
#define CMD_STOP_MEAS 0x0104
#define CMD_GET_FW_LEVEL 0xD100                     // added August 2020
#define SCD30_SERIAL_NUM_WORDS 3                    // added August 2020
// The longer serial number is 16 words / 32 bytes (means 48 bytes with CRC).
// Most I2C buffers are by default 32. Hence the length is kept to the
// 3 words = first 6 (equal to what is printed on the case).
// The additional information is for Senserion internal only.

/* needed for conversion float IEE754
 * added October 2020*/
typedef union {
    byte array[4];
    float value;
} ByteToFl;

class SCD30
{
  public:
        SCD30(void);

        boolean begin(TwoWire &wirePort = Wire, bool m_begin = true); //By default use Wire port

        boolean beginMeasuring(uint16_t pressureOffset);
        boolean beginMeasuring(void);
        boolean StopMeasurement(void);

        /* this has been removed as the implementation is NOT stable
         * in the SCD30
         *
         * boolean StartSingleMeasurement(void);
         */

        // paulvha : added get serial number August 2018
        boolean getSerialNumber(char *val);

        // paulvha : added August 2020
        boolean getForceRecalibration(uint16_t *val)  {return(getSettingValue(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, val));}
        boolean getMeasurementInterval(uint16_t *val) {return(getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, val));}
        boolean getTemperatureOffset(uint16_t *val)   {return(getSettingValue(COMMAND_SET_TEMPERATURE_OFFSET, val));}
        boolean getAltitudeCompensation(uint16_t *val){return(getSettingValue(COMMAND_SET_ALTITUDE_COMPENSATION, val));}
        boolean getFirmwareLevel(uint8_t *val);

        uint16_t getCO2(void);
        float getHumidity(void);
        float getTemperature(void);
        float getTemperatureF(void);

        boolean setMeasurementInterval(uint16_t interval);
        boolean setAmbientPressure(uint16_t pressure_mbar);
        boolean setAltitudeCompensation(uint16_t altitude);
        boolean setAutoSelfCalibration(boolean enable);
        boolean setForceRecalibration(uint16_t val);
        boolean setTemperatureOffset(uint16_t tempOffset);
        boolean setTemperatureOffset(float tempOffset);

        boolean dataAvailable();

        // paulvha : added debug messages
        void setDebug(int val);

  private:

        boolean readMeasurement();
        boolean sendCommand(uint16_t command, uint16_t arguments, bool arg);    // added August 2020
        boolean sendCommand(uint16_t command, uint16_t arguments);
        boolean sendCommand(uint16_t command);
        uint8_t ReadFromSCD30(uint16_t command, uint8_t *val, uint8_t cnt);     // added August 2020
        bool getSettingValue(uint16_t command, uint16_t *val);                  // added August 2020

        uint8_t computeCRC8(uint8_t data[], uint8_t len);

        // paulvha : added for debug messages
        void debug_cmd(uint16_t command);

        // Variables
        TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware

        // Global main datums
        float co2 = 0;
        float temperature = 0;
        float humidity = 0;

        // These track the staleness of the current data
        // This allows us to avoid calling readMeasurement() every time individual datums are requested
        boolean co2HasBeenReported = true;
        boolean humidityHasBeenReported = true;
        boolean temperatureHasBeenReported = true;

        void byte_to_float(float *value, uint8_t *p);                       // added October 2020
};
