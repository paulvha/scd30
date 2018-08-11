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
  *
  *********************************************************************
*/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

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
#define CMD_START_SINGLE_MEAS 0x0006
#define CMD_STOP_MEAS 0x0104

class SCD30
{
  public:
        SCD30(void);

        boolean begin(TwoWire &wirePort = Wire); //By default use Wire port

        boolean beginMeasuring(uint16_t pressureOffset);
        boolean beginMeasuring(void);
        boolean StopMeasurement(void);

        // paulvh : added single measurement
        boolean StartSingleMeasurement(void);

        // paulvha : added get serial number
        boolean getSerialNumber(char *val);

        uint16_t getCO2(void);
        float getHumidity(void);
        float getTemperature(void);
        float getTemperatureF(void);

        boolean setMeasurementInterval(uint16_t interval);
        boolean setAmbientPressure(uint16_t pressure_mbar);
        boolean setAltitudeCompensation(uint16_t altitude);
        boolean setAutoSelfCalibration(boolean enable);
        boolean setForceRecalibration(uint16_t val);
        boolean setTemperatureOffset(float tempOffset);

        boolean dataAvailable();
        boolean readMeasurement();

        boolean sendCommand(uint16_t command, uint16_t arguments);
        boolean sendCommand(uint16_t command);

        uint16_t readRegister(uint16_t registerAddress);

        uint8_t computeCRC8(uint8_t data[], uint8_t len);

        // paulvha : added debug messages
        void setDebug(int val);

  private:
        // paulvha : added for debug messages
        void debug_cmd(uint16_t command);

        //Variables
        TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware

        //Global main datums
        float co2 = 0;
        float temperature = 0;
        float humidity = 0;

        //These track the staleness of the current data
        //This allows us to avoid calling readMeasurement() every time individual datums are requested
        boolean co2HasBeenReported = true;
        boolean humidityHasBeenReported = true;
        boolean temperatureHasBeenReported = true;

};
