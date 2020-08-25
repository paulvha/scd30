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

  *********************************************************************
  Modified by paulvha version 10 August, 2018

  Changes:
  * Added ESP8266 board detection in begin() to support the ESP8266 clockstretching
  * Added settting and display debugging
  * Added obtaining serial number of SCD-30
  * Added StopMeasurement
  * Added StartSingleMeasurement (!! removed later as it is unstable)
  * Added CRC checks on different places
  * Added getTemperatureF (Fahrenheit)
  *
  * Added check on Temperature offset

  Modified by Paulvha version February 2019

  Changes:
  * Added option in examples 10 and 13 to set BME280 I2C address. (some use 0x76 instead of 0x77)
  * Added SoftWire (a port of the ESP8266 I2C library) for ESP32 (which does NOT support clockstretching)
  * Removed StartSingleMeasurement as that is not working in the SCD30 as it should.
  * Added option to begin() to disable starting measurement. (needed in case one wants to read serial number)
  * updated the keywords.txt file
  * updated sketches and library where needed
  *
  Changes based on Datasheet May 2020
  * added functions : getForceRecalibration, getMeasurementInterval, getTemperatureOffset, getAltitudeCompensation, getFirmwareLevel
  * updated the keywords.txt file
  * added example14 to demonstrate the new functions
  * updated sketches and library where needed
  *********************************************************************
*/

#include "paulvha_SCD30.h"
#include "printf.h"             // need to include for Arduino

/* paulvha : August 2018
 *
 * 0 : no debug message
 * 1 : sending and receiving data
 * 2 : 1 + I2c protocol progress
 */
int SCD_DEBUG = 0;

SCD30::SCD30(void)
{
  // Constructor
}

/**
 * @brief Initialize the Serial port
 * @param wirePort : I2C channel to use
 * @param m_begin  : if true will start measurement every 2 seconds
 */
boolean SCD30::begin(TwoWire &wirePort, bool m_begin)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  //We expect caller to begin their I2C port, with the speed of their choice external to the library
  //But if they forget, we start the hardware here.
  _i2cPort->begin();

  /* paulvha : August 2018
   *
   * Especially during obtaining the ACK BIT after a byte sent the SCD30 is using clock stretching  (but NOT only there)!
   * The need for clock stretching is described in the Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
   *
   * The default clock stretch (maximum wait time) on the ESP8266-thing is 230us which is set during _i2cPort->begin();
   * In the current implementation of the ESP8266 I2C driver there is NO error message when this time expired, while
   * the clock stretch is still happening, causing uncontrolled behaviour of the hardware combination.
   *
   * Based on debugging and extensive testing/scoping I have seen that the SCD30 sometimes needs as much as 800us.
   *
   * The hardware I2C in an Arduino does NOT have the possibility to set ClockStretchlimit and as such a check for ESP8266 boards
   * has been added in the driver as part of SCD30 begin() call as I did not want to change the default Wire.h and standard I2C
   * for an ESP8266.
   *
   * This has been created with ESP8266 2.4.2 driver and I expect that new boards will come available that need to be added later.
   * Open de boards.txt of the driver, look for build.board at to the list
   *
   * With setting to 20000, we set a max timeout of 20mS (7x the maximum measured) basically disabling the time-out and now wait
   * for clock stretch to be controlled by the client.  (which is nearly the same as the hardware I2C works which does NOT seem to
   * have timeout)
   *
   * The ESP32 seems to use hardware I2C and thus does not support ClockStretch. A special port of the I2C from the ESP8266 has been
   * done and included as SoftSerial for ESP32
   */

//#if defined(ARDUINO_ESP8266_THING) || defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_ESP01) || defined(ARDUINO_ESP8266_ESP13) || defined(ARDUINO_ESP8266_ESP12) || defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_THING_DEV) || defined (ARDUINO_ESP8266_ESP210) || defined(ARDUINO_MOD_WIFI_ESP8266) || defined(ARDUINO_ESP8266_PHOENIX_V1) || defined(ARDUINO_ESP8266_PHOENIX_V2) || defined(ARDUINO_ESP8266_ARDUINO)
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    if (SCD_DEBUG > 1) printf("setting clock stretching to 200000 (~200ms)\n");
    _i2cPort->setClockStretchLimit(200000);

#else
    if (SCD_DEBUG > 1) printf("NO ESP8266/ ESP32 expected\n");
#endif

  // if measurement is started immediately the serial number can often not be read
  if(! m_begin ) return(true);

  // Check for device to respond correctly
  if(beginMeasuring())            //Start continuous measurements
  {
    setMeasurementInterval(2);    //2 seconds between measurements
    setAutoSelfCalibration(true); //Enable auto-self-calibration

    return (true);
  }

  if (SCD_DEBUG > 1) printf("Something went wrong to start\n");
  return (false); //Something went wrong
}

//Returns the latest available CO2 level
//If the current level has already been reported, trigger a new read
uint16_t SCD30::getCO2(void)
{
  if (co2HasBeenReported) //Trigger a new read
    readMeasurement(); //Pull in new co2, humidity, and temp into global vars

  co2HasBeenReported = true;

  return (uint16_t)co2; //Cut off decimal as co2 is 0 to 10,000
}

//Returns the latest available humidity
//If the current level has already been reported, trigger a new read
float SCD30::getHumidity(void)
{
  if (humidityHasBeenReported) //Trigger a new read
    readMeasurement(); //Pull in new co2, humidity, and temp into global vars

  humidityHasBeenReported = true;

  return humidity;
}

//Returns the latest available temperature in Celsius
//If the current level has already been reported, trigger a new read
float SCD30::getTemperature(void)
{
  if (temperatureHasBeenReported) //Trigger a new read
    readMeasurement(); //Pull in new co2, humidity, and temp into global vars

  temperatureHasBeenReported = true;

  return temperature;
}

/* paulvha : August 2018
 * get temperature in Fahrenheit */

float SCD30::getTemperatureF(void)
{
    float output = getTemperature();
    output = (output * 9) / 5 + 32;

    return output;
}

/*
 * Read from SCD30 the amount of requested bytes
 * param val : to store the data received
 * param cnt : number of data bytes requested
 *
 * return
 * OK number of bytes read
 * 0  error
 */
uint8_t SCD30::ReadFromSCD30(uint16_t command, uint8_t *val, uint8_t cnt)
{
   if (cnt > 20 ) return(0); // max 20 data bytes

   // sent request
   if (! sendCommand(command) ) return(0);

   // Start receiving cnt = data bytes + CRC
  _i2cPort->requestFrom((uint8_t)SCD30_ADDRESS, (uint8_t) (cnt * 3 / 2));

  y = 0;

  if (SCD_DEBUG > 0)  printf("\nReceiving: " );

  // read from buffer
  if (_i2cPort->available())
  {
    for (byte x = 0 ; x < (cnt * 3 / 2) ; x++)
    {
      byte incoming = _i2cPort->read();

      if (SCD_DEBUG > 0) printf("0x%02X ", incoming);

      switch (x)
      {
        case 0:             // data bytes
        case 1:
        case 3:
        case 4:
        case 6:
        case 7:
        case 9:
        case 10:
        case 12:
        case 13:
        case 15:
        case 16:
        case 18:
        case 19:
          *val++ = incoming;
          data[y++] = incoming;
          break;

        default:             // handle CRC
          crc = computeCRC8(data,(uint8_t) 2);
          if (incoming != crc)
          {
            if (SCD_DEBUG > 1) printf("\ncrc error : expected 0x%02X, got 0x%02X\n", crc, incoming);
            return(0);
          }
          y = 0;
          break;
       }
     }

     if (SCD_DEBUG > 0) printf("\n");
  }
  else
  {
      if (SCD_DEBUG > 1)  printf("\nSensor did not sent anything\n");
      return(0);
  }

  return(cnt);
}

/* get Firmware Level see 1.4.9
 * Added August 2020
 *
 * val[0] MAJOR
 * val[1] MINOR
 *
 */
boolean SCD30::getFirmwareLevel(uint8_t *val)
{
  val[0] = val[1] = 0x0;

  if (SCD_DEBUG > 0)  printf("get Firmware level\n");

  // request from SCD30
  if (ReadFromSCD30(CMD_GET_FW_LEVEL, val, 2) != 2) return(false);

  return(true);
}

/* paulvha : August 2018:  read serial number from SCD30
 *
 * provided val buffer must be defined at least 7 digits
 *  6 serial + 0x0 termination
 *
 * return
 *  true if OK (serial number in val-buffer)
 *  false in case of error
 */
boolean SCD30::getSerialNumber(char *val)
{
  // request from SCD30
  if (ReadFromSCD30(CMD_READ_SERIALNBR, (uint8_t *) val, 6) != 6) return(false);

  val[7] = 0x0; // terminate

  return(true);
}


/* Enables or disables the ASC See 1.3.6
 *
 * ASC status is saved in non-volatile memory. When the sensor is powered down while ASC is activated SCD30
 * will continue with automatic self-calibration after repowering without sending the command.
 *
 * At this moment it is not able to detect whether the self calibration has been done or finished
 */
boolean SCD30::setAutoSelfCalibration(boolean enable)
{
  if (enable)
    return(sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 1)); //Activate continuous ASC
  else
    return(sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 0)); //Deactivate continuous ASC
}

/* get Temperature Offset see 1.4.7
 * Added August 2020
 *
 */
boolean SCD30::getTemperatureOffset(uint16_t *val)
{
  uint8_t tmp[2];
  *val =0;

  if (SCD_DEBUG > 0)  printf("Reading Temperature offset\n");

  // request from SCD30
  if (ReadFromSCD30(COMMAND_SET_TEMPERATURE_OFFSET, tmp, 2) != 2) return(false);

  *val = tmp[0] << 8 | tmp[1];

  return(true);
}

/* Set the temperature offset. See 1.4.7. updated August 2020
 *
 * Temperature offset value is saved in non-volatile memory.
 * The last set value will be used for temperature offset compensation after repowering.
 *
 * The ref temperature is based on the temperature underwhich the sensor was calibrated.
 * So if the current temperature is different call this routine to improve the quality of reading.
 *
 * Assume the calibration was done at 27C, current it reads 29C the difference is 2C. TO correct you
 * need to set 2 x 100 = 200 ( as each unit [°C x 100], i.e. one tick corresponds to 0.01°C )
 *
 * August 2018: all this does for now is lower the SCD30 temperature reading to with the offset value
 * over a period of 10 min, while increasing the humidity readings. NO impact on the CO2 readings.
 *
 * The value can NOT be negative as it will cause uncontrolled temperature and humidity results.
 *
 * (see the document in the extras directory of the library)
 *
 * Param tempoffset :
 *  to increase the temperature each unit [°C x 100], i.e. one tick corresponds to 0.01°C
 *
 */

boolean SCD30::setTemperatureOffset(uint16_t tempOffset)
{
  return (sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, tempOffset));
}

// For backward compatibility
boolean SCD30::setTemperatureOffset(float tempOffset)
{
  return (sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, uint16_t (tempOffset * 100)));
}

/* get Altitude Compensation see 1.4.8
 * Added August 2020
 *
 */
boolean SCD30::getAltitudeCompensation(uint16_t *val)
{
  uint8_t tmp[2];
  *val = 0;

  if (SCD_DEBUG > 0)  printf("Reading Altitude compensation\n");

  // request from SCD30
  if (ReadFromSCD30(COMMAND_SET_ALTITUDE_COMPENSATION, tmp, 2) != 2) return(false);

  *val = tmp[0] << 8 | tmp[1];

  return(true);
}


/* Set the altitude compenstation. See 1.4.8.
 *
 * Setting altitude is disregarded when an ambient pressure is given to the sensor,
 * Altitude value is saved in non-volatile memory. The last set value will be used for altitude compensation after repowering.
 *
 * Setting the argument to zero will deactivate the ambient pressure compensation
 */
boolean SCD30::setAltitudeCompensation(uint16_t altitude)
{
  return(sendCommand(COMMAND_SET_ALTITUDE_COMPENSATION, altitude));
}

/* Set the pressure compensation. This is passed during measurement startup.
 * mbar can be 700 to 1200
 *
 * Setting altitude is disregarded when an ambient pressure is given to the sensor,
 * Altitude value is saved in non-volatile memory. The last set value will be used for altitude compensation after repowering.
 *
 * Setting the argument to zero will deactivate the ambient pressure compensation
 */
boolean SCD30::setAmbientPressure(uint16_t pressure_mbar)
{
  if (SCD_DEBUG > 0)  printf("Set Ambient Pressure\n");

  return (beginMeasuring(pressure_mbar));
}

/* Get Forced Recalibration value (FRC) see 1.4.6
 * Added August 2020
 *
 * The FRC method imposes a permanent update of the CO2calibration curve which persists after repowering the sensor.
 * The most recently used reference value is retained in volatile memory and can be read out with the command sequence given below.
 * After repowering the sensor, the command will return the standard reference value of 400 ppm
 */
boolean SCD30::getForceRecalibration(uint16_t *val)
{
  uint8_t tmp[2];
  *val =0;

  if (SCD_DEBUG > 0)  printf("Reading Forced Recalibration Factor\n");

  // request from SCD30
  if (ReadFromSCD30(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, tmp, 2) != 2) return(false);

  *val = tmp[0] << 8 | tmp[1];

  return(true);
}

/* Set Forced Recalibration value (FRC) see 1.4.6
 *
 * Setting a reference CO2concentration by the method described here will always supersede
 * corrections from the ASC(see chapter 1.4.6) and vice-versa.
 * The reference CO2concentration has to be within the range 400 ppm ≤ cref(CO2) ≤2000 ppm
 */
boolean SCD30::setForceRecalibration(uint16_t val)
{

    if(val < 400 || val > 2000) val = 0;   //Error check
    return (sendCommand(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, val));
}

/* Begins continuous measurements see 1.4.1
 *
 * Continuous measurement status is saved in non-volatile memory. When the sensor
 * is powered down while continuous measurement mode is active SCD30 will measure
 * continuously after repowering without sending the measurement command.
 * Returns true if successful
 */
boolean SCD30::beginMeasuring(uint16_t pressureOffset)
{
  if(pressureOffset < 700 || pressureOffset > 1200) pressureOffset = 0; //Error check

  if (SCD_DEBUG > 0) printf("Begin measuring with pressure offset %d\n", pressureOffset);

  return(sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressureOffset));
}

// Overload - no pressureOffset
boolean SCD30::beginMeasuring(void)
{
  return(beginMeasuring(0));
}

/* Stop continuous measurement. see 1.4.2
 * return:
 *  true = OK
 *  false  = error
 */
boolean SCD30::StopMeasurement(void)
{
  return(sendCommand(CMD_STOP_MEAS));
}

/* Get Measurement Interval see 1.4.3
 * Added August 2020
 *
 * Interval in seconds.Available range:[2 ... 1800] given in 2 byte in the order MSB, LSB
 */
boolean SCD30::getMeasurementInterval(uint16_t *val)
{
  uint8_t tmp[2];
  *val = 0;

  if (SCD_DEBUG > 0) printf("Read measurement interval \n");

  // request from SCD30
  if (ReadFromSCD30(COMMAND_SET_MEASUREMENT_INTERVAL, tmp, 2) != 2) return(false);

  *val = tmp[0] << 8 | tmp[1];

  return(true);
}

/* Sets interval between measurements
 * 2 seconds to 1800 seconds (30 minutes)
 * return
 *  true = OK
 *  false  = error
 */
boolean SCD30::setMeasurementInterval(uint16_t interval)
{
  if (SCD_DEBUG > 0) printf("set measurement interval \n");

  if (interval < 2 || interval > 1800) return(false);
  return(sendCommand(COMMAND_SET_MEASUREMENT_INTERVAL, interval));
}

// Returns true when data is available. see 1.4.5
boolean SCD30::dataAvailable()
{
  uint8_t tmp[2];

  // request from SCD30
  if (ReadFromSCD30(COMMAND_GET_DATA_READY, tmp, 2) != 2) return(false);

  if (tmp[1] == 1) return(true);

  return (false);
}

//Get 18 bytes from SCD30. see 1.4.5

//Updates global variables with floats
//Returns true if success
boolean SCD30::readMeasurement()
{
  uint32_t tempCO2 = 0;
  uint32_t tempHumidity = 0;
  uint32_t tempTemperature = 0;
  uint8_t  temp[12];

  //Verify we have data from the sensor
  if (! dataAvailable() ) return (false);

  // request from SCD30
  if (ReadFromSCD30(COMMAND_READ_MEASUREMENT, temp, 12) != 12) return(false);

  tempCO2 = temp[0] << 24 | temp[1] << 16 | temp[2] << 8 | temp[3];
  tempTemperature = temp[4] << 24 | temp[5] << 16 | temp[6] << 8 | temp[7];
  tempHumidity  = temp[8] << 24 | temp[9] << 16 | temp[10] << 8 | temp[11];

  if (SCD_DEBUG > 0)
      printf(" CO2 : 0x%04X, Temperature 0x%04X, Humidity 0x%04X\n",tempCO2, tempTemperature, tempHumidity  );

  //Now copy the uint32s into their associated floats
  memcpy(&co2, &tempCO2, sizeof(co2));
  memcpy(&temperature, &tempTemperature, sizeof(temperature));
  memcpy(&humidity, &tempHumidity, sizeof(humidity));

  //Mark our global variables as fresh
  co2HasBeenReported = false;
  humidityHasBeenReported = false;
  temperatureHasBeenReported = false;

  return (true); //Success! New data available in globals.
}

/* Paulvha : august 2018
 * Set for debugging the driver
 *
 * 0 = disable debug messages
 * 1 = sent/receive messages
 * 2 = like 1 + protocol errors
 *
 * This can be called BEFORE performing the begin() call.
 */

void SCD30::setDebug(int val)
{
    SCD_DEBUG = val;
}

/* paulvha : august 2018
 *
 * decode the command that is being sent */
void SCD30::debug_cmd(uint16_t command)
{
    printf("Command 0x%X : ", command);

    switch(command)
    {
        case 0x0010:
            printf("COMMAND_CONTINUOUS_MEASUREMENT");
            break;
        case 0x0104:
            printf("CMD_STOP_MEAS");
            break;
        case 0x4600:
            printf("COMMAND_SET_MEASUREMENT_INTERVAL");
            break;
        case 0x0202:
            printf("COMMAND_GET_DATA_READY");
            break;
        case 0x300:
            printf("COMMAND_READ_MEASUREMENT");
            break;
        case 0x5306:
            printf("COMMAND_AUTOMATIC_SELF_CALIBRATION");
            break;
        case 0x5204:
            printf("COMMAND_SET_FORCED_RECALIBRATION_FACTOR");
            break;
        case 0x5403:
            printf("COMMAND_SET_TEMPERATURE_OFFSET");
            break;
        case 0x5102:
            printf("COMMAND_SET_ALTITUDE_COMPENSATION");
            break;
        case 0xD033:
            printf("CMD_READ_SERIALNBR");
            break;
        case 0xD025:
            printf("CMD_READ_ARTICLECODE");
            break;
        case 0x0006:
            printf("CMD_START_SINGLE_MEAS");
            break;
        case 0xD100:
            printf("CMD_GET_FW_LEVEL");
            break;
        default:
            printf("0x%04X : COMMAND_UNKNOWN");
            break;
    }
}

/* Sends a command along with arguments and CRC
 * return
 *  true = OK
 *  false = error
 */
boolean SCD30::sendCommand(uint16_t command, uint16_t arguments)
{
  uint8_t data[2];
  data[0] = arguments >> 8;
  data[1] = arguments & 0xFF;
  uint8_t crc = computeCRC8(data, 2); //Calc CRC on the arguments only, not the command

  if (SCD_DEBUG > 0)
  {
       printf("sending to I2C address 0x%x, ",SCD30_ADDRESS);
       debug_cmd(command);
       printf(", arguments 0x%x, CRC 0x%x\n",arguments, crc);
  }

  // load the I2C driver buffer
  _i2cPort->beginTransmission(SCD30_ADDRESS);
  _i2cPort->write(command >> 8); //MSB
  _i2cPort->write(command & 0xFF); //LSB

  _i2cPort->write(arguments >> 8); //MSB
  _i2cPort->write(arguments & 0xFF); //LSB
  _i2cPort->write(crc);

  // now that all is in the buffer, start sending and end transmission
  if (_i2cPort->endTransmission() != 0)
  {
      if (SCD_DEBUG > 1)  printf("Sensor did not ACK\n");
      return (false); //Sensor did not ACK
  }

  return (true);
}

// Sends just a command, no arguments, no CRC
boolean SCD30::sendCommand(uint16_t command)
{
  if (SCD_DEBUG > 0)
  {
       printf("sending to I2C address 0x%x, ",SCD30_ADDRESS);
       debug_cmd(command);
   }

  // load the I2C driver buffer
  _i2cPort->beginTransmission(SCD30_ADDRESS);
  _i2cPort->write(command >> 8); //MSB
  _i2cPort->write(command & 0xFF); //LSB

  // now that all is in the buffer, start sending and end transmission
  if (_i2cPort->endTransmission() != 0)
  {
    if (SCD_DEBUG > 1)  printf("Sensor did not ACK\n");
    return (false);
  }

  return (true);
}

//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t SCD30::computeCRC8(uint8_t data[], uint8_t len)
{
  uint8_t crc = 0xFF; //Init with 0xFF

  for (uint8_t x = 0 ; x < len ; x++)
  {
    crc ^= data[x]; // XOR-in the next input byte

    for (uint8_t i = 0 ; i < 8 ; i++)
    {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }

  return crc; //No output reflection
}
