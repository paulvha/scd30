/*
 * Based on the https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server
 *
 * After uploading this sketch, find another device that you can connect to a WiFi network – phone, laptop, etc.
 * Look for a network called “ESP8266-SCD30 XXXX”, where XXXX is the last 2 bytes of the Thing’s MAC address.
 *
 * The sketch sets the network’s password to “sparkfun”.
 *
 * After connecting to your Thing’s AP network, load up a browser and point it to 192.168.4.1/read.
 * The Thing should serve up a web page showing you its ADC and digital pin 12 readings.
 *
 * After that, give 192.168.4.1/led/0 and 192.168.4.1/led/1 a try, and keep an eye on the Thing’s green LED while you do.
 *
 * License: MIT. See license file for more information but you can
 * basically do whatever you want with this code.
 *
 * Feel like supporting open source hardware?
 * Buy an SCD30 board from SparkFun! https://www.sparkfun.com/products/14751
 * Buy an ESP8266 THING board from SparkFun!  https://www.sparkfun.com/products/13231
 *
 *
 **********************************************************************************************************
 * Extended Paulvha : august 2018
 *
 * 192.168.4.1/blink : will set a blinking led (to stop sent 192.168.4.1/led/0)
 *
 * For SCD30 readings
 * 192.168.4.1/tmp  : will provide the temperature
 * 192.168.4.1/hum  : will provide the humidity
 * 192.168.4.1/CO2  : will provide the CO2 PPM
 *
 * This example demonstrates how to connect the SCD30 to an ESP8266 THING, setup as an Access Point and read the results over the network
 *
 * Hardware Connections:
 * Attach the Qwiic Shield to your ESP32866 THING
 * Plug the sensor onto the shield
 *
 * pin layout SCD30
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
 *
 * ELSE connnect SCD30 as follows
 *   GND TO GND
 *   VCC to 3V3
 *   SCL to SCL
 *   SDA to SDA
 *
 *  Given that SCD30 is using clock stretching the driver has been modified to deal with that.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ESP8266WiFi.h>
#include "paulvha_SCD30.h"

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "sparkfun";  // password on WIFI

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5;      // Thing's onboard, green LED
const int ANALOG_PIN = A0;  // The only analog pin on the Thing
const int DIGITAL_PIN = 12; // Digital pin to be read

//////////////////////////////////////////////////////////////////////////
//////////// Change to the pressure in mbar on your location /////////////
/////// for better SCD30 - CO2 results (between 700 and 1200 mbar)  //////
//////////////////////////////////////////////////////////////////////////
#define pressure 1028

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (ONLY NEEDED CASE OF SCD30 ERRORS)      //
// For debugging : Make sure to cut the link and have a jumper on the   //
// DTR/reset. Include the jumper for programming, remove before         //
// starting serial monitor on 115000 baud.                              //
//                                                                      //
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
WiFiServer server(80);
SCD30 airSensor;

int val, val1;
float val2;
int detect_SDC30 = 0;

void setup()
{
  initHardware();
  setupWiFi();
  server.begin();
  val = -1 ;  // initialize action
}

void loop()
{
  // Check if a client has connected
  WiFiClient client = server.available();

  if (!client) {

    if ( val == -3 )    // blink was requested earlier
    {
        delay(500);
        digitalWrite(LED_PIN, val1);
        val1 = 1 - val1;      // switch level
    }

    return;
  }

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  // Match the request
  //val = -1; // We'll use 'val' to keep track of both the
                // request type (read/set) and value if set.
  if (req.indexOf("/led/0") != -1)
    val = 0; // Will write LED low

  else if (req.indexOf("/led/1") != -1)
    val = 1; // Will write LED high

  else if (req.indexOf("/read") != -1)
    val = -2; // Will print pin reads

  else if (req.indexOf("/blink") != -1)
    val = -3; // Will blink led

  else if (req.indexOf("/tmp") != -1)
    val = -4; // Will provided the temp from SCD30

  else if (req.indexOf("/hum") != -1)
    val = -5; // Will provided the humidity from SCD30

  else if (req.indexOf("/co2") != -1)
    val = -6; // Will provided the Co2 from SCD30

  // Otherwise request will be invalid. We'll say as much in HTML

  // Set GPIO5 according to the request
  if (val >= 0)
    digitalWrite(LED_PIN, val);

  client.flush();

  // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<body>";
  s += "<p><font size=\"20\">";

  // If we're setting the LED, print out a message saying we did
  if (val >= 0)
  {
    s += "LED is now ";
    s += (val)?"on":"off";
  }
  else if (val == -2)
  { // If we're reading pins, print out those values:
    s += "Analog Pin = ";
    s += String(analogRead(ANALOG_PIN));
    s += "<br>"; // Go to the next line.
    s += "Digital Pin 12 = ";
    s += String(digitalRead(DIGITAL_PIN));
  }
  else if (val == -3)
  {
    s += "blink has been set";
    val1 = 1;
    digitalWrite(LED_PIN, val1);
  }

  //SCD30 readings
  else if (val == -4)
  {
    if (detect_SDC30 == 1)
    {
      s += "Temperature = ";
      val2 = airSensor.getTemperature();
      s += val2;
      s += " celsius";
    }
    else
      s += "SCD30 not detected";
  }

  else if (val == -5)
  {
    if (detect_SDC30 == 1)
    {
      s += "Humidity = ";
      val2 = airSensor.getHumidity();
      s += val2;
      s += "% ";
    }
    else
      s += "SCD30 not detected";
  }

  else if (val == -6)
  {
    if (detect_SDC30 == 1)
    {
      s += "CO2 (PPM) = ";
      val2 = airSensor.getCO2();
      s += val2;
      s += " ";
    }
    else
      s += "SCD30 not detected";
  }

  else
  {
    s += "Invalid Request.<br> Try /led/1, /led/0, / blink, /tmp, /hum, /co2 or /read.";
  }

  s += "</font></p>";
  s += "</body>";
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");

  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266-SCD30 Thing " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

void initHardware()
{
  Serial.begin(115200);

  // set GPIO
  pinMode(DIGITAL_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Don't need to set ANALOG_PIN as input,
  // that's all it can be.

  // setup I2C
  SCD30WIRE.begin();

  // set SCD30 driver debug level (only in case of errors)
  // requires serial monitor (remove DTR-jumper before starting monitor)
  // 0 : no messages
  // 1 : request sending and receiving
  // 2 : request sending and receiving + show protocol errors
  airSensor.setDebug(scd_debug);

  //This will cause readings to occur every two seconds
  if (! airSensor.begin(SCD30WIRE))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }

  // Pressure adjustment
  airSensor.setAmbientPressure(pressure); //Current ambient pressure in mBar: 700 to 1200

  DeviceInfo();

  detect_SDC30 = 1;
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

