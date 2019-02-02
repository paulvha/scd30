/*
 *
 *   This sketch is based on the WiFiWebServer &  on the https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server
 *
 *   Standard it demonstrates how to set up a simple HTTP-like server.
 *   The server will set a GPIO pin (LED_PIN defined below) depending on the request
 *     http://server_ip/gpio/0 will set the GPIO low,
 *     http://server_ip/gpio/1 will set the GPIO high
 *   server_ip is the IP address of the ESP8266 module, will be printed to Serial when the module is connected.
 *
 **********************************************************************************************************
 * Extended Paulvha : august 2018
 *
 * For SCD30 readings
 * http://server_ip/tmp  : will provide the temperature
 * http://server_ip/hum  : will provide the humidity
 * http://server_ip/CO2  : will provide the CO2 PPM
 *
 * This example demonstrates how to connect the SCD30 to an ESP8266 THING, setup as an Access Point and read the results over the network
 *
 * Make sure to cut the link and have a jumper on the DTR/reset. Include the jumper for programming, remove before starting serial monitor on 115000 baud.
 *
 * Hardware Connections:
 * Attach the Qwiic Shield to your ESP32866 THING
 * Plug the sensor onto the shield
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
 */

#include <ESP8266WiFi.h>
#include "paulvha_SCD30.h"

////////////////////////////////////////////////////
// WiFi Definitions (MUST be set to your network) //
////////////////////////////////////////////////////
const char* ssid = "your-ssid";
const char* password = "your-password";

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5;      // Thing's onboard, green LED

//////////////////////////////////////////////////////////////////////////
//////////// Change to the pressure in mbar on your location /////////////
/////// for better SCD30 - CO2 results (between 700 and 1200 mbar)  //////
//////////////////////////////////////////////////////////////////////////
#define pressure 1028

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

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

// create an instance of the SCD30
SCD30 airSensor;

float val2;
int detect_SDC30 = 0;

void setup() {

  initHardware();
  setupWiFi();

  // Start the server
  server.begin();
  Serial.println(F("Server started"));

  // Print the IP address & instructions
  Serial.print(F("Try "));
  Serial.print(WiFi.localIP());
  Serial.println(F("/gpio/1, /gpio/0, /tmp, /hum or /co2."));
}

void loop() {

  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  Serial.println(F("new client"));
  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  // Match the request
  int val = -1;

  if (req.indexOf("/gpio/0") != -1)
    val = 0;

  else if (req.indexOf("/gpio/1") != -1)
    val = 1;

  else if (req.indexOf("/tmp") != -1)
    val = -4; // Will provided the temp from SCD30

  else if (req.indexOf("/hum") != -1)
    val = -5; // Will provided the humidity from SCD30

  else if (req.indexOf("/co2") != -1)
    val = -6; // Will provided the Co2 from SCD30

  // Otherwise request will be invalid. We'll say as much in HTML

  // Set GPIO according to the request
  if (val >= 0)
    digitalWrite(LED_PIN, val);

  client.flush();

 // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<body>";
  s += "<p><font size=\"20\">";

  // If we're setting the GPIO, print out a message saying we did
  if (val >= 0)
  {
    s += "GPIO-";
    s += LED_PIN;
    s += " is now ";
    s += (val)?"on":"off";
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
      s += "%";
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
    }
    else
      s += "SCD30 not detected";
  }

  else
  {
    s += "Invalid Request.<br> Try /gpio/1, /gpio/0, /tmp, /hum or /co2.";
  }

  s += "</font></p>";
  s += "</body>";
  s += "</html>\n";


  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println(F("Client disonnected"));

  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

void setupWiFi()
{
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println(F("WiFi connected"));
}

void initHardware()
{
  Serial.begin(115200);

  // set GPIO
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setup I2C
  Wire.begin();

  // set SCD30 driver debug level (only in case of errors)
  // requires serial monitor (remove DTR-jumper before starting monitor)
  // 0 : no messages
  // 1 : request sending and receiving
  // 2 : request sending and receiving + show protocol errors
  airSensor.setDebug(scd_debug);

  //This will cause SCD30 readings to occur every two seconds
  if (airSensor.begin() == true)
  {
    // Pressure adjustment
    airSensor.setAmbientPressure(pressure); //Current ambient pressure in mBar: 700 to 1200

    detect_SDC30 = 1;
  }
}
