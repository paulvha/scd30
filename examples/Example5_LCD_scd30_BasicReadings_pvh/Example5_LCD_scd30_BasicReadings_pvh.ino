/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751
  
  This example prints the current CO2 level, relative humidity, and temperature in C.

  Hardware Connections:
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32, but otherwise you can 
  connect directly on the SDA / SCL (see below) or other
  * obtain the LCD https://www.sparkfun.com/products/16396
  * Plug the device into an available Qwiic port or I2C
  * Connect the LCD to Qwiic port or I2C, 
  * Connect optional button to digital pin
  
  Software:
  * Install the LCD software http://librarymanager/All#SparkFun_SerLCD.
  * In case ESP32 see instructions below
  * Set the parameters in the sketch below
  
  Open the serial monitor at 115200 baud to see the output

  **********************************************************************
  * Versioning:
  **********************************************************************
  august 2018 / paulvha:
    Support ESP8266-Thing
    include option to debug driver

  January 2019 / paulvha
    Added SoftWire support for ESP32

  October 2020 / paulvha
    Created this example based on example 1 to display the output also on an LCD
    used the https://www.sparkfun.com/products/16396
    
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

  ==== CONNECT TO ARDUINO ====

  SCD30   Arduino    LCD
    VCC --- 5V------ RAW 3V3 - 9
    GND --- GND----- GND 
    SCL --- SCL -----CL   ( SCL Arduino UNO A5)
    SDA --- SDA ---- DA   ( SDA Arduino UNO A4)
     
  The LCD has pull-up resistors to 3v3 already

            
  === CONNECT TO ESP8266-THING ===

  Make sure to cut the link and have a jumper on the DTR/reset.
  Include the jumper for programming, remove before starting serial monitor

  SCD30    ESP8266  LCD
    GND --- GND ----GND
    VCC --- 3V3 ----3V3 - 9
    SCL --- SCL --- CL  
    SDA --- SDA --- DA  

  The LCD has pull-up resistors to 3v3 already
  Given that SCD30 is using clock stretching the driver has been modified to deal with that.

  === CONNECT TO ESP32-THING ===

  SCD30    ESP32    LCD
    GND --- GND --- GND
    VCC --- USB --- RAW 3V3 -9
    SCL --- 22  --- CL  
    SDA --- 21  --- DA  

  The LCD has pull-up resistors to 3v3 already
  
  !!!! WARNING !!!!! WARNING !!!!! WARNING !!!!! WARNING !!!!! WARNING !!!!! WARNING !!!!!
  Given that SCD30 is using clock stretching, SoftWire is selected by the driver to deal with that.
  This means however a small change is needed for in SerLCD.h 
  In line 5, change
  #include <Wire.h>
  to
  #include "../../scd30/src/SoftWire/SoftWire.h"
 
  Make sure to press the GPIO0 button for connect / upload

***********************************************************************************************
* https://www.sparkfun.com/products/16396
*
* MAKE SURE TO INSTALL http://librarymanager/All#SparkFun_SerLCD.
* 
* The SparkFun SerLCD is an AVR-based, serial enabled LCD that provides a simple and cost 
* effective solution for adding a 16x2 Black on RGB Liquid Crystal Display into your project. 
* Both the SCD30 and LCD are connected on the same WIRE device.
*
  The Qwiic adapter should be attached to the display as follows. If you have a model (board or LCD)
  without QWiic connect, or connect is indicated.
  Display  / Qwiic Cable Color        LCD -connection without Qwiic
  GND      / Black                    GND
  RAW      / Red                      3V3 -9 v
  SDA      / Blue                     I2c DA
  SCL      / Yellow                   I2C CL

  Note: If you connect directly to a 5V Arduino instead, you *MUST* use
  a level-shifter on SDA and SCL to convert the i2c voltage levels down to 3.3V for the display.
  
  !!!! Measured with a scope it turns out that the pull up is already to 3V3 !!!!
  If ONLYONBUTTON is set, connect a push-button switch between pin BUTTONINPUT and ground.
   
  No support.. use as you like.. good luck !!
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
//                SELECT THE INTERFACES                                 //
//////////////////////////////////////////////////////////////////////////
#define SCD30WIRE Wire
#define LCDCON Wire

//////////////////////////////////////////////////////////////////////////
//                SELECT LCD settings                                   //
//////////////////////////////////////////////////////////////////////////
// CO2 limits what is good or bad or ugly ?
//
// CO2 outside is ~ 400PPM      (used to be 200 - 300ppm around 1900 but is increasing due to polution)
// double that size gets bad    (hence 700 is set as limit so you can open windows on-time)
// treetime is ugly             (hence 1000 is set as limit as you really do not want 1200ppm)
///////////////////////////////////////////////////////////////////////// 
#define LCDTEMPCELSIUS  true     // set to false to display temperature in Fahrenheit

#define LCDBACKGROUNDCOLOR  1    // Normal background color: 1 = white, 2 = red, 3 = green 4 = blue 5 = off

#define CO2LIMITLOW  700         // Background LCD color will start LCDBACKGROUNDCOLOR and turn to blue if 
                                 // CO2 is above this limit to return to LCDBACKGROUNDCOLOR background when below. 
                                 // set to zero to disable

#define CO2LIMITHIGH 1000        // Background LCD color will start LCDBACKGROUNDCOLOR and turn to red if 
                                 // CO2 is above this limit to return to CO2LIMITLOW and then LCDBACKGROUNDCOLOR background when below. 
                                 // set to zero to disable
                                                                  
#define ONLYONLIMIT false        // only display the results on the LCD display if the CO2LIMITLOW or CO2LIMITHIGH is exceeded
                                 // set to false disables this option.
                                 // do NOT select together with ONLYONBUTTON
                                 // make sure to set CO2LIMITLOW > 0  && CO2LIMITHIGH > 0(compile will fail)
                               
#define ONLYONBUTTON false       // only display the results on the LCD display (blue) if the CO2LIMITLOW or (red) if CO2LIMITHIGH
                                 // is exceeded OR for LCDTIMEOUT seconds if a button is pushed
                                 // set to false disables this option
                                 // do NOT select together with ONLYONLIMIT
                                 // if CO2LIMITLOW is zero the LCD will only display when button is pressed  
#if ONLYONBUTTON == true                    
#define BUTTONINPUT  10         // Digital input where button is connected for ONLYONBUTTON between GND
                                 // is ignored if ONLYONBUTTON is set to false
                                 // Artemis / Apollo3 set as D27
                                 // ESP32, Arduino set as 10

#define LCDTIMEOUT 10            // Number of seconds LCD is displayed after button was pressed 
#endif                           // is ignored if ONLYONBUTTON is set to false        
                                                               
//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

// checks will happen at pre-processor time to 
#if ONLYONLIMIT == true && ONLYONBUTTON == true 
#error you can NOT set BOTH ONLYONLIMIT and ONLYONBUTTON to true
#endif

#if ONLYONLIMIT == true && ( CO2LIMITLOW == 0 ||CO2LIMITHIGH == 0)
#error you MUST set CO2LIMITLOW and CO2LIMITHIGH when ONLYONLIMIT to true
#endif

#include "paulvha_SCD30.h"
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

SerLCD lcd;         // Initialize the library with default I2C address 0x72
SCD30 airSensor;    // Initialize the library with default I2C address 0x61

byte ppm[8] = {     // ppm custom character
  0b11100,
  0b10100,
  0b11100,
  0b10000,
  0b00111,
  0b00101,
  0b00111,
  0b00100
};

#if LCDTEMPCELSIUS == true 
byte deg[8] = {     // celsius custom character
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b01111,
  0b01100,
  0b01100,
  0b01111
};
#else
byte deg[8] = {   // Fahrenheit custom character
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b01111,
  0b01100,
  0b01110,
  0b01100
};
#endif

void setup()
{
  SCD30WIRE.begin();

  Serial.begin(115200);
  Serial.println("SCD30 Example 5 : basic reading with SparkFun OpenLCD / SerLCD");

  airSensor.setDebug(scd_debug);

  // This will cause readings to occur every two seconds
  if (! airSensor.begin(SCD30WIRE))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }
  
  // initialize LCD
  lcdinit();
  
  // display device info
  DeviceInfo();
}

void loop()
{
  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");
    Serial.print(airSensor.getCO2());
    
#if LCDTEMPCELSIUS == true
    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);
#else
    Serial.print(" temp(F):");
    Serial.print(airSensor.getTemperatureF(), 1);
#endif

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
    
    printLCD(true);
    
  }
  else {
    Serial.println("No data");
    printLCD(false);
  }
  
  // delay for 2 seconds but capture button pressed
  for (int i=0; i < 10;i++){
    delay(200);
    checkButton();
  }
}

// display the SCD30 device information
void DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) +1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)
  if (airSensor.getSerialNumber(buf)) {
   Serial.print(F("SCD30 serial number : "));
   Serial.println(buf);
   
   lcd.setCursor(0, 0);            // pos 0, line 0
   lcd.write("snr: ");
   lcd.write(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val) ) {
    Serial.print("SCD30 Firmware level: Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
    
    lcd.setCursor(0, 1);            // pos 0, line 1
    lcd.write("Fw:  ");
    sprintf(buf,"%d.%d", val[0], val[1]);
    lcd.write(buf);
    
  }
  else {
    Serial.println("Could not obtain firmware level");
  }
  
  delay(5000);
}

// checks for button pressed to set the LCD on and keep on for 
// LCDTIMEOUT seconds after button has been released.
// return true to turn on OR false to turn / stay off.
bool checkButton()
{
#if ONLYONBUTTON == true
  static unsigned long startTime = 0;
  
  // button pressed ?
  if (! digitalRead(BUTTONINPUT)){
    startTime = millis();
  }

  if (startTime > 0) {
    if (millis() - startTime < (LCDTIMEOUT*1000))  return true;
    else  startTime = 0;      // reset starttime
  }    

  return false;

#endif //ONLYONBUTTON
  return true;  
}

// initialize the LCD
void lcdinit()
{
  lcd.begin(LCDCON);

  lcd.createChar(0, ppm);     // create custom characters
  lcd.createChar(1, deg);

  lcdsetbackground();         // set background

#if ONLYONBUTTON == true
  pinMode(BUTTONINPUT,INPUT_PULLUP);
#endif
}

// set requested background color
void lcdsetbackground()
{
  
#if ONLYONLIMIT == true
  lcd.setBacklight(0, 0, 0);  // off
  return;
#endif

#if ONLYONBUTTON == true
  if(! checkButton()) {
    lcd.setBacklight(0, 0, 0);  // off
    return;
  }
#endif //ONLYONBUTTON

  switch(LCDBACKGROUNDCOLOR){
    
    case 2:   // red
      lcd.setBacklight(255, 0, 0); // bright red
      break;
    case 3:   // green
      lcd.setBacklight(0, 255, 0); // bright green
      break;
    case 4:   // blue
      lcd.setBacklight(0, 0, 255); // bright blue
      break;
    case 5:   // off
      lcd.setBacklight(0, 0, 0);
      break;
    case 1:   // white
    default:
      lcd.setBacklight(255, 255, 255); // bright white
  }
}

// print results on LCD
// @parameter dd : true is display new data else no-data indicator
void printLCD(bool dd)
{
  char buf[10];
  int co2 = airSensor.getCO2();
  static bool limitLowWasSet = false;       // low limit has been set (true)
  static bool limitHighWasSet = false;      // high limit has been set
  static bool MeasureInd = true;
  
// change background to red on high limit (if limit was set)
#if CO2LIMITHIGH > 0 
 
  if (co2 > CO2LIMITHIGH){
    // change once..
    if(! limitHighWasSet){
      lcd.setBacklight(255, 0, 0); // bright red
      limitHighWasSet = true;
    }
  }
  else if (limitHighWasSet){
    lcd.setBacklight(0, 0, 255); // bright blue
    limitHighWasSet = false;
  }
#endif //CO2LIMITHIGH

#if CO2LIMITLOW > 0 
  if ( ! limitHighWasSet){

    // only change is high limit was not set
    if (co2 > CO2LIMITLOW ){
      // change once..
      if(! limitLowWasSet){
        lcd.setBacklight(0, 0, 255); //bright blue
        limitLowWasSet = true;
      }
    }
    else if (limitLowWasSet){
      lcdsetbackground();           // reset to original request
      limitLowWasSet = false;
    }
  }
#endif //CO2LIMITLOW
  
// only display if limit has been reached  
#if ONLYONLIMIT == true
  if(! limitLowWasSet && ! limitHighWasSet) {
    lcd.clear();
    return;
  }
#endif //ONLYONLIMIT

// only display if button was pressed or limit has been reached
#if ONLYONBUTTON == true
  if(! checkButton() && ! limitLowWasSet && !limitHighWasSet) {
    lcd.clear();
    lcd.setBacklight(0, 0, 0);  // off
    return;
  }
#endif //ONLYONBUTTON

  // if no data available indicate with . and return
  if (!dd) {
    lcd.setCursor(15, 0);            // pos 15, line 0
    
    // display measurement indicator 
    if (MeasureInd)  lcd.write(".");
    else lcd.write(" ");
    
    MeasureInd = !MeasureInd;
    return;
  }
  
  // just in case next no-data, start display .
  MeasureInd = true;
  
  lcd.clear();
  lcd.write("Co2: Temp: Hum:");

  lcd.setCursor(0, 1);            // pos 0, line 1
  
  sprintf(buf,"%d",co2);
  lcd.write(buf);
  lcd.writeChar(0);               // add custom ppm

  lcd.setCursor(5, 1);            // pos 5, line 1
  
#if LCDTEMPCELSIUS == true    
  FromFloat(buf, airSensor.getTemperature(),1);
#else
  FromFloat(buf, airSensor.getTemperatureF(),1);  
#endif
  
  lcd.write(buf);
  lcd.writeChar(1);               // add customer degree

  lcd.setCursor(11, 1);           // pos 11, line 1
  FromFloat(buf, airSensor.getHumidity(),1);
  lcd.write(buf);
  lcd.write('%');
}

// This is a workaround as sprintf on Artemis/Apollo3 is not recognizing %f (returns empty)
// based on source print.cpp/ printFloat
int FromFloat(char *buf, double number, uint8_t digits) 
{ 
  char t_buf[10];
  buf[0] = 0x0;
  
  if (isnan(number)) {
    strcpy(buf,"nan");
    return 3;
  }
  
  if (isinf(number)) {
    strcpy(buf,"inf");
    return 3;
  }

  if (number > 4294967040.0 || number <-4294967040.0) {
    strcpy(buf,"ovf");
    return 3;
  }
    
  // Handle negative numbers
  if (number < 0.0)
  {
     strcat(buf,"-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;

  sprintf(t_buf,"%ld", int_part);
  strcat(buf,t_buf);
  
  if (digits > 0) {
    
    // Print the decimal point, but only if there are digits beyond
    strcat(buf,".");  
  
    // Extract digits from the remainder one at a time
    while (digits-- > 0)
    {
      remainder *= 10.0;
      unsigned int toPrint = (unsigned int)(remainder);
      sprintf(t_buf,"%d", toPrint);
      strcat(buf,t_buf);
      remainder -= toPrint; 
    } 
  }

  return (int) strlen(buf);
}
