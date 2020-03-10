/**
 *  SoftWire - Version 1.0 / paulvha / February 2019
 *
 * This is a bit-banging I2C implemenation that is taken from the ESP8266.
 * It has been adjusted to work on an ESP32 and support clock-stretching.
 *
 * The hardware I2C on an ESP32 is known for NOT supporting clock stretching.
 *
 * While it is aimed and tested for the SCD30, it should work for other
 * devices on the ESP32 as well.
 *
 * Below the original heading on the ESP8266
 */

/*
  si2c.c - Software I2C library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if defined(ARDUINO_ARCH_ESP32)

#include "twi.h"

unsigned int preferred_si2c_clock = 100000;
uint8_t twi_dcount = 4;
static unsigned char twi_sda, twi_scl;
static uint32_t twi_clockStretchLimit;

#define SDA_LOW()   (SETLOW(twi_sda))
#define SDA_HIGH()  (pinMode(twi_sda, INPUT))
#define SDA_READ()  (digitalRead(twi_sda))
#define SCL_LOW()   (SETLOW(twi_scl))
#define SCL_HIGH()  (pinMode(twi_scl, INPUT | PULLUP))
#define SCL_READ()  (digitalRead(twi_scl))
#define SETLOW(p)   {digitalWrite(p, LOW); pinMode(p,OUTPUT);}

// fine tine the clock stretch so provided limit value leads to about 1us
#define TWI_CLOCK_STRETCH_MULTIPLIER 5

void twi_setClock(unsigned int freq){
    if (freq <= 80000) twi_dcount = 6;       // 69 Khz
    else if (freq <= 100000) twi_dcount = 4; // 98 khz
    else twi_dcount = 1;                     // 185 Khz
}

void twi_setClockStretchLimit(uint32_t limit){
  twi_clockStretchLimit = limit * TWI_CLOCK_STRETCH_MULTIPLIER;
}

void twi_init(unsigned char sda, unsigned char scl){
  twi_sda = sda;
  twi_scl = scl;

  pinMode(twi_sda, INPUT_PULLUP);
  pinMode(twi_scl, INPUT_PULLUP);

  twi_setClock(preferred_si2c_clock);
  twi_setClockStretchLimit(500);       // default value is ~500 uS
}

void twi_stop(void){
  pinMode(twi_sda, INPUT);
  pinMode(twi_scl, INPUT);
}

/* usleep has an overhead to useconds
 *          cycle time
 * v = 6 =>  14,4us => 69 Khz
 * v = 5 =>  12us   => 83Khz
 * v = 4 =>  10.2us => 98khz
 * V = 3 =>  9,5us  => 105Khz
 * v = 2 =>  7,4 us => 135Khz
 * v = 1 =>  5,4 us => 185Khz
 * */
void IRAM_ATTR twi_delay(uint8_t v){
  usleep(v);
}

static bool twi_write_start(void) {
  unsigned int i;

  SCL_HIGH();
  SDA_HIGH();

  if (SDA_READ() == 0) {
    twi_recovery();
    return false;
  }

  twi_delay(twi_dcount);
  SDA_LOW();
  twi_delay(twi_dcount);
  return true;
}

static bool twi_write_stop(void){
  uint32_t i = 0;
  SCL_LOW();
  SDA_LOW();
  twi_delay(twi_dcount);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < twi_clockStretchLimit); // Clock stretching
  twi_delay(twi_dcount);
  SDA_HIGH();
  twi_delay(twi_dcount);

  return true;
}

static bool twi_write_bit(bool bit) {
  uint32_t i = 0;
  SCL_LOW();
  if (bit)  SDA_HIGH();
  else  SDA_LOW();
  twi_delay(twi_dcount);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < twi_clockStretchLimit); // Clock stretching
  twi_delay(twi_dcount);
  return true;
}

static bool twi_read_bit(void) {
  uint32_t i = 0;
  SCL_LOW();
  SDA_HIGH();
  twi_delay(twi_dcount);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < twi_clockStretchLimit);// Clock stretching
  bool bit = SDA_READ();
  twi_delay(twi_dcount);
  return bit;
}

static bool twi_write_byte(unsigned char byte) {
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) {
    twi_write_bit(byte & 0x80);
    byte <<= 1;
  }
  return !twi_read_bit();//NACK/ACK
}

static unsigned char twi_read_byte(bool nack) {
  unsigned char byte = 0;
  unsigned char bit;
  for (bit = 0; bit < 8; bit++)
    byte = (byte << 1) | twi_read_bit();
  twi_write_bit(nack);
  return byte;
}

 /* if the bus is not 'clear' try the recommended recovery sequence, START, 9 Clocks, STOP
  * https://www.analog.com/media/en/technical-documentation/application-notes/54305147357414an686_0.pdf
  * The procedure is as follows:
  * 1)  Master tries to assert a Logic 1 on the SDA line
  * 2)  Master still sees a Logic 0 and then generates a clock
  *     pulse on SCL (1-0-1 transition)
  * 3)  Master  examines  SDA.  If  SDA  =  0,  go  to  Step  2;  if
  *     SDA = 1, go to Step 4
  * 4) Generate a STOP condition
  */
void twi_recovery()
{
    uint8_t x, i;

    for(x = 0 ; x < 10 ; x++)
    {
        SDA_HIGH();
        SCL_HIGH();

        if(!SDA_READ() || !SCL_READ()) { // bus in busy state
            SDA_HIGH();
            SCL_HIGH();
            twi_delay(5);
            SDA_LOW();
            i = 0;

            while(SDA_READ() == 0 && (i++) < 10){
                twi_delay(5);
                SCL_LOW();
                twi_delay(5);
                SCL_HIGH();
            }

            twi_delay(5);
            SDA_HIGH();
        }
        else
             break;
    }

    twi_write_stop();
}

unsigned char twi_writeTo(unsigned char address, unsigned char * buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start()) return(I2C_SDA_HELD_LOW_AFTER_INIT); //line busy
  if(!twi_write_byte(((address << 1) | 0) & 0xFF)) {
    if (sendStop) twi_write_stop();
    return 2; //received NACK on transmit of address
  }

  for(i=0; i<len; i++) {
    if(!twi_write_byte(buf[i])) {
      if (sendStop)  twi_write_stop();
      return 3; //received NACK on transmit of data
    }
  }

  if(sendStop) twi_write_stop();

  if (SDA_READ() == 0) twi_recovery();

  return(I2C_OK);
}

unsigned char twi_readFrom(unsigned char address, unsigned char* buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start())    return(I2C_SDA_HELD_LOW_AFTER_INIT); //line busy

  if(!twi_write_byte(((address << 1) | 1) & 0xFF)) {
    if (sendStop)  twi_write_stop();
    return (2) ; //received NACK on transmit of address
  }

  for(i=0; i<(len-1); i++)
    buf[i] = twi_read_byte(false);

  buf[len-1] = twi_read_byte(true);

  if(sendStop) twi_write_stop();

  if (SDA_READ() == 0) twi_recovery();

  return(I2C_OK);
}

uint8_t twi_status() {
    if (SCL_READ()==0)
        return I2C_SCL_HELD_LOW;              // SCL held low by another device, no procedure available to recover
    int clockCount = 20;

    while (SDA_READ()==0 && clockCount>0) {   // if SDA low, read the bits slaves have to sent to a max
        --clockCount;
        twi_read_bit();
        if (SCL_READ()==0)
            return I2C_SCL_HELD_LOW_AFTER_READ; // I2C bus error. SCL held low beyond slave clock stretch time
    }

    if (SDA_READ()==0)
        return I2C_SDA_HELD_LOW;             // I2C bus error. SDA line held low by slave/another_master after n bits.

    if(!twi_write_start())
        return I2C_SDA_HELD_LOW_AFTER_INIT;  // line busy. SDA again held low by another device. 2nd master?

    return I2C_OK;                           //all ok
}

#endif //ARDUINO_ARCH_ESP32
