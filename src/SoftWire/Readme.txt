SoftWire - Version 1.0 / paulvha / February 2019

Version 1.0.1 / paulvha / March 2020
- fixed warning in twi.c with IDE 1.8.12

This is a bit-banging I2C implemenation that is taken from the ESP8266.
It has been adjusted to work on an ESP32 and support clock-stretching.

The hardware I2C on an ESP32 is known for NOT supporting clock stretching.

While it is aimed and tested for the SCD30, it should work for other devices on the ESP32 as well.
it has been tested with BME280 and SPS30.

twi.c
