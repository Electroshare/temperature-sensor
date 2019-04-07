# Code

## sensor_temp_attiny

First version with parity generator and 8 sensors support

* 433MHz module
* ATtiny85 
* DS18B20 temperature sensor

16 bits data frame : 
[PARITY |15| ID_SENSOR |12| DECIMAL_TEMP |8| SIGN |7| INTEGER_TEMP]
[1 | 3 | 4 | 1 | 7]

## sensor_nrf24l01_attiny

Code for sensors. Up to 8 sensors with 1 main station for the moment. Low power consumption. **Need to be tested !**

* nRF24L01 module
* ATtiny85
* DS18B20 temperature sensor

## main_station_nrf24l01_nano

Code for the main station. Up to 8 sensors with 1 main station for the moment. **Need to be tested !**

* nRF24L01 module
* Arduino Nano
* I2C and Serial communication

## sensor_nrf24l01_attiny13a

Code for sensors, but smaller to fit inside an attiny13a.
Doesn't use the arduino IDE, but a makefile with avrdude.
**Still under developpment !**

* nRF24L01 module
* ATtiny13a
* DS18B20 temperature sensor
