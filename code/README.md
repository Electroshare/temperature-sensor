# Code

## sensor_temp_attiny

First version with parity generator and 8 sensors support

* 433MHz module
* ATtiny 85 
* DS18B20 temperature sensor

16 bits data frame : 
[PARITY |15| ID_SENSOR |12| DECIMAL_TEMP |8| SIGN |7| INTEGER_TEMP]
[1 | 3 | 4 | 1 | 7]
