/*
 * Temperature sensor
 * CREATION : 03-08-2018
 * UPDATE : 03-08-2018
 * rev : 1.0
 * Louis Barbier
 *
 * This file is part of Temperature-sensor.
 *
 * Temperature-sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.

 * Temperature-sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Temperature-sensor.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


/*
                                                                                            ^^
                                 +-\/-+           nRF24L01   CE, pin3 ------|              //
                           PB5  1|o   |8  Vcc --- nRF24L01  VCC, pin2 ------x----------x--|<|-- 5V
                   NC      PB3  2|    |7  PB2 --- nRF24L01  SCK, pin5 --|<|---x-[22k]--|  LED
               DS18B20 --- PB4  3|    |6  PB1 --- nRF24L01 MOSI, pin6  1n4148 |
    nRF24L01 GND, pin1 -x- GND  4|    |5  PB0 --- nRF24L01 MISO, pin7         |
                        |        +----+                                       |
                        |-----------------------------------------------||----x-- nRF24L01 CSN, pin4 
                                                                       10nF
*/

#include "RF24.h"
#include "OneWire.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

// General
#define SENSE_ALIM 2
#define SENSOR_NUM 0b001 // 8 sensors maximum

// nRF24L01
#define RF_ALIM 3
#define RF_CHANNEL 96
#define CE_PIN 3
#define CSN_PIN 3
uint8_t addresses[][6] = {"sense","main"};

// DS18B20 
#define SENSE_TEMP 4
#define RESOLUTION_TEMP 12
const int ReadROM = 0x33;
const int MatchROM = 0x55;
const int SkipROM = 0xCC;
const int ConvertT = 0x44;
const int ReadScratchpad = 0xBE;

OneWire ow = OneWire(SENSE_TEMP);
RF24 radio(CE_PIN, CSN_PIN);

float temp = 0.0;
uint8_t tInt = 0;
uint8_t tDec = 0;

volatile int wdt_count = 0;

int generateParity(int data);

void start() {
  ADCSRA &= ~_BV(ADEN);  // switch ADC OFF
  // Configure attiny85 sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  //DDRB = (1 << RF_ALIM) | (0 << SENSE_ALIM);
  //PORTB = 1 << RF_ALIM;

  radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.setAutoAck(1);
  radio.setRetries(15,5); // 5 retries, (15+1)*250us=4000us between each retry
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.setPayloadSize(5); // 5 bytes payload
  radio.setPALevel(RF24_PA_MAX);
  if(!radio.setDataRate(RF24_250KBPS))
  	radio.setDataRate(RF24_1MBPS);
}

bool readTemperature () {
  int retry = 3;
  bool error = false;
  do {
    retry--;
    error = false;
    cli();                            // No interrupts
    if (ow.Reset() != 0) {
      sei();
      error = true;
    } 
    else {
      ow.Write(SkipROM);
      ow.Write(ConvertT);
      while (ow.Read() != 0xFF);
      ow.Reset();
      ow.Write(SkipROM);
      ow.Write(ReadScratchpad);
      ow.ReadBytes(9);
      sei();                          // Interrupts
      if (ow.CRC(9) != 0) //ow.DataWords[0]
        error = true;
    }
  } while(retry != 0 && error);

  return error;
}

void processTemp() {
  tInt = ow.un.DataWords[0]>>4;
  tDec = ow.un.DataWords[0] & 0x000F;
/*  int8_t tInt = buf & 0x00FF;
  uint8_t tDec = (buf >> 8) & 0x000F;
  float temp = tInt;
  temp += ((tDec>>3) & 0x01)*1.0/2 + ((tDec>>2) & 0x01)*1.0/4 + ((tDec>>1) & 0x01)*1.0/8 + ((tDec>>0) & 0x01)*1.0/16;*/
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  wdt_count++;
  WDTCR |= _BV(WDIE);           // Watchdog goes to interrupt not reset
}

void system_sleep(uint8_t duration) {
  ACSR |= _BV(ACD); // switch Analog Comparator OFF
  wdt_enable(WDTO_8S);
  // set wdt to generate interrupt instead of reset (see p47)
  WDTCR |= _BV(WDIE);
   
  wdt_count = 0;
  sei();
  while(wdt_count < duration){       
    sleep_mode();                   // Make CPU sleep until next WDT interrupt
  }

  wdt_disable ();
  ACSR &= ~ _BV(ACD); // switch Analog Comparator ON
  // wait for stabilization of internale voltage ref (See p44)
  //_delay_ms(1);
}

int main() 
{
  start();
  
  while(1) {
    //PORTB |= 1 << RF_ALIM;
    radio.powerUp();
    readTemperature();
    processTemp();

    uint16_t data = (SENSOR_NUM << 4) | tDec;
    data = (data << 8) | tInt;
    data = generateParity(data);
	
	radio.stopListening();
    radio.write(&data, sizeof(uint16_t));

    radio.powerDown();
    //PORTB &= ~(1 << RF_ALIM);
    system_sleep(10);
  } 
}

int generateParity(int data) {
  int nbOnes = 0;
  for(int i = 0; i < 16; i++) {
    if((data >> i) & 0x0001)
      nbOnes++;
  }
  if(nbOnes%2)
    data = data | (1 << 15);

  return data;
}
