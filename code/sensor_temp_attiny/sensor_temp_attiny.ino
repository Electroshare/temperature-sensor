/*
 * Temperature sensor
 * CREATION : 30-12-2017
 * UPDATE : 01-01-2018
 * rev : 1.2
 * Louis Barbier
 */
/*
TRAME : [PARITE | ID_SONDE | DECIMALE_TEMP | SIGNE | ENTIER_TEMP]
        16      15         12              8       7            0
*/
#include <Manchester.h>
#include "OneWire.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define RF_DATA 1
#define RF_ALIM 0
#define SENSE_ALIM 2
#define SENSE_TEMP 4
#define LED_PIN 3
#define SENSOR_NUM 0b001 // 8 sensors maximum
#define RESOLUTION_TEMP 12

// DS18B20 
const int ReadROM = 0x33;
const int MatchROM = 0x55;
const int SkipROM = 0xCC;
const int ConvertT = 0x44;
const int ReadScratchpad = 0xBE;

OneWire ow = OneWire(SENSE_TEMP);

float temp = 0.0;
uint8_t tInt = 0;
uint8_t tDec = 0;

volatile int wdt_count = 0;

int generateParity(int data);

void start() {
  ADCSRA &= ~_BV(ADEN);  // switch ADC OFF
  // Configure attiny85 sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  DDRB = (1 << RF_ALIM) | (0 << RF_DATA) | (0 << SENSE_ALIM); //| (1 << LED_PIN);
  //PORTB &= ~(1 << RF_ALIM);
  man.setupTransmit(RF_DATA, MAN_300); // Rx pin, speed
  PORTB = 0;//PORTB = (1 << RF_DATA);
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
    PORTB |= 1 << RF_ALIM;
    //PORTB |= 1 << LED_PIN;
    readTemperature();
    processTemp();

    uint16_t data = (SENSOR_NUM << 4) | tDec;
    data = (data << 8) | tInt;
    data = generateParity(data);

    _delay_ms(5);

    man.transmit(data);
    
    _delay_ms(125);

    man.transmit(data);

    PORTB &= ~(1 << RF_ALIM);
    //PORTB &= ~(1 << LED_PIN);
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
