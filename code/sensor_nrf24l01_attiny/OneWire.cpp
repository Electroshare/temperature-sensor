// One Wire Protocol **********************************************
#include "OneWire.h"

OneWire::OneWire(int OneWirePin) {
  OneWire::OneWirePin = OneWirePin;
  OneWire::Setup();
}

inline void OneWire::PinLow () {
  DDRB = DDRB | 1<<OneWirePin;
}

inline void OneWire::PinRelease () {
  DDRB = DDRB & ~(1<<OneWirePin);
}

// Returns 0 or 1
inline uint8_t OneWire::PinRead () {
  return PINB>>OneWirePin & 1;
}

void OneWire::DelayMicros (int micro) {
  TCNT1 = 0; TIFR = 1<<OCF1A;
  OCR1A = (micro>>1) - 1;
  while ((TIFR & 1<<OCF1A) == 0);
}

void OneWire::LowRelease (int low, int high) {
  PinLow();
  DelayMicros(low);
  PinRelease();
  DelayMicros(high);
}

uint8_t OneWire::Setup () {
  TCCR1 = 0<<CTC1 | 0<<PWM1A | 5<<CS10;  // CTC mode, 500kHz clock
  GTCCR = 0<<PWM1B;
 }

uint8_t OneWire::Reset () {
  uint8_t data = 1;
  LowRelease(480, 70);
  data = PinRead();
  DelayMicros(410);
  return data;                         // 0 = device present
}

void OneWire::Write (uint8_t data) {
  int del;
  for (int i = 0; i<8; i++) {
	if ((data & 1) == 1) del = 6; else del = 60;
	LowRelease(del, 70 - del);
	data = data >> 1;
  }
}

uint8_t OneWire::Read () {
  uint8_t data = 0;
  for (int i = 0; i<8; i++) {
	LowRelease(6, 9);
	data = data | PinRead()<<i;
	DelayMicros(55);
  }
  return data;
}

// Read bytes into array, least significant byte first
void OneWire::ReadBytes (int bytes) {
  for (int i=0; i<bytes; i++) {
	un.DataBytes[i] = OneWire::Read();
  }
}

// Calculate CRC over buffer - 0x00 is correct
uint8_t OneWire::CRC (int bytes) {
  uint8_t crc = 0;
  for (int j=0; j<bytes; j++) {
	crc = crc ^ un.DataBytes[j];
	for (int i=0; i<8; i++) crc = crc>>1 ^ ((crc & 1) ? 0x8c : 0);
  }
  return crc;
}

unsigned int OneWire::getTempRaw() {
  return un.DataWords[0];  
 }

