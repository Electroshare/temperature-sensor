
// One Wire Protocol **********************************************

#ifndef ONEWIRE_h
#define ONEWIRE_h

#include "Arduino.h"

class OneWire
{
public:

	OneWire(int OneWirePin);
	
	inline void PinLow ();

	inline void PinRelease ();

	// Returns 0 or 1
	inline uint8_t PinRead ();

	void DelayMicros (int micro);

	void LowRelease (int low, int high);

	uint8_t Setup ();

	uint8_t Reset ();

	void Write (uint8_t data);

	uint8_t Read ();
	// Read bytes into array, least significant byte first
	void ReadBytes (int bytes);

	// Calculate CRC over buffer - 0x00 is correct
	uint8_t CRC (int bytes);

  unsigned int getTempRaw();

	int OneWirePin;

	
	// Buffer to read data or ROM code
	union {
	  uint8_t DataBytes[9];
	  unsigned int DataWords[4];
	}un;

};

#endif
