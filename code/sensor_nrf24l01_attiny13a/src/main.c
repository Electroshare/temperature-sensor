/*
* Simple talking to nrf24L01 using attiny13a
* written by: Pierre Cadart
* 14/10/2018
*
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define NRF_CONFIG     0x00
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH      0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS     0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_TX_ADDR    0x10
#define NRF_RX_PW_P0   0x11
#define NRF_DYNPD      0x1C
#define NRF_FEATURE    0x1D

#define W_REGISTER     0x20
#define W_TX_PAYLOAD   0xA0
#define FLUSH_TX       0xE1
#define FLUSH_RX       0xE2
#define ACTIVATE       0x50

#define PIN_MOSI   0
#define PIN_MISO   1
#define PIN_SCK    2
#define PIN_CSN    3
#define PIN_DEBUG  4

static const char ADDRESS[6] = "00001";

volatile uint16_t cnt=0;
volatile uint8_t needSend=0;

inline void setCSN(void);
inline void unsetCSN(void);
void wreg(uint8_t val, uint8_t add);
void wSPIBuff(uint8_t *buff, uint8_t size);
void wSPI(uint8_t val);

int main(void)
{
  // Setup the clock
  cli();			//Disable global interrupts
  TCCR0B |= 1<<CS00 | 1<<CS01;	//Divide by 64
  OCR0B = 1;			//Count 100 cycles for 1/10 second interrupt
  TCCR0B |= 1<<WGM01;		//Put Timer/Counter1 in CTC mode
  TIMSK0 |= 1<<OCIE0B;		//enable timer compare interrupt
  sei();			//Enable global interrupts

  // Setup pins
  DDRB |= (1<<PIN_MOSI);	//Set PortB Pin0 as an output (MOSI)
  //DDRB &= (0<<1);		//Set PortB Pin1 as an input  (MISO)
  DDRB |= (1<<PIN_SCK);		//Set PortB Pin2 as an output (SCK)
  DDRB |= (1<<PIN_CSN);		//Set PortB Pin3 as an output (CSN)

  DDRB |= (1<<PIN_DEBUG);	//Set PortB Pin4 as an output (debug)
  
  // Wait 100ms for configuration
  while(!needSend){}
  needSend = 0;

  // Configure nRF24L01
  wreg(0x7C, NRF_CONFIG);       // Mask interrupts, enable 2 byte CRC, power-down
  wreg(0x5F, NRF_SETUP_RETR);   // retry 1500µs, 15 times
  wreg(0x07, NRF_RF_SETUP);     // 1Mbps, max power
  // activate features (shockburst)
  setCSN();
  wSPI(ACTIVATE);
  wSPI(0x73);
  unsetCSN();
  // configure features
  /*
  wreg(0x04, NRF_FEATURE);      // Dynamic payload length
  wreg(0x3F, NRF_DYNPD);        // Dynamic payload for every pipe
  */
  wreg(0x00, NRF_FEATURE);      // Dynamic payload length
  wreg(0x00, NRF_DYNPD);        // Dynamic payload for every pipe

  wreg(0x4C, NRF_RF_CH);        // Use channel 76
  // clear flags
  wreg(0x70, NRF_STATUS);        // Clear status interrupts
  // flush tx and rx
  setCSN();
  wSPI(FLUSH_TX);
  unsetCSN();
  setCSN();
  wSPI(FLUSH_RX);
  unsetCSN();
  // switch on
  wreg(0x7E, NRF_CONFIG);       // Mask interrupts, enable 2 byte CRC, power-up
  // wait 100ms for power-up
  while(!needSend){}
  needSend = 0;

  // Set timer compare back to 10 -> 1s / cycle
  OCR0B = 10;
  // receive pipe 0 address
  setCSN();
  wSPI(W_REGISTER|NRF_RX_ADDR_P0);
  wSPIBuff(ADDRESS, 5);
  unsetCSN();
  // sending pipe address
  setCSN();
  wSPI(W_REGISTER|NRF_TX_ADDR);
  wSPIBuff(ADDRESS, 5);
  unsetCSN();
  // packet size = 32
  wreg(0x20, NRF_RX_PW_P0);
  // enable p0 rx (for tx...)
  wreg(0x01, NRF_EN_RXADDR);

  // reset needSend, just in case
  needSend = 0;
  
  // Main loop
  while(1) {
    if(needSend){
      needSend = 0;
      setCSN();
      wSPI(W_TX_PAYLOAD);
      wSPI((cnt>>8) & 0xff);
      wSPI((cnt>>0) & 0xff);
      for(char i=0; i<2; i++)
        wSPI(0);
      unsetCSN();
      PORTB &= ~(1<<PIN_DEBUG);
    }
  }
}

// Set the CSN pin low to begin a transmission
inline void setCSN(void){
  PORTB &= ~(1<<PIN_CSN);
}

// Set the CSN pin high to finish a transmission
inline void unsetCSN(void){
  PORTB |= (1<<PIN_CSN);
}

// Write an entire buffer to the SPI
void wSPIBuff(uint8_t *buff, uint8_t size) {
  while(--size)
    wSPI(*buff++);
}

// Write a single byte to the SPI
void wSPI(uint8_t val) {
  for(uint8_t i=0; i<8; i++){
    PORTB |= (1<<PIN_SCK);
    if(((1<<7) & val) != 0) {
      PORTB |= (1<<PIN_MOSI);
    }else{
      PORTB &= ~(1<<PIN_MOSI);
    }
    PORTB &= ~(1<<PIN_SCK);
    val <<= 1;
  }
}

// Write a register value 
void wreg(uint8_t val, uint8_t add){
  setCSN();
  wSPI(W_REGISTER | add);
  wSPI(val);
  unsetCSN();
}

// Interrupt Service Routine
ISR(TIM0_COMPB_vect)
{
  cnt += 1;
  if(cnt%128 == 0){
    PORTB |= (1<<PIN_DEBUG);
    needSend=1;
    //PORTB |= (1<<PIN_DEBUG);
  }
}
