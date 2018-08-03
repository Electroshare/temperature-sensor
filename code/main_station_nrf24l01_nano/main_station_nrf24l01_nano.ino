/*
 * Microcontroleur de liaison entre les capteurs de températures (433MHz) et le rasberry pi (I2C)
 * CREATION : 03-08-2018
 * MAJ : 03-08-2018
 * rev : 1.1
 */
/*
Each data sensor is coded on 2 bytes. 
I2C implemented
Serial implemented
Watchdog not implemented yet

TRAME : [PARITÉ | ID_SONDE | DECIMALE_TEMP | SIGNE | ENTIER_TEMP]
        16      15         12              8       7            0
*/
#include <Wire.h>
#include <RF24.h>
//#include <avr/wdt.h>

#define SL_ADR_I2C 0x10

// nRF24L01
#define CE_PIN 9
#define CSN_PIN 10
#define RF_CHANNEL 96
uint8_t addresses[][6] = {"main","sense"};

#define NB_SENSOR 6 
#define NB_BYTE_SENSOR 2 // 2 bytes by sensor
#define NB_TAB NB_SENSOR*NB_BYTE_SENSOR
#define TIME_POOL_VALIDITY 10000 // en millisecondes (10 secondes)
#define TIME_VALIDITY 300000 // en millisecondes (5 min)

#define RESOLUTION_TEMP 12

void receiveData(uint8_t nbBytes);
void sendData();
void recupTempInt();
float processTemp(uint16_t buf);
uint8_t processID(uint16_t buf);
bool checkParity(uint16_t buf);

byte registre = 0; // id data to send
byte length = 0;

uint8_t tempTab[NB_TAB+1] = {0};
unsigned long time_actual = 0, time_prev = 0, time_sensor[NB_SENSOR] = {0};

/*void wdtEnable(void)
{
    
}*/

RF24 radio(CE_PIN, CSN_PIN);

void setup() 
{            
  Wire.begin(SL_ADR_I2C);

  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.begin(9600);

  // nRF24L01
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
  radio.startListening();

  //wdtEnable();
}

void loop() 
{
  if(millis() > time_actual) {
    time_actual = millis();
    //wdt_reset();
  }
  else {
    time_actual = 0;
    time_prev = 0;
    for (int i = 0; i < NB_SENSOR; i++) {
      time_sensor[i] = 0;
    }
  }
  
  if(radio.available()) 
  {
    uint16_t buf;
    radio.read(&buf, sizeof(uint16_t));
    if(checkParity(buf)) {
      uint8_t idSensor = processID(buf);
      float temp = processTemp(buf);
      if(temp >= -85 && temp <= 85 && idSensor < NB_SENSOR && idSensor >= 0) {
        tempTab[idSensor*NB_BYTE_SENSOR] = (uint8_t)(buf & 0x00FF);
        tempTab[(idSensor*NB_BYTE_SENSOR)+1] = (uint8_t)((buf >> 8) & 0x00FF);

        // Validity sensor
        time_sensor[idSensor] = time_actual;
        tempTab[NB_TAB] |= (1 << idSensor); // 1 << idSensor;
      }
    }
  }

  if(Serial.available() > 0) {
    uint8_t idSensorToSend = Serial.read();
    Serial.write(tempTab[idSensorToSend]);
    while(Serial.available() > 0)
      Serial.read();
  }
  
  if (time_actual - time_prev > TIME_POOL_VALIDITY)
  {
    for (int i = 0; i < NB_SENSOR; i++)
    {
      if(time_actual - time_sensor[i] > TIME_VALIDITY)
        tempTab[NB_TAB] &= ~(1 << i);
    }
    time_prev = time_actual;
  }
}

void receiveData(uint8_t nbBytes)
{
  switch(nbBytes) {
    case 1:
      registre = Wire.read();
      length = 1;
      if(registre >= NB_TAB+16)
        registre = 0; // id data à envoyer
      break;
    case 2:
      registre = Wire.read();
      length = Wire.read();
      if(registre >= NB_TAB+16)
        registre = 0; // id data à envoyer
      if(length+registre > NB_TAB+16)
        length = 1;
      break;

    default:
      return;
      break;
  }
}

void sendData()
{
  for (uint8_t i = 0; i < length; i++)
  {
    Wire.write(tempTab[registre+i]);
  }
}

float processTemp(uint16_t buf) {
  int8_t tInt = buf & 0x00FF;
  uint8_t tDec = (buf >> 8) & 0x000F;
  float temp = tInt;
  temp += ((tDec>>3) & 0x01)*1.0/2 + ((tDec>>2) & 0x01)*1.0/4 + ((tDec>>1) & 0x01)*1.0/8 + ((tDec>>0) & 0x01)*1.0/16;
  return temp;
}

uint8_t processID(uint16_t buf) {
  return ((buf >> 12) & 0x07);
}

bool checkParity(uint16_t buf) {
  int nbOnes = 0;
  for(int i = 0; i < 16; i++) {
    if((buf >> i) & 0x0001)
      nbOnes++;
  }
  if(nbOnes % 2)
    return false;
  else
    return true;
}
