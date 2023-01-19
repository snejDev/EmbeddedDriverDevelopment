/* SPI Slave Demo

 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *
 
 */
#include <SPI.h>
#include<stdint.h>  
#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10

//Command Definitions
#define CMD_LED_CTRL      0x01
#define CMD_SENSOR_READ   0x02
#define CMD_LED_READ      0x03
#define CMD_PRINT         0x04
#define CMD_ID_READ       0x05

uint8_t dataBuff;

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  //make SPI as slave
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}


//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}
  

// The setup() function runs right after reset.
void setup() 
{
  // Initialize serial communication 
  Serial.begin(9600);
  // Initialize SPI Slave.
  SPI_SlaveInit();
  Serial.println("Slave Initialized");
}

uint8_t CmdVerify(uint8_t dataBuff)
{
  if((dataBuff==CMD_LED_CTRL)||(dataBuff==CMD_LED_READ)||(dataBuff==CMD_SENSOR_READ))
    return 0xF5;
  else 
    return 0xA5;
}

// The loop function runs continuously after setup().
void loop() 
{
  Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS));

  //Acknowledgement
  dataBuff = SPI_SlaveReceive();
  uint8_t ack = CmdVerify(dataBuff);
  SPI_SlaveTransmit(ack);
  uint8_t dummy = SPI_SlaveReceive();

  if(ack==0xF5)
  {
    switch(dataBuff) 
    {
      case CMD_LED_CTRL:
      { 
        uint8_t Dpin = (uint8_t)SPI_SlaveReceive();
        uint8_t value = (uint8_t)SPI_SlaveReceive();
        Serial.println(Dpin);
        Serial.println(value);
        pinMode(Dpin,OUTPUT);
        digitalWrite(Dpin,value);
        break;
      }

      case CMD_SENSOR_READ:
      {
        uint8_t Apin = (uint8_t)SPI_SlaveReceive();
        SPI_SlaveTransmit(analogRead(Apin+14));     //A0: 14
        uint8_t dummy = SPI_SlaveReceive();
        break;
      }

      case CMD_LED_READ:
      {
        uint8_t pin = (uint8_t)SPI_SlaveReceive();
        SPI_SlaveTransmit(digitalRead(pin));
        Serial.println("SensorRead");
        uint8_t dummy = SPI_SlaveReceive();
        break;
      }
      
      default: Serial.println("Invalid Command");
    }
  } 
}
