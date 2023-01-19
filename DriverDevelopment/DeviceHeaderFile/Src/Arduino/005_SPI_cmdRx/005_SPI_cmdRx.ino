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

  Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS));

  SPI_SlaveTransmit((uint8_t)0xF5);
  uint8_t dummy = SPI_SlaveReceive();
  dataBuff = SPI_SlaveReceive();
  Serial.println(dummy);
  Serial.println(dataBuff);
}

uint8_t CmdVerify(uint8_t dataBuff)
{
  if(dataBuff==0x01)
    return 0xF5;
  else 
    return 0xA5;
}

// The loop function runs continuously after setup().
void loop() 
{

}
