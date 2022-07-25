/*
Works with the clk provided by the arduino
Microstepping does not seem to work properly
Could add a timer to make the steps
Works when the step/dir are controlled by the arduino
*/

#include <SPI.h>

// Note: You also have to connect GND, 5V/VIO and VM.
//       A connection diagram can be found in the schematics.
#define EN_PIN    7 //enable (CFG6)
#define DIR_PIN   8 //direction
#define STEP_PIN  9 //step
#define STALL_PIN 5 //STG_TST

#define CS_PIN   10 //CS chip select
#define MOSI_PIN 51 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN 50 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  52 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)
#define CLK16_PIN 11 //CLK16_PIN 

//TMC2660 registers
#define REG_DRVCTRL    0x0
#define REG_CHOPCONF   0x08
#define REG_SMARTEN    0x0A
#define REG_SGCSCONF   0x0C
#define REG_DRVCONF    0x0E

//TMC4361 registers
#define REG_COVERLOW 0x6C
#define REG_SPI_OUTCONF 0x04
#define WRITE_FLAG 0x80

void setup()
{
  //set pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); //LOW or HIGH
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(STALL_PIN,INPUT);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  //digitalWrite(MOSI_PIN, LOW);
  pinMode(MISO_PIN, INPUT);
  //digitalWrite(MISO_PIN, LOW);
  pinMode(SCK_PIN, OUTPUT);
  //digitalWrite(SCK_PIN, LOW);
  pinMode(CLK16_PIN,OUTPUT);

  //set up Timer1
  TCCR1A = bit (COM1A0); //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10); //CTC, no prescaling
  OCR1A = 0; //output every 2 cycle on pin 11 -> 8Mhz
   
  //init serial port
  Serial.begin(115200); //init serial port and set baudrate
  while(!Serial); //wait for serial port to connect (needed for Leonardo only)
  Serial.println("\nStart...");
  
  //init SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //SPI Mode 3
  SPI.setBitOrder(MSBFIRST); //MSB first
  SPI.setClockDivider(SPI_CLOCK_DIV32); //clk=Fcpu/128
  //SPI.beginTransaction(SPISettings(1000000UL, MSBFIRST, SPI_MODE3));
  readReg(0x00);
  readReg(0x0A);
  readReg(0x0B);
  readReg(0x31);//Ext clk reg
  writeReg(0x04,0x4440010A); //SPI_OUT_CONF
  writeReg(0x6C,0x000901B4); //COVER_LOW CHOPCONF
  writeReg(0x6C,0x000D4109); //COVER_LOW SGSCONF CS
  writeReg(0x6C,0x000E0010); //COVER_LOW DRVCONF
  writeReg(0x6C,0x00000100); //COVER_LOW DRVCTRL 256 microsteps
  writeReg(0x6C,0x000A8202); //COVER_LOW

  //outputs on (LOW active)
  digitalWrite(EN_PIN, LOW);
  Serial.println("Setup end");
}

void loop()
{
  static uint32_t last_time=0;
  uint32_t ms = millis();
  uint32_t data;
  uint8_t s;

  if((ms-last_time) > 2000) //run every 2s
  {
    last_time = ms;
    readReg(0x00);
  }
  /*if(digitalRead(STALL_PIN)){
    Serial.println("Motor Stall");
  }*/
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(5);
}
