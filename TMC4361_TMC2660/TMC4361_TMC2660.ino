/*
Works with the clk provided by the arduino
Microstepping does not seem to work properly
Could add a timer to make the steps
Works when the step/dir are controlled by the arduino
*/

#include <SPI.h>
#include "TMC4361A_Register.h"
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
#define ADR_COVERLOW 0x6C
#define ADR_SPIOUTCONF 0x04
#define WRITE_FLAG 0x80
bool step_state = false;
ISR(TIMER3_COMPA_vect){
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  step_state = !step_state;
  digitalWrite(STEP_PIN, step_state);
}
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
  OCR1A = 0; //output every 1 cycle on pin 11 -> 16Mhz

  //set up Timer3 for sending step pulses
  {
    cli(); //disable interrupts
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0; //init counter value to 0
    OCR3A = 10; //Define freq of interrupt 16*10^6/1024 -1
    TCCR3B |= (1 << WGM12); // CTC mode
    TCCR3B |= (1 << CS11); //enable prescaler at 8
    TIMSK3 |= (1 << OCIE3A); //Timer compare interrupt
    sei(); //enable interrupts
  }
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
  uint32_t DIR_SETUP_TIME = 10;//#clock cycle step pulse wait after dir change
  uint32_t STP_LENGTH_ADD = 25;//#clock cycle step pulse is held
  writeReg(0x10,(DIR_SETUP_TIME << 16 | STP_LENGTH_ADD));
  uint32_t RAMPMODE = 0b100; //No ramp positioning mode
  writeReg(0x20,RAMPMODE);
  uint32_t VMAX = 5000; //<16777215
  writeReg(0x24,VMAX);
  uint32_t XACTUAL = 0;
  writeReg(0x21,XACTUAL);
  uint32_t XTARGET = 1000;//5 full turns
  writeReg(0x37,XTARGET);
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
    readReg(0x21); //read Xactual
  }

  /*if(digitalRead(STALL_PIN)){
    Serial.println("Motor Stall");
  }*/
  //make steps
//  digitalWrite(STEP_PIN, HIGH);
//  delayMicroseconds(5);
//  digitalWrite(STEP_PIN, LOW);
//  delayMicroseconds(5);
}
