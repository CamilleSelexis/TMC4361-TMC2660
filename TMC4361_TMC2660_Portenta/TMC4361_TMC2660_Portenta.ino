/*
Works with the clk provided by the arduino
Microstepping does not seem to work properly
Could add a timer to make the steps
Works when the step/dir are controlled by the arduino
*/

#include <SPI.h>
#include "TMC4361A_Register.h"
#include <SoftSPIB.h>
#include "Arduino.h"
#include "PwmOut.h"
#include "pinDefinitions.h"
#include "PinNames.h"
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include "Portenta_H7_TimerInterrupt.h"

#define CS_HIGH delayMicroseconds(20);digitalWrite(CS_ENC,HIGH);delayMicroseconds(20);
#define CS_LOW delayMicroseconds(20);digitalWrite(CS_ENC,LOW);delayMicroseconds(20);
// Note: You also have to connect GND, 5V/VIO and VM.
//       A connection diagram can be found in the schematics.
#define EN_PIN    D6 //enable (CFG6)
#define DIR_PIN   D11 //direction
#define STEP_PIN  D12 //step
#define STALL_PIN 5 //STG_TST

#define CS_PIN   D7 //CS chip select
#define MOSI_PIN D8 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN D10 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  D9 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)
#define CLK16_PIN D1 //CLK16_PIN Timer pin //Timer 1

#define SW_MOSI D2
#define SW_MISO D4
#define SW_SCK D3
#define CS_ENC D5

SoftSPIB mySPI(SW_MOSI,SW_MISO,SW_SCK);
//MbedSPI mySPI(SW_MOSI,SW_MISO,SW_SCK);

#define WRITE_FLAG 0x80
bool step_state = false;

#define TIMER1_FREQ 16000000

Portenta_H7_Timer ITimer0(TIM15);
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
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(CLK16_PIN,OUTPUT);

  /*pinMode(CS_ENC,OUTPUT);digitalWrite(CS_ENC,HIGH);
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,OUTPUT);
  pinMode(SW_SCK,OUTPUT);*/

  pinMode(CS_ENC,INPUT);
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,INPUT);
  pinMode(SW_SCK,INPUT);
  
  //set up Timer1 for clk generation
  mbed::PwmOut* pwm = new mbed::PwmOut(digitalPinToPinName(D1));
  digitalPinToPwm(D1) = pwm;
  TIM1->PSC = 0;
  TIM1->ARR = 10; //Freq = F_CPU/(ARR*(PSC+1)) -> 200/10 = 20 MHz
  TIM1->CCR1 = 5; //Used to define the duty cycle D1 HIGH when CNT<CCR1 & D1 LOW when CNT>CCR1

  //init serial port
  Serial.begin(115200); //init serial port and set baudrate
  while(!Serial); //wait for serial port to connect (needed for Leonardo only)
  Serial.println("\nStart...");
  //init SPI encoder
  mySPI.begin();
  //mySPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE2));
  mySPI.setBitOrder(MSBFIRST);
  mySPI.setDataMode(SPI_MODE2);
  mySPI.setClockDivider(128);
  //mySPI.end();
  //init SPI driver
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE3));
  //init SPI encoder
  uint32_t SPI_OUT_CONF = 0x8440010B;
  writeReg(0x04,SPI_OUT_CONF); //SPI_OUT_CONF 844 -> SPI timing conf 10B -> 1us between poll TMC26x S/D output
  uint32_t GENERAL_CONF = 0x00006020;
  //writeReg(0x00,0x00006020); //Internal Step control Base value
  uint32_t STEP_CONF = 0x00FB0C080;
  writeReg(0x0A,0x00FB0C80); // 200 steps/rev 256 microsteps
  uint32_t CLK_FREQ = 0x00F42400;
  writeReg(0x31,CLK_FREQ); //16MHz external clock
  
  //Encoder setup
  uint32_t FILTER = 0x00000004; //1MHz sample rate
  uint32_t ENC_IN_CONF = 0x00000400 | 0x00010000; //internal multiturn
  writeReg(0x07,ENC_IN_CONF);
  uint32_t ENC_IN_RES = 0x00001000; //Encoder resolution at 4096;
  writeReg(0x54,ENC_IN_RES);
  uint32_t ENC_IN_DATA = 0x0008000F;//21B Angle data bits + 1 2 Status bits + 1+ 1 multiturn bits
  writeReg(0x08,ENC_IN_DATA);
  uint32_t ADDR_TO_ENC = 0x00000020; //Addr to obtain encoder data (0x20 angle 0x2c turn)
  writeReg(0x68,ADDR_TO_ENC);
  uint32_t SER_CLK_IN_HIGH = 0x0004;
  uint32_t SER_CLK_IN_LOW = 0x0004;
  writeReg(0x56,(SER_CLK_IN_LOW<<16)|SER_CLK_IN_HIGH);
  uint32_t SSI_IN_CLK_DELAY = 0x00F00080;
  writeReg(0x57,SSI_IN_CLK_DELAY);//8*16 clock delay between CS low & start of data transfer
  uint32_t SER_PTIME = 0x13880; //13880 5 ms delay between encoder call
  writeReg(0x58,SER_PTIME);

  //writeReg(0x50, 0x00000000);//Set encoder position to 255
  //readReg(0x6B);
  GENERAL_CONF = GENERAL_CONF | 0x00300C00; //Encoder in mode SPI must be done at the end
  writeReg(0x00,GENERAL_CONF);
  //readReg(0x6B);
  //TMC2660 config
  writeReg(0x6C,0x000901B4); //COVER_LOW CHOPCONF
  writeReg(0x6C,0x000D4107); //COVER_LOW SGSCONF CS D4107
  writeReg(0x6C,0x000E0010); //COVER_LOW DRVCONF SG & SPI interface
  writeReg(0x6C,0x00000000); //COVER_LOW DRVCTRL 256 microsteps
  writeReg(0x6C,0x000A8202); //COVER_LOW

  //Movement parameters
  uint32_t DIR_SETUP_TIME = 2;//#clock cycle step pulse wait after dir change
  uint32_t STP_LENGTH_ADD = 1;//#clock cycle step pulse is held
  writeReg(0x10,((DIR_SETUP_TIME << 16) | STP_LENGTH_ADD));
  uint32_t RAMPMODE = 0b101; //trapezoidal ramp positioning mode
  writeReg(0x20,RAMPMODE);
  uint32_t VMAX = 0x04FFFF00; //<8.338 Mpps at 32MHz last 8 bits are decimal
  writeReg(0x24,VMAX);
  uint32_t AMAX = 0x000FFFFF; //<4.194 Mpps
  writeReg(0x28,AMAX);
  uint32_t DMAX = 0x000FFFFF; //<4.194 Mpps
  writeReg(0x29,DMAX);
  uint32_t XACTUAL = 0;
  writeReg(0x21,XACTUAL);
  uint32_t XTARGET = 5120000;//100 full turns at 256 usteps
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
    data = readReg(0x21); //read Xactual //Return last data (Status flag)
    uint32_t Xactual = readReg(0x22); //read Vactual Return Xactual 
    uint32_t Vactual = readReg(0x37); //read Xtarget return Vactual
    uint32_t Xtarget = readReg(0x0F); //read Status flag register return Xtarget
    Serial.print("Actual pos = ");Serial.print(Xactual);Serial.print(" Actual speed = ");
    Serial.print(Vactual);Serial.print(" Actual target = ");Serial.println(Xtarget);
    //writeReg(0x6C,0x000901B0); //COVER_LOW CHOPCONF Disable the MOSFET
    /*if(Xactual == Xtarget){
      writeReg(0x6C,0x000901B0); //COVER_LOW CHOPCONF Disable the MOSFET
      //Serial.println("MOSFET disabled");
      writeReg(0x37,(Xtarget==256000)?0:256000);
    }*/
    readEncoder();
    readMultiturn();
  }
}
