/*
Works with the clk provided by the arduino
Microstepping does not seem to work properly
Could add a timer to make the steps
Works when the step/dir are controlled by the arduino
*/

#include <SPI.h>
#include "TMC4361A_Register.h"
#include <SoftSPIB.h>

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
#define CLK16_PIN D14 //CLK16_PIN 

#define SW_MOSI D0
#define SW_MISO D2
#define SW_SCK D1
#define CS_ENC D3

SoftSPIB mySPI(SW_MOSI,SW_MISO,SW_SCK);

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

  /*pinMode(CS_ENC,OUTPUT);digitalWrite(CS_ENC,HIGH);
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,OUTPUT);
  pinMode(SW_SCK,OUTPUT);*/

  pinMode(CS_ENC,INPUT);
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,INPUT);
  pinMode(SW_SCK,INPUT);
  
  //set up Timer1
  TCCR1A = bit (COM1A0); //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10); //CTC, no prescaling
  OCR1A = 0; //output every 1 cycle on pin 11 -> 16Mhz

  //set up Timer3 for sending step pulses
  /*{
    cli(); //disable interrupts
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0; //init counter value to 0
    OCR3A = 10; //Define freq of interrupt 16*10^6/1024 -1
    TCCR3B |= (1 << WGM12); // CTC mode
    TCCR3B |= (1 << CS11); //enable prescaler at 8
    TIMSK3 |= (1 << OCIE3A); //Timer compare interrupt
    sei(); //enable interrupts
  }*/
  //init serial port
  Serial.begin(115200); //init serial port and set baudrate
  while(!Serial); //wait for serial port to connect (needed for Leonardo only)
  Serial.println("\nStart...");
  //init softSPI
  mySPI.begin();
  mySPI.setBitOrder(MSBFIRST);
  mySPI.setDataMode(SPI_MODE2);
  mySPI.setClockDivider(SPI_CLOCK_DIV4);
  //init SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //SPI Mode 3
  SPI.setBitOrder(MSBFIRST); //MSB first
  SPI.setClockDivider(SPI_CLOCK_DIV8); //clk=Fcpu/128

  uint32_t SPI_OUT_CONF = 0x8440010B;
  writeReg(0x04,SPI_OUT_CONF); //SPI_OUT_CONF 844 -> SPI timing conf 10B -> 1us between poll TMC26x S/D output
  uint32_t GENERAL_CONF = 0x00006020;
  //writeReg(0x00,0x00006020); //Internal Step control Base value
  uint32_t STEP_CONF = 0x00FB0C080;
  writeReg(0x0A,0x00FB0C80); // 200 steps/rev 256 microsteps
  uint32_t CLK_FREQ = 0x00F424000;
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
  writeReg(0x6C,0x000D4107); //COVER_LOW SGSCONF CS
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

  if((ms-last_time) > 1000) //run every 2s
  {
    last_time = ms;
    data = readReg(0x21); //read Xactual //Return last data (Status flag)
    uint32_t Xactual = readReg(0x22); //read Vactual Return Xactual 
    uint32_t Vactual = readReg(0x37); //read Xtarget return Vactual
    uint32_t Xtarget = readReg(0x0F); //read Status flag register return Xtarget
    Serial.print("Actual pos = ");Serial.print(Xactual);Serial.print(" Actual speed = ");
    Serial.print(Vactual);Serial.print(" Actual target = ");Serial.println(Xtarget);
    /*if(Xactual == Xtarget){
      //writeReg(0x6C,0x000901B0); //COVER_LOW CHOPCONF Disable the MOSFET
      //Serial.println("MOSFET disabled");
      writeReg(0x37,(Xtarget==256000)?0:256000);
    }*/
    /*data = readReg(0x6B);
    data = readReg(0x51);*/
    data = readReg(0x6A); //read data from encoder
    uint32_t Eangle = readReg(0x50); //read computed position
    
    float angle_deg  = float((Eangle&0x0fff))*360/4096;//Should be 4096 but we miss the LSB
    //Serial.print("Status 0x20 = ");Serial.println(status_bits,BIN);
    Serial.print("angle = ");Serial.println(angle_deg);
    //data = readReg(0x51);
    //Serial.print("Encoder angle = 0x");Serial.println(Eangle,HEX);
    Serial.print("External angle = ");Serial.println(readReg(0x50));
    //readEncoder();
    Serial.println("----------------");
  }
}

void readEncoder(){
    digitalWrite(CS_ENC,LOW);
    delayMicroseconds(50);  
    //The timing seems to miss the last bit
    //MSB is thus garbage
    uint8_t cmd = 0x20;
    uint16_t angle = 0;
    angle = mySPI.transfer(cmd);//send streaming command
    angle <<= 8;
    angle |= mySPI.transfer(0x00);
    CS_HIGH;CS_LOW;
    //Read it again so it is updated
    angle = 0;
    angle = mySPI.transfer(cmd);//send streaming command
    angle <<= 8;
    angle |= mySPI.transfer(0x00);
    //angle = (angle<<1);//removes garbage MSB
    //Serial.print("angle = ");Serial.println(angle,HEX);
    uint8_t status_bits = (angle>>12);//take the 4 first bit 0/EF/UV/Parity
    /*if(status_bits >= 0x8){ //if MSB == 1 then invert MSB angle
      uint16_t mask = 0x0800;
      angle = angle^mask; //XOR
    }
    //Serial.print("angle after correction = ");Serial.println(angle,HEX);*/
    float angle_deg  = float((angle&0x0fff))*360/4096;//Should be 4096 but we miss the LSB
    //Serial.print("Status 0x20 = ");Serial.println(status_bits,BIN);
    Serial.print("angle = ");Serial.println(angle_deg);
    CS_HIGH;CS_LOW; 
    uint16_t turn = 0;
    cmd = 0x2C;
    turn = mySPI.transfer(cmd);//send streaming command
    turn <<= 8;
    turn |= mySPI.transfer(0x00);
    CS_HIGH;CS_LOW;
    turn = mySPI.transfer(cmd);//send streaming command
    turn <<= 8;
    turn |= mySPI.transfer(0x00);
    status_bits = 0;
    //turn = (turn<<1);//remove the garbage MSB
    status_bits = (turn>>12);//First 4 bits are status ID(110)/Parity
    int temp = (turn&0x0fff);
    float turn_count = float(temp)/8;
    CS_HIGH;
    //Serial.print("Status 0x2C = ");Serial.println(status_bits,BIN);
    Serial.print("turn = ");Serial.println(turn_count);
    Serial.println("---------");

  
}
