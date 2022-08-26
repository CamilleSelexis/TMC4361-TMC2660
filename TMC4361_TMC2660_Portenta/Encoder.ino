void readEncoder(){
  uint32_t Eangle = readReg(0x6A); //read data from encoder
  Eangle = readReg(0x50); //read Encoder data from the TMC4361A registers given in usteps
  uint32_t ETMCangle = readReg(0x50);
  float angle_deg  = float((Eangle&0x0fff))*360/4096;//Should be 4096 but we miss the LSB
  Serial.print("angle real = ");Serial.println(angle_deg);
  //data = readReg(0x51);
  //Serial.print("Encoder angle = 0x");Serial.println(Eangle,HEX);
  Serial.print("TMC4361A internal encoder angle = ");Serial.println(ETMCangle);
  Serial.println("----------------");
}

void readMultiturn(){
  uint32_t ADDR_TO_ENC = 0x0000002C; //Put the encoder in a config where it reads the multiturn data
  writeReg(0x68,ADDR_TO_ENC);
  delay(50);
  uint32_t data = readReg(0x6A); //Read answer of last call
  uint32_t Eturn = readReg(0x6A); //read actual encoder data
  float multiturnData = float((Eturn&0x0fff))/8;
  Serial.print("Multiturn data = ");Serial.println(multiturnData);
  Serial.println("----------------");
  ADDR_TO_ENC = 0x00000020; //Return the encoder to its angle config
  writeReg(0x68,ADDR_TO_ENC);
  delay(50);
}

void readEncoder2(){
    enablemySPI();
    mySPI.setBitOrder(MSBFIRST);
    mySPI.setDataMode(SPI_MODE2);
    mySPI.setClockDivider(128);
    
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
    if(status_bits >= 0x8){ //if MSB == 1 then invert MSB angle
      uint16_t mask = 0x0800;
      angle = angle^mask; //XOR
    }
    //Serial.print("angle after correction = ");Serial.println(angle,HEX);
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
    disablemySPI();
  
}

void enablemySPI(){
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,OUTPUT);
  pinMode(SW_SCK,OUTPUT);
  pinMode(CS_ENC,OUTPUT);digitalWrite(CS_ENC,HIGH);
  mySPI.begin();
}
void disablemySPI(){
  //Put all inputs floating
  pinMode(SW_MISO,INPUT);
  pinMode(SW_MOSI,INPUT);
  pinMode(SW_SCK,INPUT);
  pinMode(CS_ENC,INPUT);
  mySPI.end();
}
