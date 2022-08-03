void writeReg(uint8_t address, uint32_t command){
   
 delay(100);
 unsigned long i_datagram = 0;
 Serial.print("Sending command ");Serial.print(command,HEX);
 Serial.print(" at address :");Serial.println(address);
 
 digitalWrite(CS_PIN,LOW);
 delayMicroseconds(10);
 i_datagram = SPI.transfer(address|WRITE_FLAG);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((command >> 24) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((command >> 16) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((command >> 8) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((command>>0) & 0xff);
 //i_datagram <<= 8;
 digitalWrite(CS_PIN,HIGH);
 
 Serial.print("Received: ");
 Serial.println(i_datagram,HEX);
}
//To obtain the correct data the registers need to be read 2 times
//This can be optimized for multiple registers to only n+1 read
unsigned long readReg(uint8_t address){
   
 delay(100);
 unsigned long i_datagram = 0;
 
 digitalWrite(CS_PIN,LOW);
 delayMicroseconds(10);
 i_datagram |= SPI.transfer(address);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer(0x00);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer(0x00);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer(0x00);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer(0xff);
 //i_datagram <<= 8;
 digitalWrite(CS_PIN,HIGH);
 
 /*Serial.print("Received: ");
 Serial.println(i_datagram,HEX);*/
 return i_datagram;
}
