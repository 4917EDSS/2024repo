unsigned char packetData[8] = {0};
unsigned int sensors[2] = {0,100};
int dataLength = 4;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  //Serial.begin(9600);
  packetData[0] = 0xFF;
  packetData[1] = 0xA5;
  packetData[2] = dataLength + 1;

//  pinMode(BUTTON_PIN, INPUT);
} 


void loop() {
  // put your main code here, to run repeatedly:
  unsigned char checksum = 0;
  int index = 0;

  packetData[3] = sensors[0] & 0xFF;
  packetData[4] = (sensors[0] >> 8) & 0xFF;
  packetData[5] = sensors[1] & 0xFF;
  packetData[6] = (sensors[1] >> 8) & 0xFF;

  // Checksum the header, length and data bytes but not the checksum byte
  // Checksum is 8 bits.
  for(index = 3; index < dataLength + 3; index++) {
    checksum += packetData[index];
  }
  packetData[7] = checksum;
  //packetData[7] = 0x00;

  // Increment fake data
  sensors[0]++;
  sensors[1]++;

  //unsigned char myBite = 0xA5;
  
  //while(digitalRead(BUTTON_PIN) == HIGH);
   Serial.write(packetData, dataLength + 4);
  //Serial.write(&packetData[0],1); //1010110101
  //Serial.write(&packetData[1],1); //1010110101
  //Serial.write(&packetData[2],1); //1010110101
  //delay(1); //would like 260 micro sec  0.000260
  //while(digitalRead(BUTTON_PIN) == LOW);
}
