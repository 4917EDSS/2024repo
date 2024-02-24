unsigned char version = 0x01;
unsigned char packetData[19] = {0};
unsigned int sensors[8] = {0,100, 200, 300, 400, 500, 600, 700};  // Sensor data is mocked up as offset by 100 counters
int dataLength = 16;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  packetData[1] = 0xA5;	// Header byte to know this is the start of the packet
  packetData[2] = version;
} 


void loop() {
  // put your main code here, to run repeatedly:
  unsigned char checksum = 0;
  int index = 3;

  // Wait until command is received from robotRIO
  // Serial.Read()?
  
  // Now send packet
  packetData[index++] = sensors[0] & 0xFF;
  packetData[index++] = (sensors[0] >> 8) & 0xFF;
  packetData[index++] = sensors[1] & 0xFF;
  packetData[index++] = (sensors[1] >> 8) & 0xFF;
  packetData[index++] = sensors[2] & 0xFF;
  packetData[index++] = (sensors[2] >> 8) & 0xFF;
  packetData[index++] = sensors[3] & 0xFF;
  packetData[index++] = (sensors[3] >> 8) & 0xFF;
  packetData[index++] = sensors[4] & 0xFF;
  packetData[index++] = (sensors[4] >> 8) & 0xFF;
  packetData[index++] = sensors[5] & 0xFF;
  packetData[index++] = (sensors[5] >> 8) & 0xFF;
  packetData[index++] = sensors[6] & 0xFF;
  packetData[index++] = (sensors[6] >> 8) & 0xFF;
  packetData[index++] = sensors[7] & 0xFF;
  packetData[index++] = (sensors[7] >> 8) & 0xFF;

  // Checksum the header, version and data bytes but not the checksum byte
  // Checksum is 8 bits.
  for(index = 0; index < dataLength + 2; index++) {
    checksum += packetData[index];
  }
  packetData[18] = checksum;

  // Increment fake data
  sensors[0]++;
  sensors[1]++;
  sensors[2]++;
  sensors[3]++;
  sensors[4]++;
  sensors[5]++;
  sensors[6]++;
  sensors[7]++;

  Serial.write(packetData, dataLength + 3);
}
