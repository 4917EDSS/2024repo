unsigned char packetData[7] = {0};
unsigned int sensors[2] = {0,100};
int dataLength = 4;

#define BUTTON_PIN 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  packetData[0] = 0xA5;
  packetData[1] = dataLength + 1;

  pinMode(BUTTON_PIN, INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  unsigned char checksum = 0;
  int index = 0;

  packetData[2] = sensors[0] & 0xFF;
  packetData[3] = (sensors[0] >> 8) & 0xFF;
  packetData[4] = sensors[1] & 0xFF;
  packetData[5] = (sensors[1] >> 8) & 0xFF;

  // Checksum the header, length and data bytes but not the checksum byte
  for(index = 0; index < dataLength + 2; index++) {
    checksum += packetData[index];
  }
  packetData[6] = checksum;

  // Increment fake data
  sensors[0]++;
  sensors[1]++;

  //while(digitalRead(BUTTON_PIN) == HIGH);
  Serial.write(packetData, dataLength + 3);
  //while(digitalRead(BUTTON_PIN) == LOW);
}
