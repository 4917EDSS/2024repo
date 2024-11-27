// demo: CAN-BUS Shield, send data
// loovee@seeed.cc


#include <SPI.h>

#define CAN_2515

// Set SPI CS Pin according to your hardware
// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

int RGB_B = 7;


#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif

void setup() {
  pinMode(RGB_B, OUTPUT);
  //digitalWrite(RGB_B, HIGH);

  SERIAL_PORT_MONITOR.begin(115200);
  while (!Serial) {};

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
}

unsigned char stmp[8] = { 0, 0, 0, 5, 0, 0, 0, 0 };

void loop() {
  digitalWrite(RGB_B, HIGH);

  uint8_t CAN_ID = 0;

  // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf
  stmp[7] = stmp[7] + 1;
  if (stmp[7] == 100) {
    stmp[7] = 0;
    stmp[6] = stmp[6] + 1;

    if (stmp[6] == 100) {
      stmp[6] = 0;
      stmp[5] = stmp[5] + 1;
    }
  }
  //virtual byte sendMsgBuf(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true);

  unsigned char stmptmp[8];
  memcpy(stmptmp, stmp, 8);

  CAN.sendMsgBuf(CAN_ID, 0, sizeof(stmp), stmptmp);

  delay(100);  // send data per 100ms
  SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");
}

// END OF LINE
