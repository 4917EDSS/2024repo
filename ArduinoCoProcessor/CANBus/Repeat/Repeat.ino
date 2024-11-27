#include <SPI.h>

#define CAN_2515

// Set SPI CS Pin according to your hardware
// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

int LED = 7;
boolean ledON = 1;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif

void setup() {
  SERIAL_PORT_MONITOR.begin(115200);
  pinMode(LED, OUTPUT);

  SERIAL_PORT_MONITOR.println("RECEIVE PROGRAM STARTED");

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
}


void loop() {
  //SERIAL_PORT_MONITOR.println("help");

  unsigned char len = 0;
  unsigned char buf[8];

  //SERIAL_PORT_MONITOR.println("checking");
  //SERIAL_PORT_MONITOR.println(CAN_MSGAVAIL);

  unsigned char readValue = CAN.readMsgBuf(&len, buf);

  SERIAL_PORT_MONITOR.println(readValue);

  SERIAL_PORT_MONITOR.print("Sending: ");
  // ID, unknown, Size of packet, data
  SERIAL_PORT_MONITOR.println(CAN.sendMsgBuf(456, 0, sizeof(readValue), readValue));

  /*
  if (CAN_MSGAVAIL == CAN.checkReceive()) {  // check if data coming
    CAN.readMsgBuf(&len, buf);               // read data,  len: data length, buf: data buf


    unsigned long canId = CAN.getCanId();


    SERIAL_PORT_MONITOR.println("-----------------------------");
    SERIAL_PORT_MONITOR.println("get data from ID: 0x");
    SERIAL_PORT_MONITOR.println(canId, HEX);

    for (int i = 0; i < len; i++) {  // print the data
      SERIAL_PORT_MONITOR.print(buf[i]);
      SERIAL_PORT_MONITOR.print("\t");
      if (ledON && i == 0) {

        digitalWrite(LED, buf[i]);
        ledON = 0;
        delay(500);
      } else if ((!(ledON)) && i == 4) {

        digitalWrite(LED, buf[i]);
        ledON = 1;
      }
    }
    SERIAL_PORT_MONITOR.println();
  }
  */
}

//END FILE
