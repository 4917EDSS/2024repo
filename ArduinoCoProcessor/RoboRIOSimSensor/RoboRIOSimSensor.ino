#include <math.h>

#define BRIGHTNESS 25

#define RED BRIGHTNESS, 0, 0
#define GREEN 0, BRIGHTNESS, 0
#define BLUE 0, 0, BRIGHTNESS
#define YELLOW BRIGHTNESS, BRIGHTNESS, 0
#define MAGENTA BRIGHTNESS, 0, BRIGHTNESS
#define CYAN  0, BRIGHTNESS, BRIGHTNESS
#define OFF  0, 0, 0

#define COLOR_PACKET_SIZE 75

#define RECEIVE_SIZE 19
uint8_t receive_buffer[RECEIVE_SIZE];
bool pattern = false;

uint8_t row_brightness[6] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(38400);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  //digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  //delay(1000);                      // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(20);  
  
  uint8_t checksum = 0;
  uint8_t led_colors[COLOR_PACKET_SIZE] = {0xA5, 0x02,
                          OFF, OFF, OFF, OFF,
                          OFF, OFF, OFF, OFF,
                          OFF, OFF, OFF, OFF,
                          OFF, OFF, OFF, OFF,
                          OFF, OFF, OFF, OFF,
                          OFF, OFF, OFF, OFF,
                          0};


  for (int i=0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      led_colors[i*12 + j*3 + 2] = row_brightness[i];
    }
  }

  for (int i =0; i < COLOR_PACKET_SIZE; i++) {
    checksum += led_colors[i];
  }

  led_colors[COLOR_PACKET_SIZE-1]=checksum;
  
  Serial.write(led_colors, COLOR_PACKET_SIZE);



  // Receive data
  bool isPacketComplete = false;
  int index = 0;
  checksum = 0;
  int value;

  // loop while the packet is not complete
  while (isPacketComplete == false) {
    // Read from the serial port
    int value = Serial.read();

    // Was there any data received?
    if(value != -1) {
      
      // If we are on the start of the packet, is the start equal to the magic byte?
      if (index == 0) {
        if (value == 0xA5) {

          // If we are at the start of the array and the first value is A5 than we can recording data
          receive_buffer[index] = value;
          checksum += (uint8_t) value;
          index ++;
        } // If the value is not A5, continue looking for the magic byte 
      
      } else {
        // Once we have found the first byte, keep recording data 
        receive_buffer[index] = value;
        
        // Don't add recived checksum to the checksum
        if (index != (RECEIVE_SIZE - 1)) {
          checksum += (uint8_t) value;
        }
        
        index ++;
      }

      // If we are at the end of the packet 
      if (index == RECEIVE_SIZE) {
        // Don't packet any more
        isPacketComplete = true;
      }
    }
  }

  // If the checksum does not match, exit the loop and start over
  if (checksum != receive_buffer[RECEIVE_SIZE-1]) {
    //return;

    // Received data was invalid, toss result and try again.
  }


  int16_t SensorData[8];

  for (int i = 0; i < 8; i++) {
    // MSB/LSB might need to be swapped
    SensorData[i] = (receive_buffer[i*2 + 3] << 8) + receive_buffer[i*2 + 2];
  }

  int scale = 4;

  row_brightness[0] = (uint8_t) ((int16_t) min(SensorData[0] >> scale, 255));
  row_brightness[1] = (uint8_t) ((int16_t) min(SensorData[2] >> scale, 255));
  row_brightness[2] = (uint8_t) ((int16_t) min(SensorData[3] >> scale, 255));
  row_brightness[3] = (uint8_t) ((int16_t) min(SensorData[4] >> scale, 255));
  row_brightness[4] = (uint8_t) ((int16_t) min(SensorData[5] >> scale, 255));
  row_brightness[5] = (uint8_t) ((int16_t) min(SensorData[7] >> scale, 255));

  

}
