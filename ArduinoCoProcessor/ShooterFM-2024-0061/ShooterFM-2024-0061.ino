// Import neopixel library for LEDs
#include <Adafruit_NeoPixel.h>

// Define adafruit neopixel stuff
#define PIN 2
#define NUMPIXELS 24
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

#define PACKET_SIZE 19
#define COLOUR_SIZE 75

unsigned char version = 0x02;
unsigned char packetData[PACKET_SIZE] = {0};
unsigned int sensors[8] = {0,100, 200, 300, 400, 500, 600, 700};  // Sensor data is mocked up as offset by 100 counters

// Create a varible for elapsed time
long int timeSample = millis();

// Create an array for recieving data
uint8_t colourData[COLOUR_SIZE];


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  //Serial.begin(9600);
  packetData[0] = 0xA5;	// Header byte to know this is the start of the packet
  packetData[1] = version;

  pinMode(LED_BUILTIN, OUTPUT);

  pixels.begin();
  
  // Set all the leds to green
  for (int i = 0; i < 24; i++) {
    setLeds(i, 0, 255, 0);
  }

  // Enable the pixels
  pixels.show();
} 


bool sampleTime(float sampleRate) {
  // Get the current time when this function is called
  long int currentTime = millis();

  // Find the elapsed time since last sampleRate
  long int elapsedTime = timeSample - currentTime;

  // Check if the elapsed time is larger than the sample rate
  if (elapsedTime >= sampleRate) {
    // Set the timeSample to the current time
    timeSample = currentTime;
    // The elapsed time has exceeded the sample rate
    return true;
  
  } else {
    // The elapsed time is less than the sample rate
    return false;
  }
}


void getSensorData(uint8_t * sensorData) {
  // Initialize all the sensor arrays
  uint8_t sensorPins[] = {A0, A4, A1, A5, A2, A6, A3, A7};
  uint16_t value;

  // Loop through each sensor
  for (int i = 0; i < 8; i ++) {
    // Reading sensor data and setting values in the pointer array 
     value = analogRead(sensorPins[i]);
     sensorData[i*2 + 0] = value & 0xFF;
     sensorData[i*2 + 1] = (value >> 8) & 0xFF;
  }
}

void setLedArray(uint8_t * colours) {
  // Re-map all of the LEDs physical locations to a spot more condusive for controlling 
  uint8_t ledMapping[] = {12, 13, 11, 10, 15, 14, 8, 9, 16, 17, 7, 6, 19, 18, 4, 5, 20, 21, 3, 2, 23, 22, 0, 1};
  
  // For each byte of colour data fetch the elements 
  for (int i = 0; i < NUMPIXELS; i ++) {
    // Set each LED with its data
    setLeds(ledMapping[i], colours[3*i], colours[3*i + 1], colours[3*i + 2]);
    //setLeds(i, colours[3*i], colours[3*i + 1], colours[3*i + 2]);
  }
  
  // Activate the pixel
  pixels.show();
}


// Set the leds 
void setLeds(int pixelIndex, int R, int G, int B) {
  // Make sure that the values don't exceed a total value of 510
  int totalValue = R + G + B;
  int maxBrightness = 510;
  bool isCompliant = false;
  
  // Check if the values are exceeding 510
  if (totalValue > maxBrightness) {
    // The value is exceeding the maximum, and we don't want it to explode
    isCompliant = false;
  
  } else {
    // It is in compliance with the power regulations, so it can continue
    isCompliant = true;
  }

  // Run the next part only if the led is within the power threshold
  if (isCompliant) {
    // Set the LED values
    pixels.setPixelColor(pixelIndex, pixels.Color(R, G, B));
  }
}


void loop() {  
  
  // Initialize varibles
  bool isPacketComplete = false;
  int index = 0;
  uint8_t checksum = 0;


  // loop while the packet is not complete
  while (isPacketComplete == false) {
    // Read from the serial port
    int value = Serial1.read();

    // Was there any data received?
    if(value != -1) {
      // If we are on the start of the packet, is the start equal to the magic byte?
      if (index == 0) {
        if (value == 0xA5) {
          // If we are at the start of the array and the first value is A5 than we can recording data
          colourData[index] = value;
          checksum += (uint8_t) value;
          index ++;
        } // If the value is not A5, continue looking for the magic byte 
      
      } else {
        // Once we have found the first byte, keep recording data 
        colourData[index] = value;
        
        // Don't add recived checksum to the checksum
        if (index != (COLOUR_SIZE - 1)) {
          checksum += (uint8_t) value;
        }
        
        index ++;
      }
      

      // If we are at the end of the packet 
      if (index == COLOUR_SIZE) {
        // Don't packet any more
        isPacketComplete = true;
      }
    }
  }

  // If the checksum does not match, exit the loop and start over
  if (checksum != colourData[COLOUR_SIZE-1]) {
    return;

    //char buffer [50];
    //sprintf (buffer, "received value %d, my value %d", colourData[COLOUR_SIZE-1], checksum);
    //Serial.println(buffer);
  }

  // Set the leds 
  // TODO:  Change this to start at index 1 and drop one byte on the packet (74 instead of 75)
  //        Right now we have to send a dummy byte as the second byte to get things to align.
  setLedArray(&colourData[2]);

  // Adding sensor data to the packet array starting at element 2
  getSensorData(&packetData[2]);

  // Checksum the header, version and data bytes but not the checksum byte
  // Checksum is 8 bits.
  checksum = 0;

  for(index = 0; index < (PACKET_SIZE-1); index++) {
    checksum += packetData[index];
  }
  packetData[PACKET_SIZE-1] = checksum;

  // Send the data
  Serial1.write(packetData, PACKET_SIZE);

}
