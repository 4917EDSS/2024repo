// This project was compiled using Arduino IDE 2.3.2
// The board is an "Arduino Nano Every"
// Download "Arduino megaAVR Boards 1.8.8" from Tools > Board > Board Manager to be able to select this board (Tools > Board)
// Download "Adafruit NeoPixel by Adafruit" from Tools > Manage Libraries


// Import neopixel library for LEDs
#include <Adafruit_NeoPixel.h>

// Version of the roboRIO/Arduino protocol
#define VERSION 0x03

// Define adafruit neopixel stuff
#define PIN 2
#define NUM_PIXELS 24
#define DELAYVAL 500

// Number of sensors that the board (or board pair) supports
#define NUM_SENSORS 8

// Number of sensor samples to save for averaging
#define NUM_SAMPLES_TO_SAVE  10

// Incoming messages handler states
#define STATE_HEADER 0  // Looking for the header byte
#define STATE_COMMAND 1  // Looking for the command byte
#define STATE_PARAMETERS 2  // Looking for the paramter bytes
#define STATE_CHECKSUM 3  // Looking for the checksum

#define HEADER_BYTE 0xA5

// Commands from the roboRIO
#define COMMAND_GET_SENSORS 0x01
#define COMMAND_SET_LED_HALVES 0x02
#define COMMAND_SET_EACH_LED 0x03

// Transmit: Header byte, protocol version byte, sensor data (2 bytes per sensor), checksum
#define TX_PACKET_SIZE (2 + NUM_SENSORS * 2 + 1)
// Receive: Header byte, command byte, longest command parameters, checksum)
// Probably won't sture header, command or checksum in here but leave room in case
#define RX_MAX_PACKET_SIZE (2 + NUM_PIXELS * 3 + 1)


Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint8_t transmitData[TX_PACKET_SIZE] = {0};
uint8_t receiveData[RX_MAX_PACKET_SIZE] = {0};
unsigned int sensorData[NUM_SENSORS][NUM_SAMPLES_TO_SAVE] = {0};
int sampleIndex = 0;
// Need to know if we haven't yet gotten NUM_SAMPLES_TO_SAVE sensor samples saved
bool sensorSamplesFull = false;

// Create a varible for elapsed time
long int timeSample = millis();


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  
  transmitData[0] = HEADER_BYTE;   // Header byte to know this is the start of the packet
  transmitData[1] = VERSION;

  pinMode(LED_BUILTIN, OUTPUT);

  pixels.begin();
  
  // Set all the leds to green
  for (int i = 0; i < NUM_PIXELS; i++) {
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


// Set the leds 
void setLeds(int pixelIndex, int R, int G, int B) {
  // Make sure that the values don't exceed a total value of 510
  int totalValue = R + G + B;
  int maxBrightness = 510; // If this value drops below 3*128, update the code below
  
  // Check if the values are exceeding 510
  if (totalValue > maxBrightness) {
    // The value is exceeding the maximum, and we don't want it to explode
  // So set each colour to half the brightness
  // value >>= 1 means shift all bits to the right by one which is the same
  // as dividing it by 2 but much more efficient computationally
    R >>= 1;
    G >>= 1;
    B >>= 1;  
  }

  // Set the LED values
  pixels.setPixelColor(pixelIndex, pixels.Color(R, G, B));
}


void setLedArray(uint8_t command, uint8_t *colours) {
  // Re-map all of the LEDs physical locations to a spot more condusive for controlling
  // 'static' means that this variable is created on bootup and lives forever
  static uint8_t ledMapping[] = {12, 13, 11, 10, 15, 14, 8, 9, 16, 17, 7, 6, 19, 18, 4, 5, 20, 21, 3, 2, 23, 22, 0, 1};
  
  switch(command) {
    case COMMAND_SET_LED_HALVES:
      for (int i = 0; i < NUM_PIXELS; i++) {
        // Set first half of LEDs with one RGB colour
        if(i < (NUM_PIXELS / 2)){
          setLeds(ledMapping[i], colours[0], colours[1], colours[2]);
        } 
        // Set second half of LEDs with the second RGB colour
        else {
          setLeds(ledMapping[i], colours[3], colours[4], colours[5]);
        }
      }
      break;
    
    case COMMAND_SET_EACH_LED:
      // Set each LED to the specified RGB colour
      for (int i = 0; i < NUM_PIXELS; i++) {
        setLeds(ledMapping[i], colours[3*i], colours[3*i + 1], colours[3*i + 2]);
      }
      break;
    
   default:
    return;
    break;
  }
    
  // Activate the pixel
  pixels.show();
}


void getSensorData() {
  // Initialize all the sensor arrays
  // 'static' means that this variable is created on bootup and lives forever
  static uint8_t sensorPins[] = {A0, A4, A1, A5, A2, A6, A3, A7};
  
  // Loop through each sensor
  for (int i = 0; i < NUM_SENSORS; i ++) {
    // Reading sensor data and setting values
     sensorData[i][sampleIndex] = analogRead(sensorPins[i]);
  }
  
  // Increment the sample index and go back to 0 if it's at the end
  sampleIndex++;
  if (sampleIndex >= NUM_SAMPLES_TO_SAVE) {
    sampleIndex = 0;
    sensorSamplesFull = true; // We can now average over the whole sample buffer
  }
}


unsigned int getAveragedSensorValue(int sensorId) {
  unsigned int sum = 0;
  
  for (int i = 0; i < NUM_SAMPLES_TO_SAVE; i ++) {
    sum += sensorData[sensorId][i];
  }
  
  if(sensorSamplesFull) {
    return sum / NUM_SAMPLES_TO_SAVE;
  } else {
    return sum / sampleIndex;
  }
}


void sendSensorData() {
  unsigned int value;
  uint8_t checksum = 0;

  // Add all the sensors to the transmit packet
  for (int i = 0; i < NUM_SENSORS; i++) {
    value = getAveragedSensorValue(i);
    transmitData[i*2 + 2] = value & 0xFF;
    transmitData[i*2 + 3] = (value >> 8) & 0xFF;
  }

  // Add the checksum
  for (int i = 0; i < (TX_PACKET_SIZE - 1); i++) {
    checksum += transmitData[i];
  }
  transmitData[TX_PACKET_SIZE - 1] = checksum;

  Serial1.write(transmitData, TX_PACKET_SIZE);
}


void handleMessage(uint8_t command, uint8_t *parameters) {
  switch(command) {
    case COMMAND_GET_SENSORS:
      // Send data without changing our LEDs
      sendSensorData();
      break;

    case COMMAND_SET_LED_HALVES:
      // setLedArray can handle both types of LED commands
      setLedArray(command, parameters);
      sendSensorData();
      break;

    case COMMAND_SET_EACH_LED:
      // setLedArray can handle both types of LED commands
      setLedArray(command, parameters);
      sendSensorData();
      break;

    default:
      // Unknown command so just ignore it
      break;
  }
}


// Process incoming message bytes and act on completed messages
void receiveMessage() {
  // 'static' means that this variable is created on bootup and lives forever
  static int state = STATE_HEADER;
  static uint8_t command = 0;
  static int numParameterBytes = 0;
  static int nextParameterByteIndex = 0;

  uint8_t calculatedChecksum;
  uint8_t actualChecksum;
  
  while (Serial1.available()) {
    switch (state) {
      case STATE_HEADER:
        // If we found the header, look for the command byte next.  Otherwise keep looking
        if (Serial1.read() == HEADER_BYTE) {
          state = STATE_COMMAND;
        }
        break;

      case STATE_COMMAND:
        command = Serial1.read();

        // Figure out how many parameter bytes we are getting
        switch(command) {
          case COMMAND_GET_SENSORS:
            numParameterBytes = 0;
            state = STATE_CHECKSUM;
            break;

          case COMMAND_SET_LED_HALVES:
            numParameterBytes = 6;
            nextParameterByteIndex = 0;
            state = STATE_PARAMETERS;
            break;

          case COMMAND_SET_EACH_LED:
            numParameterBytes = 72;
            nextParameterByteIndex = 0;
            state = STATE_PARAMETERS;
            break;

          default:
            // Unknown command so just start looking for header again
            state = STATE_HEADER;
            break;
        }
        break;

      case STATE_PARAMETERS:
        receiveData[nextParameterByteIndex++] = Serial1.read();
        if (nextParameterByteIndex >= numParameterBytes) {
          state = STATE_CHECKSUM;
        }
        break;

      case STATE_CHECKSUM:
        actualChecksum = Serial1.read();

        calculatedChecksum = HEADER_BYTE;
        calculatedChecksum += command;
        for (int i = 0; i < numParameterBytes; i++) {
          calculatedChecksum += receiveData[i];
        }

        if(actualChecksum == calculatedChecksum) {
          // Message is good, act on it
          handleMessage(command, receiveData);
        }
        // Else, we could search through the paramter bytes to see if we find another 0xA5 to start the message
        // but that's much more complicated and most of the time our message will be simple: 0xA5 0x01 0xA6
        // which makes it easy to resync to if we lose a byte.

        state = STATE_HEADER;
        command = 0;
        numParameterBytes = 0;
        nextParameterByteIndex = 0;
        break;
    }
  }
}

void loop() {  
  getSensorData();
  receiveMessage();
}
  
