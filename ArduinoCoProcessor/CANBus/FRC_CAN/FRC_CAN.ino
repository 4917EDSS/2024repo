
#include "frc_mcp2515.h"
#include "frc_CAN.h"

// Define the CS pin and the interrupt pin
#define CAN_CS 10
#define CAN_INTERRUPT 2

// Define the switches
int button1 = 4;
int button2 = 3;

// Create an MCP2515 device. Only need to create 1 of these
frc::MCP2515 mcp2515{CAN_CS};

// Create an FRC CAN Device. You can create up to 16 of these in 1 progam
// Any more will overflow a global array.  The device number, manufacturer 
// and devicetype are basically filters for all incoming data.  You will 
// receive all callbacks associated with these and further process it
// based on the API ID and data packets.
frc::CAN frcCANDevice{3, frc::CANManufacturer::kTeamUse, frc::CANDeviceType::kMiscellaneous};


// Callback function. This will be called any time a new message is received
// Matching one of the enabled devices.
void CANCallback(frc::CAN* can, int apiId, bool rtr, const frc::CANData& data) {
    Serial.print("In callback. API: ");
    Serial.print(apiId, HEX);

    Serial.print(" RTR: ");
    Serial.print(rtr);

    Serial.print(" Data: ");

    for (int i = 0; i < data.length; i++)
    {
      Serial.print(data.data[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback(uint32_t id, const frc::CANData& data) {
    /*Serial.print("Unknown message ");
    Serial.print(id & CAN_EFF_MASK, HEX);

    Serial.print(" Data: ");

    for (int i = 0; i < data.length; i++)
    {
      Serial.print(data.data[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");    */
}


void setup() {
    // Begin serial port
    Serial.begin(115200);
    
    // Initialize the MCP2515. If any error values are set, initialization failed
    auto err = mcp2515.reset();
    // CAN rate must be 1000KBPS to work with the FRC Ecosystem
    // Clock rate must match clock rate of CAN Board.
    err = mcp2515.setBitrate(frc::CAN_1000KBPS, frc::CAN_CLOCK::MCP_16MHZ);

    // Set up to normal CAN mode
    err = mcp2515.setNormalMode();
    //err = mcp2515.setListenOnlyMode();

    // Prepare our interrupt pin
    pinMode(CAN_INTERRUPT, INPUT);

    // Define the buttons that have been attached
    // They are INPUT_PULLUP because when the switch is high, the pin will get pulled to GND 
    pinMode(button1, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);
    
    // Set up FRC CAN to be able to use the CAN Impl and callbacks
    // Last parameter can be set to nullptr if unknown messages should be skipped
    frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, UnknownMessageCallback);

    // All CAN Devices must be added to the read list. Otherwise they will not be handled correctly.
    frcCANDevice.AddToReadList();
}

unsigned long long lastSendMs = 0;
int count = 0;

void loop() {
    // Update must be called every loop in order to receive messages
    frc::CAN::Update();

    uint8_t data[8];

    // Writes can happen any time, this uses a periodic send
    auto now = millis();
    if (now - lastSendMs > 100) {
        lastSendMs += 100;

        // zero memory buffer
        memset(data, 0, 8);

        // increment byte 0
        data[0] = count;
        count++;

        // Check button 1
        if (digitalRead(button1) == LOW) {
          // Button has been pressed, set 1
          data[1] = 1;
        
        } else {
          // Button not pressed, set 0
          data[1] = 0;
        }

        // Check button 2
        if (digitalRead(button2) == LOW) {
          // Button 2 has been pressed!
          data[2] = 1;

        } else {
          // Button is not currently being pressed
          data[2] = 0;
        }


        // Send bytes. The API command is any 10 bit value specific to the device though
        // typically used for commands and configuration.
        frcCANDevice.WritePacket(data, 8, 0x123);
    }
}
