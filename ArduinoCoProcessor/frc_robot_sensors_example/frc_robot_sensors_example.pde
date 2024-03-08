// Written by Aidan Fiser

import processing.serial.*;

Serial ser;

void setup() {
  //                     Port  Baud rate
  ser = new Serial(this,"COM11",38400);
  println("Connected");
  
  delay(1000); // Wait for arduino to connect
}

void getRawData() {
  int prevByte = 0; // Sensors are two byte values
  int byteCount = 0;  // Keeps track of current byte (out of 19 for sensors)
  // Send sensor command
  ser.write(0xA5);
  ser.write(0x01);
  ser.write(0xA6);
  
  for(int e=0;e<19;e++) { // Expects the 19 bytes from the sensor command (ignores checksum)
    while(ser.available() <= 0); // Wait until something is in the serial buffer 
    int b = ser.read();
     if(b != 0xA5 && byteCount < 16) { // Make sure it isn't the command byte and it is one of the 8 sensors (2 bytes each)
         if(byteCount % 2 == 1) { // Merge two bytes every other byte read
           int fullByte = prevByte|b<<8;
           print(fullByte+ "("+byteCount/2+") | "); 
         }
         prevByte = b; // Keep track of previous byte for later
         byteCount++;
     }
     else {
       ser.read();
       e+=1; // Skip version byte
       byteCount = 0;
     }
  }
   println();
 println();
 }

void draw() { // Loop
  getRawData();
}
