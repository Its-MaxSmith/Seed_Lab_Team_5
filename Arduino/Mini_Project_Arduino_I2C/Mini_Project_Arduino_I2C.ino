// Mini Project I2C stuff
// Max Smith
// 9/29/25

// Includes
#include <Wire.h>

// I2C address set as 8
#define MY_ADDR 8


// Global Variables
volatile uint8_t offset = 0;
volatile uint8_t = instruction[1] = {0};
volatile uint8_t msgLength = 0;




// Required Setup
void setup() {

  // Baud rate set to 9600
  Serial.begin(9600); // Start serial reading

  // Initialize I2C
  Wire.begin(MY_ADDR);

  // Set callbacks for I2C interupts
  Wire.onReceive(receive);

} // End setup




// Body code
void loop() {

  if (msgLength > 0) printReceived();
  msglength = 0;

}




// I2C functions

// Print recieved function
void printReceived() {

  // Print to the serial monitor
  Serial.print("String recieved: ");

  // Print string loop - return nothing
  for (int i = 0; i < msgLength; i++) {
    Serial.print((char)instruction[i]);  
  }

  Serial.println();
  }
  
} // End printReceived function

// Receive function - return nothing
void receive() {

   // Set the offset as first byte
   offset = Wire.read();

   // If there is information after the offset read while
   while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
   }
   
} // End receive function