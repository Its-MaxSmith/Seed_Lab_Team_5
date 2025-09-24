// Assignment 2 1a Arduino
// Max Smith
// 9/8/25
//
// Arduino program that links to a RPi to read a string from the Pi
// to Arduino and displays the string character and ASCII codes

// Includes
#include <Wire.h>

// I2C address also set as 8 on RPi
#define MY_ADDR 8

// Global variables
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;


// Setup
void setup() {

  // Baud rate set to 9600 match RPi
  Serial.begin(9600);

  // Initialize I2C
  Wire.begin(MY_ADDR);

  // Set callbacks for I2C interupts
  Wire.onReceive(receive);

} // End setup


// Body code - infinite loop
void loop() {

  // If there is data on the buffer read buffer
  if (msgLength > 0) {
    
    if (offset == 1) {
        digitalWrite(LED_BUILTIN, instruction[0]);
    }

    printReceived();
    msgLength = 0;
  }

} // End body loop


// Print recieved function
void printReceived() {

  // Print to the serial monitor
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("String recieved: ");

  // Print string loop - return nothing
  for (int i = 0; i < msgLength; i++) {
    Serial.print((char)instruction[i]);  
  }

  Serial.println();
  Serial.print("ASCII code: ");

  // Print ASCII loop
  for (int i = 0; i < msgLength; i++) {
    Serial.print(String(instruction[i]) + " ");
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