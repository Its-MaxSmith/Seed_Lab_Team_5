

// CODE WRITTEN BY BENJAMIN SMITH
// This code will read the state of the encoders through the interupts.


#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;


#define rightWheelEncoderA 2
#define rightWheelEncoderB 6

#define leftWheelEncoderA 3
#define leftWheelEncoderB 5

#define rightPWMpin 10
#define leftPWMpin 9
#define rightDirection 8
#define leftDirection 7
#define safteyPin 4

#define wheelRadius 0.06
#define clankerWidth 0.15


// GLOBAL VARS
bool LAST_R_A;
bool LAST_L_A;
int RIGHTWHEELCOUNTS = 0;
int LEFTWHEELCOUNTS = 0;





void setup() {
  
  Serial.begin(9600);
  pinMode(rightWheelEncoderA, INPUT);
  pinMode(rightWheelEncoderB, INPUT);
  pinMode(leftWheelEncoderA, INPUT);
  pinMode(leftWheelEncoderB, INPUT);
  pinMode(rightPWMpin, OUTPUT);
  pinMode(leftPWMpin, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(leftDirection, OUTPUT);
  pinMode(safteyPin, OUTPUT);
  
  //Initial encoder values
  bool LAST_R_A = digitalRead(rightWheelEncoderA);
  bool LAST_L_A = digitalRead(leftWheelEncoderA);

  attachInterrupt(digitalPinToInterrupt(rightWheelEncoderA), rightEncoderInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoderA), leftEncoderInterupt, CHANGE);

  
  //Initialise motor drivers
  digitalWrite(safteyPin, HIGH);


}

void loop() {
  long currentTime = millis();
  long lastTime = 0;

  //wheel angular position
  float leftWheelPos = 0;
  float rightWheelPos = 0;
  float leftWheelLastPos = 0;
  float rightWheelLastPos = 0;
  
  //wheel angular velocity
  float leftWheelSpeed = 0;
  float rightWheelSpeed = 0;

  //wheel position
  float leftDistance = 0;
  float rightDistance = 0;
  float centerDistance = 0;

  //clanker orientation
  float phi = 0;
  float xPos = 0;
  float yPos = 0;

  //Clanker Outputs
  float rightPWM = 0;
  float leftPWM = 0;

  
  //clanker targets
  float targetSpeed = 6.28;

  //Control Constants
  float rightKp = 5;
  float leftKp = 5;

  while (true)
  {
    //Wait for next loop
    while(millis() < currentTime){}

    leftWheelPos = (float)LEFTWHEELCOUNTS/1600*6.28;
    rightWheelPos = (float)RIGHTWHEELCOUNTS/1600*6.28;

    leftWheelSpeed = (leftWheelPos - leftWheelLastPos)/0.02;
    rightWheelSpeed = (rightWheelPos - rightWheelLastPos)/0.02;



    if (millis() < 1000)
      rightPWM = 0;
    else if (millis() < 3000)
      rightPWM = 86;
    else
      rightPWM = 0;
    


    digitalWrite(leftDirection, HIGH);
    analogWrite(leftPWMpin, leftPWM);
    digitalWrite(rightDirection, HIGH);
    analogWrite(rightPWMpin, rightPWM);





      if (millis() < 3000)
      {
        Serial.print(millis()/1000.0);
        Serial.print('\t');
        Serial.print(rightPWM *0.0345);
        Serial.print('\t');
        Serial.println(rightWheelSpeed);
      }

    //Update all previous values
    leftWheelLastPos = leftWheelPos;
    rightWheelLastPos = rightWheelPos;

    lastTime = currentTime;
    currentTime = lastTime + 1;
  }
}









void rightEncoderInterupt()
{
  //ENCODER SECTON
    bool A = digitalRead(rightWheelEncoderA);
    bool B = digitalRead(rightWheelEncoderB);
    
    //Dark magic
    if (A != LAST_R_A)
      if(B != A)
      {
        RIGHTWHEELCOUNTS++;
      }
      else
      {
        RIGHTWHEELCOUNTS--;
      }

    // Save prev states
    LAST_R_A = A;
}


void leftEncoderInterupt()
{
  //ENCODER SECTON
    bool A = digitalRead(leftWheelEncoderA);
    bool B = digitalRead(leftWheelEncoderB);
    
    //Dark magic
    if (A != LAST_L_A)
      if(B != A)
      {
        LEFTWHEELCOUNTS--;
      }
      else
      {
        LEFTWHEELCOUNTS++;
      }

    // Save prev states
    LAST_L_A = A;
}
