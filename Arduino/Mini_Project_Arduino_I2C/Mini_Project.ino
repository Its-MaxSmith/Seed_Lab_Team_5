// CODE WRITTEN BY BENJAMIN SMITH AND STEPHEN THOMAS

#include <Wire.h>

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
#define myAddress 8

// GLOBAL VARS
bool LAST_R_A;
bool LAST_L_A;
long RIGHTWHEELCOUNTS = 0;
long LEFTWHEELCOUNTS = 0;
char MESSAGE[32];
int MESSAGE_LENGTH = 0;


void receive();

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
  attachInterrupt(digitalPinToInterrupt(rightWheelEncoderA), rightEncoderInterupt, HIGH);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoderA), leftEncoderInterupt, HIGH);

  Wire.begin(myAddress);
  Wire.onReceive(receive);
  
  
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
  float leftTangentalVelocity = 0;
  float rightTangentalVelocity = 0;
  float centerDistance = 0;
  float centerSpeed = 0;
  //clanker orientation
  float phi = 0;
  float xPos = 0;
  float yPos = 0;
  //Clanker Outputs
  float rightPWM = 0;
  float leftPWM = 0;

  //clanker targets
  float targetRSpeed = 0;
  float targetLSpeed = 0;
  float targetRPos =0;
  float targetLPos =0;

  //Control Constants
  float rightKp = 70;
  float leftKp = 70;
  
  float rightKi = 2;
  float leftKi = 2;
  while (true)
  {
    //Wait for next loop
    while(millis() < currentTime){}
    leftWheelPos = (float)LEFTWHEELCOUNTS/1600*6.28;
    rightWheelPos = (float)RIGHTWHEELCOUNTS/1600*6.28;
    leftWheelSpeed = (leftWheelPos - leftWheelLastPos)/0.02;
    rightWheelSpeed = (rightWheelPos - rightWheelLastPos)/0.02;
    leftDistance = leftWheelPos * wheelRadius;
    rightDistance = rightWheelPos * wheelRadius;
    leftTangentalVelocity = leftWheelSpeed * wheelRadius;
    rightTangentalVelocity = rightWheelSpeed * wheelRadius;
    centerDistance = (leftDistance + rightDistance) / 2.0;
    centerSpeed = (leftTangentalVelocity + rightTangentalVelocity) / 2.0;
    phi = (rightDistance - leftDistance) / clankerWidth;
    //keep phi within bounds (0,2pi)
    phi = fmod(phi, 2.0*PI);
    if (phi < 0) phi += 2.0*PI;
    //update x and y position based upon phi value
    xPos += centerSpeed * cos(phi) * 0.02;
    yPos += centerSpeed * sin(phi) * 0.02;


    for (int i = 0; i < MESSAGE_LENGTH; i++)
        Serial.print(MESSAGE[i]);
    Serial.print("");
    if (MESSAGE_LENGTH == 0)
      Serial.println("No message received");

    //Set target position from message
    if (MESSAGE[0]== '0'){
    targetLPos=3.14;
//    Serial.print("0");
    }
    else if (MESSAGE[0]== '1'){
    targetLPos=0;
//    Serial.print("1");
    }
    if (MESSAGE[1]== '0'){
    targetRPos=3.14;
//    Serial.print("0");
    }
    else if (MESSAGE[1]== '1'){
    targetRPos=0;
//    Serial.print("1");
    }
//    Serial.println("");
     
    float errorRP = targetRPos- rightWheelPos;
    float errorLP = targetLPos- leftWheelPos;
    while(Serial.available()){Serial.read();}
    
    targetRSpeed=errorRP*rightKi;
    targetLSpeed=errorLP*leftKi;
 

    float errorRV = targetRSpeed - rightWheelSpeed;
    float errorLV = targetLSpeed - leftWheelSpeed;

    

    

    
//    Serial.print(currentTime, 5);
//    Serial.print(" ");
//    Serial.print(targetRPos, 5);
//    Serial.print(" ");
//    Serial.println(targetLPos, 5);
   // Serial.print(" ");
    //Serial.println(phi, 5);
    


//    Serial.print(RIGHTWHEELCOUNTS);
//    Serial.print(" ");
//    Serial.println(LEFTWHEELCOUNTS);
//    Serial.print(" ");
//    Serial.print(leftPWM);
//    Serial.print(" ");
//    Serial.print(rightPWM);
//    Serial.print(" ");
//    Serial.print(leftWheelSpeed);
//    Serial.print(" ");
//    Serial.println(rightWheelSpeed);



    //Apply low-pass filter to outputs to reduce gittering
    rightPWM = rightPWM*0.95 + (errorRV * rightKp)*0.05;
    leftPWM =  leftPWM *0.95 + (errorLV * leftKp)*0.05;

    //Spin motors
    spinMotors(rightPWM, leftPWM);

    //Update all previous values
    leftWheelLastPos = leftWheelPos;
    rightWheelLastPos = rightWheelPos;
    lastTime = currentTime;
    currentTime = lastTime + 20;
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

void spinMotors(float R, float L)
{
    L = constrain(L, -255, 255);
    R = constrain(R, -255, 255);
  
    if (L >= 0)
      digitalWrite(leftDirection, HIGH);
    else
    {
      digitalWrite(leftDirection, LOW);
      L *= -1;
    }
    if (R >= 0)
      digitalWrite(rightDirection, HIGH);
    else
    {
      digitalWrite(rightDirection, LOW);
      R *= -1;
    }
    R = constrain(R, 0, 255);
    L = constrain(L, 0, 255);
    analogWrite(leftPWMpin, L);
    analogWrite(rightPWMpin, R);
}

void receive()
{
  Wire.read();
  MESSAGE_LENGTH = 0;
  while(Wire.available())
  {
    MESSAGE[MESSAGE_LENGTH] = Wire.read();
    MESSAGE_LENGTH++;
    if (MESSAGE_LENGTH > 31)
      break;
  }
  

//  Serial.print(MESSAGE[0]);
//  Serial.println(MESSAGE[1]);
}
