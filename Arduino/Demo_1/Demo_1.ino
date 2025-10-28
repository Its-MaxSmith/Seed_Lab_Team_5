// CODE WRITTEN BY BENJAMIN SMITH AND STEPHEN THOMAS
// This code is used for mutliple purposes, requiring an arduino microcontroller connected to 2 motors with encoders attached, and a battery connection to work properly.
// This code records the distance the motors travel and orientation angle, along with getting the robot to move to specified orientation and distance values based upon user input to the serial monitor, input as phi (in radians), and distance to move forward (in meters).
// This code functions using PI controllers and interrupts, all of which have their respective sections labeled, along with any other sections required to achieve the desired output labeled as well.

//libraries necessary for proper usage
#include <Wire.h>
//definitions of values to be used in code
#define rightWheelEncoderA 2
#define rightWheelEncoderB 6
#define leftWheelEncoderA 3
#define leftWheelEncoderB 5
#define rightPWMpin 10
#define leftPWMpin 9
#define rightDirection 8
#define leftDirection 7
#define safteyPin 4
#define wheelRadius 0.075
#define clankerWidth 0.14
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
  //initiallize pins and serial monitor
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

  //Initialize encoder values
  bool LAST_R_A = digitalRead(rightWheelEncoderA);
  bool LAST_L_A = digitalRead(leftWheelEncoderA);
  attachInterrupt(digitalPinToInterrupt(rightWheelEncoderA), rightEncoderInterupt, HIGH);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoderA), leftEncoderInterupt, HIGH);

  //initialize I2C communication
  Wire.begin(myAddress);
  Wire.onReceive(receive);

  //Initialize motor drivers
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
  float targetCP = 0;
  float targetPhi = 0;
  float maxWheelSpeed = 7;

  //Control Constants
  float rightKp = 70;
  float leftKp = 70;
  float centerKp = 2;
  float centerKi = 0.5;
  float centerIntegral = 0;
  float phiKp = .225;
  float phiKi = .1;
  float phiIntegral = 0;

  // determine current position and orientation
  while (true)
  {
    //Wait for next loop
    while (millis() < currentTime) {}

    leftWheelPos = (float)LEFTWHEELCOUNTS / 1600 * 6.28;
    rightWheelPos = (float)RIGHTWHEELCOUNTS / 1600 * 6.28;

    leftDistance = leftWheelPos * wheelRadius;
    rightDistance = rightWheelPos * wheelRadius;

    leftWheelSpeed = (leftWheelPos - leftWheelLastPos) / 0.02;
    rightWheelSpeed = (rightWheelPos - rightWheelLastPos) / 0.02;

    leftTangentalVelocity = leftWheelSpeed * wheelRadius;
    rightTangentalVelocity = rightWheelSpeed * wheelRadius;

    centerDistance = (leftDistance + rightDistance) / 2.0;
    centerSpeed = (leftTangentalVelocity + rightTangentalVelocity) / 2.0;
    phi = (rightDistance - leftDistance) / clankerWidth / 2.0;

    //update x and y position based upon phi value
    xPos += centerSpeed * cos(phi) * 0.02;
    yPos += centerSpeed * sin(phi) * 0.02;

    // allow user input from serial monitor for phi and position
    if (Serial.available()) {
      targetPhi = Serial.parseFloat();
      if (Serial.peek() == ',') {
        Serial.read();
      }
      targetCP = Serial.parseFloat();
      while (Serial.available()) {
        Serial.read();
      }
    }

    // deterimine error values to be used in PI controllers
    float errorPhi = targetPhi - phi;
    float errorCP = targetCP - centerDistance;

    /*Serial.print(targetCP);
      Serial.print(" ");
      Serial.print(centerDistance);
      Serial.print(" ");
      Serial.print(targetPhi);
      Serial.print(" ");
      Serial.println(phi);
    */

    //Intergal controller
    centerIntegral = centerIntegral * 0.95 + errorCP;
    phiIntegral = phiIntegral * 0.95 + errorPhi;


    // Position to wheel speed
    targetRSpeed = errorCP * centerKp + centerIntegral * centerKi;
    targetLSpeed = errorCP * centerKp + centerIntegral * centerKi;
    // Phi to wheel speed
    targetRSpeed += errorPhi * phiKp + phiIntegral * phiKi;
    targetLSpeed -= errorPhi * phiKp + phiIntegral * phiKi;
    //
    float errorRV = targetRSpeed - rightWheelSpeed;
    float errorLV = targetLSpeed - leftWheelSpeed;










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


    //Limit robot speed
    if (errorRV > maxWheelSpeed && errorRV > errorLV)
    {
      errorLV = errorLV / errorRV * maxWheelSpeed;
      errorRV = maxWheelSpeed;
    }
    if (errorLV > maxWheelSpeed && errorLV > errorRV)
    {
      errorRV = errorRV / errorLV * maxWheelSpeed;
      errorLV = maxWheelSpeed;
    }

    //Apply low-pass filter to outputs to reduce gittering
    rightPWM = rightPWM * 0.95 + (errorRV * rightKp) * 0.05;
    leftPWM =  leftPWM * 0.95 + (errorLV * leftKp) * 0.05;


    //    Serial.print(currentTime, 5);
    //    Serial.print(" ");
    //    Serial.print(centerDistance, 5);
    //    Serial.print(" ");
    //    Serial.print(leftPWM);
    //    Serial.print(" ");
    //    Serial.println(rightPWM);

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
    if (B != A)
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
    if (B != A)
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
  // motor controls
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

  //if (L < 10) L = 0;
  //if (R < 10) R = 0;

  //Serial.print(L);
  //Serial.print(" ");
  //Serial.println(R);


  analogWrite(leftPWMpin, L);
  analogWrite(rightPWMpin, R);
}

void receive()
{
  //I2C communication function
  Wire.read();
  MESSAGE_LENGTH = 0;
  while (Wire.available())
  {
    MESSAGE[MESSAGE_LENGTH] = Wire.read();
    MESSAGE_LENGTH++;
    if (MESSAGE_LENGTH > 31)
      break;
  }
  //  Serial.print(MESSAGE[0]);
  //  Serial.println(MESSAGE[1]);
}
/*void receiveFromSerial()
  {
  int i=0;
  while(Serial.available())
  {
   MESSAGE[i]= Serial.read();
   i++;
   if( i >31)
   break;
  }
  }
*/
