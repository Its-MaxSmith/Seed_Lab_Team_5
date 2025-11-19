// CODE WRITTEN BY BENJAMIN SMITH AND STEPHEN THOMAS
// This code incorporates multiple functionalities together. The code works with an Arduino Uno, 2 motors with encoders attached, a battery connection, and a IIC protocol setup to drive a robot using a state machine.
//The code has every section used for achieving the goal of controlling a robot using a state machine labeled, with sections including the controls for the state machine itself, the encoder interrupts section, 
//the spin motors section, the recieve message section for communication, and many more. The code works by controlling the state of the robot based upon the data received from a raspberry pi micrcontroller
//communicating with the arduino, sending data in the form of forward distance, turn angle left or right, and following steps( either turning left 90 degrees, right 90 degrees, or nothing), which 
//is recorded to the arduino, then fed back into the state machine to determine where the robot is currently, and what state it should move into next. 

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
#define wheelRadius 0.074
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

  // for IIC
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
  float targetCP = 0;
  float targetPhi = 0;
  float maxWheelSpeed = 1.5;

  //Control Constants
  float rightKp = 70;
  float leftKp = 73;
  float errorCP = 0;
  float errorPhi = 0;
  float centerKp = 2;
  float centerKi = 1;
  float centerIntegral = 0;
  float phiKp = 0.225;
  float phiKi = 0.13;
  float phiIntegral = 0;

  // CLANKER STATE
  char command = '\0'; // Default no command
  int state = 0; // Default to do nothing state
  float forward_command_ammount = 0;
  bool newforwardCommandFlag = false;
  float phi_correction = 0;
  bool newPhiCorrectionFlag = false;
  bool hasSearched = false;
  int countR = 0;
  int countL = 0;
  int countN = 0;
  
  while (true)
  {
        //Wait for next loop
    while(millis() < currentTime){}

    if(Serial.available())
      receiveFromSerial();

    
    //////////////////////////STATE MACHINE////////////////////////////////
    //STATE 0 -> DO NOTHING
    //STATE 1 -> GO FORWARD
    //STATE 2 -> TURN 90 RIGHT
    //STATE 3 -> TURN 90 LEFT
    //STATE 4 -> SEARCH
    //STATE 5 -> FINNISH

    switch (state)
    {
      case 0: // Start Do nothing state
        RIGHTWHEELCOUNTS = 0;
        LEFTWHEELCOUNTS = 0;
        targetCP = 0;
        targetPhi = 0;
        if (command == 'S')
        {
          Serial.println("Entering SEARCH state");
          state = 4;
          RIGHTWHEELCOUNTS = 0;
          LEFTWHEELCOUNTS = 0;
          targetCP = 0;
          targetPhi = 0;
          phi = 0;
          newPhiCorrectionFlag = false;
          
          command = '\0';
        }
        else if (newforwardCommandFlag)
        {
          Serial.println("Entering FORWARD state");
          state = 1;
          RIGHTWHEELCOUNTS = 0;
          LEFTWHEELCOUNTS = 0;
          newPhiCorrectionFlag = false;
          targetCP = forward_command_ammount;
          newforwardCommandFlag = false;
          int countR = 0;
          int countL = 0;
          int countN = 0;
        }
        break;

      case 1: ///DRIVE FORWARD
      
      //Forward movement correction
      if (newforwardCommandFlag) // If we have a distance estimation
      {
        targetCP = (centerDistance + forward_command_ammount - 0.08);
        newforwardCommandFlag = false;
      }

      //Left Right correction
      if (newPhiCorrectionFlag) // If we have a distance estimation
      {
        targetPhi = phi + phi_correction * 1.5;
        newPhiCorrectionFlag = false;
      }

      //Sum commands for determining direction
      if(abs(errorCP) < 1.5)
      {
        if (command == 'L')
          countL++;
        else if (command == 'R')
          countR++;
        else if (command == 'N')
          countN++;
      }
      command = '\0';
      
      
      
      //IF move is finished
      if (fabs(errorCP) < .15)
        {
          if (countL >= countR )
          {
            Serial.println("Entering LEFT TURN state");
            RIGHTWHEELCOUNTS = 0;
            LEFTWHEELCOUNTS = 0;
            targetPhi = 1.628;
            targetCP = 0;
            state = 3;
          }
          else if (countR >= countL )
          {
            Serial.println("Entering RIGHT TURN state");
            RIGHTWHEELCOUNTS = 0;
                        LEFTWHEELCOUNTS = 0;
            targetPhi = -1.628;
            targetCP = 0;
            state = 2;
          }
//         else if (countN >= countL && countN >= countR)
//          {
//            Serial.println("Entering FINISHED state");
//            state = 5;
//          }
        }
      break;

      case 2: //TURN RIGHT
      targetCP = 0.1;
      if (fabs(errorPhi) < 0.1)// IF turn is finnished
      {
        Serial.println("Entering FINISHED state");
        RIGHTWHEELCOUNTS = 0;
        LEFTWHEELCOUNTS = 0;
        targetCP = 0;
        targetPhi = 0;
        state = 5;
      }
      break;

      case 3: // Turn Left
      targetCP = 0.1;
      if (fabs(errorPhi) < 0.1)// IF turn is finnished
      {
        Serial.println("Entering FINISHED state");
        state = 5;
      }
      break;

      case 4: // SEARCH MODE
        if (newPhiCorrectionFlag)
        {
            //Serial.println("B");
            if (phi_correction != 0){
              phi=0;
              if(fabs(targetPhi- phi_correction) >.02)
              targetPhi = phi_correction;
            }
            phi_correction = 0; // Reset phi_correction to stop infinite integration
        }
        else
        {
          targetPhi = phi + 0.6; //Carrot on a stick
        }

        
        // IF search is finnished
        if (fabs(targetPhi - phi) < 0.02)
        {
         
          newPhiCorrectionFlag = false;
          
          Serial.println("Entering FORWARD state");
          state = 1;
          RIGHTWHEELCOUNTS = 0;
          LEFTWHEELCOUNTS = 0;
          newPhiCorrectionFlag = false;
          int countR = 0;
          int countL = 0;
          int countN = 0;
          
          targetCP = 1; //Run with three feet forward
          targetPhi = -.2;
          
          newforwardCommandFlag = false;
            
        }
      break;

      case 5:
      //Serial.println("State is finished");
      digitalWrite(safteyPin, LOW);
      break;
      
      default:
      Serial.print("State machine is broken. State:");
      Serial.println(state);
      break;
      
    }

   ////////////////////////////// LOCALIZATION /////////////////////////////
    
    leftWheelPos = (float)LEFTWHEELCOUNTS/1600*6.28;
    rightWheelPos = (float)RIGHTWHEELCOUNTS/1600*6.28;

    leftDistance = leftWheelPos * wheelRadius;
    rightDistance = rightWheelPos * wheelRadius;
    
    leftWheelSpeed = (leftWheelPos - leftWheelLastPos)/0.02;
    rightWheelSpeed = (rightWheelPos - rightWheelLastPos)/0.02;

    leftTangentalVelocity = leftWheelSpeed * wheelRadius;
    rightTangentalVelocity = rightWheelSpeed * wheelRadius;
    
    centerDistance = (leftDistance + rightDistance) / 2.0;
    centerSpeed = (leftTangentalVelocity + rightTangentalVelocity) / 2.0;
    
    phi += (rightTangentalVelocity - leftTangentalVelocity) * 0.02 / clankerWidth / 2;
    
    //update x and y position based upon phi value
//    xPos += centerSpeed * cos(phi) * 0.02;
//    yPos += centerSpeed * sin(phi) * 0.02;

    
    
  
    

    //////////////////////////////////// MESSAGE DECODING ///////////////////////////////////
    if (MESSAGE_LENGTH > 0)
    {
//      Serial.println("NEW MESSAGE");
      for (int i = 0; i < MESSAGE_LENGTH - 1; i++)
        Serial.print(MESSAGE[i]);
      Serial.println("");
//      Serial.println("END MESSAGE");
//      Serial.print("LENGTH : ");
//      Serial.println(MESSAGE_LENGTH);
      char mode = '\0';
      float value = 0;
      //Decode the message
      for (int i = 0; i < MESSAGE_LENGTH - 1; i++)
      {
        switch (MESSAGE[i])
        {
          case 'f':
            mode = 'f';
            break;
          case 'r':
            mode = 'r';
            break;
          case 'l':
            mode = 'l';
            newPhiCorrectionFlag = true;
            break;
          case 'L':
            if (command == '\0')
              command = 'L';
            break;
          case 'R':
            if (command == '\0')
              command = 'R';
            break;
          case 'S':
            if (command == '\0' && hasSearched == false)
              command = 'S';
              hasSearched = true;
            break;
          case 'N':
            if (command == '\0')
              command = 'N';
            break;
          case '0':
            value = value*10 + 0;
            break;
          case '1':
            value = value*10 + 1;
            break;
          case '2':
            value = value*10 + 2;
            break;
          case '3':
            value = value*10 + 3;
            break;
          case '4':
            value = value*10 + 4;
            break;
          case '5':
            value = value*10 + 5;
            break;
          case '6':
            value = value*10 + 6;
            break;
          case '7':
            value = value*10 + 7;
            break;
          case '8':
            value = value*10 + 8;
            break;
          case '9':
            value = value*10 + 9;
            break;
          case '\n':  //dont print unknown char for new line
          break;
          case '\r':  //dont print unknown char for carrage return
          break;
          case ',':   //Case for if two messages in one line
            if (mode == 'f')
            {
              if (value != 0)
              {
                forward_command_ammount = (value) * 0.305 / 12;
                newforwardCommandFlag = true;
              }
              value = 0;
            }
            else if (mode == 'r')
            {
              if (value != 0)
              {
                phi_correction =-1 * value * 0.01745;
                newPhiCorrectionFlag = true;
              }
              value = 0;
            }
            else if (mode == 'l')
            {
              if (value != 0)
              {
                phi_correction = value * 0.01745;
                newPhiCorrectionFlag = true;
              }
              value = 0;
            }
            mode= '\0';
            break;
        }
      }
      if (mode == 'f')
      {
           forward_command_ammount = (value) * 0.305 / 12;
           newforwardCommandFlag = true;
      }
      else if (mode == 'r')
      {
      if (value != 0)
      {
        phi_correction = -1* value * 0.01745;
        newPhiCorrectionFlag = true;
      }
    }
      else if (mode == 'l'){
        if (value != 0)
        {
            phi_correction = value * 0.01745;
            newPhiCorrectionFlag = true;
        }
      }
      else
      {
        Serial.print("INVALID MODE : ");
        Serial.println(mode);
      }
      MESSAGE_LENGTH = 0;
    }   


    ////////////////////////// CONTROL ////////////////////////////////
    
    errorPhi = targetPhi - phi;
    errorCP = targetCP - centerDistance;


    centerIntegral = centerIntegral*0.95 + errorCP;
    phiIntegral = phiIntegral*0.95 + errorPhi;


    targetRSpeed = 0;
    targetLSpeed = 0;

    // Position to wheel speed
    targetRSpeed = errorCP * centerKp + centerIntegral * centerKi;
    targetLSpeed = errorCP * centerKp + centerIntegral * centerKi;
      
    // Phi to wheel speed
    targetRSpeed += errorPhi * phiKp + phiIntegral * phiKi;
    targetLSpeed -= errorPhi * phiKp + phiIntegral * phiKi;



    float errorRV = targetRSpeed - rightWheelSpeed;
    float errorLV = targetLSpeed - leftWheelSpeed;

    //Limit robot speed
    if (errorRV > maxWheelSpeed && errorRV > errorLV)  
    {
      errorLV = errorLV/errorRV * maxWheelSpeed;
      errorRV = maxWheelSpeed;
    }
    if (errorLV > maxWheelSpeed && errorLV > errorRV)  
    {
      errorRV = errorRV/errorLV * maxWheelSpeed;
      errorLV = maxWheelSpeed;
    }

    //Apply low-pass filter to outputs to reduce gittering
    rightPWM = rightPWM*0.95 + (errorRV * rightKp)*0.05;
    leftPWM =  leftPWM *0.95 + (errorLV * leftKp)*0.05;


    Serial.print(targetPhi * 57.2958);
    Serial.print(" ");
    Serial.print(targetCP);
    Serial.print(" ");
    if (state == 0)
      Serial.print("DO NOTHING");
    else if (state == 1)
      Serial.print("FORWARD");
    else if (state == 2)
      Serial.print("RIGHT");
    else if (state == 3)
      Serial.print("LEFT");
    else if (state == 4)
     Serial.print("SEARCH");
    else if (state == 5)
     Serial.print("FINNISH");
   Serial.print(" ");
    Serial.print(phi_correction);
    Serial.print(" ");
    Serial.println(command);


   /* Serial.print(phi * 57.2958, 5);
    Serial.print(" ");
    Serial.print(targetPhi * 57.2958, 5);
    Serial.print(" ");
    Serial.print(centerDistance * 39.3701, 5);
    Serial.print(" ");
    Serial.println(targetCP * 39.3701, 5);
    */

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

//    if (L < 10) L = 0;
//    if (R < 10) R = 0;

    
    analogWrite(leftPWMpin, L);
    analogWrite(rightPWMpin, R);
}

void receive()
{
  Wire.read();
  int i = 0;
  while(Wire.available())
  {
    MESSAGE[i] = Wire.read();
    i++;
    if (i > 31)
      break;
  }
  MESSAGE_LENGTH = i + 1;
}


void receiveFromSerial()
{
  int i = 0;
  while(Serial.available())
  {
   MESSAGE[i] = Serial.read();
   i++;
   if (i > 30)
     break;
  }
  
  MESSAGE_LENGTH = i + 1;
}


/*
 *        MESSAGE = "f#########"  THIS WILL GO FORWARD ####### INCHES
 *        
 *        MESSAGE = "r#########"  THIS TURN ####### DEGREES RIGHT
 * 
 *        MESSAGE = "l#########"  THIS TURN ####### DEGREES LEFT
 *        
 *        MESSAGE = "R"  RIGHT ARROW
 * 
 *        MESSAGE = "L"  LEFT ARROW
 *        
 *        MESSAGE = "S" THIS WILL ENTER SEARCH MODE
 *        
 *        MESSAGE = "N" NO MARKER
 *        
 *    
 *        10
 *        10
 * 
 */
