/* This code is responsible for determining the position of a robot with 2 motors on a Cartesian plane.
 * The code also determines orientation based upon angle formed with the x-axis of the robot. 
 * The wheels of the robot will need to be moved manually, and through this movement, an output will be printed to the serial monitor showing the time, x, y and phi values, in order to be used in tandem with MATLAB or copied into a text file and then uploaded afterwards.
 * The results are then animated into a MATLAB graph showcasing the changing orientation and position of the bot. The pins defined in the setup will need to be connected properly to each motor's encoder, and the correct pins are labeled already for easy adoption of the code.
 */

#define rightWheelEncoderA 2
#define rightWheelEncoderB 6
#define leftWheelEncoderA 3
#define leftWheelEncoderB 5
#define wheelRadius 0.06
#define clankerWidth 0.15

// GLOBAL VARS
// encoder A values initialization
bool LAST_R_A;
bool LAST_L_A;
// encoder counts value initialization
int RIGHTWHEELCOUNTS = 0;
int LEFTWHEELCOUNTS = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(6, INPUT);
  pinMode(5, INPUT);
  // Initial encoder values
  bool LAST_R_A = digitalRead(rightWheelEncoderA);
  bool LAST_L_A = digitalRead(leftWheelEncoderA);
  // attach interrupts to the encoder pins to run their specific ISR functions when needed
  attachInterrupt(digitalPinToInterrupt(rightWheelEncoderA), rightEncoderInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoderA), leftEncoderInterupt, CHANGE);
}

void loop() {
  long currentTime = millis();
  long lastTime = 0;
  // wheel rotational position
  float leftWheelPos = 0;
  float rightWheelPos = 0;
  float leftWheelLastPos = 0;
  float rightWheelLastPos = 0;
  // wheel position
  float leftDistance = 0;
  float rightDistance = 0;
  float centerDistance = 0;
  // robot orientation
  float phi = 0;
  float xPos = 0;
  float yPos = 0;
  
  while (true)
  {
    // Wait for next loop
    while(millis() < currentTime){}
    // get values for left and right wheel rotational position
    leftWheelPos = (float)LEFTWHEELCOUNTS / 1600 * 6.28;
    rightWheelPos = (float)RIGHTWHEELCOUNTS / 1600 * 6.28;
    // use change in rotational position to determine distance traveled
    leftDistance = leftWheelPos * wheelRadius;
    rightDistance = rightWheelPos * wheelRadius;
    // using distance traveled by each wheel, get distance traveled by center of robot
    centerDistance = (leftDistance + rightDistance) / 2.0;
    // using distance change for each wheel, determine orientation of robot
    phi = (rightDistance - leftDistance) / clankerWidth;
    // keep phi within bounds of (0, 2pi)
    phi = fmod(phi, 2.0 * PI);
    if (phi < 0) phi += 2.0 * PI;
    // update x and y position based upon phi value
    xPos = centerDistance * cos(phi);
    yPos = centerDistance * sin(phi);
    // print out time, x, y, and phi values
    Serial.print(currentTime);
    Serial.print(" ");
    Serial.print(xPos, 5);
    Serial.print(" ");
    Serial.print(yPos, 5);
    Serial.print(" ");
    Serial.println(phi, 5);
    // Update position and time values
    leftWheelLastPos = leftWheelPos;
    rightWheelLastPos = rightWheelPos;
    lastTime = currentTime;
    currentTime = lastTime + 20;
  }
}

void rightEncoderInterupt()
{
  // ENCODER SECTION
  bool A = digitalRead(rightWheelEncoderA);
  bool B = digitalRead(rightWheelEncoderB);
  // increment count based upon how encoder values changed between current and last interrupt 
  if (A != LAST_R_A)
    if(B != A)
    {
      RIGHTWHEELCOUNTS++;
    }
    else
    {
      RIGHTWHEELCOUNTS--;
    }
  // rewrite last values to the current encoder values before exiting function
  LAST_R_A = A;
}

void leftEncoderInterupt()
{
  // ENCODER SECTION
  bool A = digitalRead(leftWheelEncoderA);
  bool B = digitalRead(leftWheelEncoderB);
  // increment count based upon how encoder values changed between current and last interrupt 
  if (A != LAST_L_A)
    if(B != A)
    {
      LEFTWHEELCOUNTS--;
    }
    else
    {
      LEFTWHEELCOUNTS++;
    }
  // rewrite last values to the current encoder values before exiting function
  LAST_L_A = A;
}
