#include <Servo.h>
Servo carServo;
// Things that we can change

// Parking
//---------------------------------------------
int INITIALIMPULSES = 10; // Distance befroe the parking algorithm starts
int PARK_NEEDED_DISTANCE_IMPULSES = 11;// Min Distance needed for parking between two obstacles
int PARKSCANNINGSPEED = 15; // Speed with which we are scanning for Parking space
int MANEVRASPEED = 15; // Speed at which the Park instructions execute
int MANEVRA_0_IMP = 2;//forward left
int MANEVRA_1_IMP = 4 ;// Distance for each car movement in manevra 1 1IMP = 6,3cm
int MANEVRA_2_IMP = 11 ; // Distance for each car movement in manevera 2 1IMP = 6,3cm
int PARKINGDELAY = 3000; // Waiting time in parked position
bool parkingFinished; // True when we have parked the car
int prevDist[2];
int prevFrontDist[2];
bool firstBoxFound = false;
// ---------------------------------------------

//Avoidance
//----------------------------------------------
bool objectInfront = false;

// Debugging
//---------------------------------------------
bool JEVOISSERVO = false; //  print correction step and panval from followJevois()
bool PrintDistanceToSideBox = true; // print the distance to side boxes
bool EncoderPrint = true; // print encoder count
// ---------------------------------------------

// Jevois Init
//---------------------------------------------
// How precise we calculate panval (10 is to the 0.n, 100 is to 0.0n etc.)
#define SCALE 100
// What is center
#define PANZERO 75
// What is max left
#define PANMIN 35
// What is max right
#define PANMAX 120
// What is center while searching for parking space
#define PARKSCANNINGPANZERO 75
#define TARGXCHANGE 0// was 20 - change the center postion of jevoiss by ... 
int INITSPEED = 15; // Speed for Jevois start
int speed;
int lastSpeed = 0;
int direction = 0;
int position = 80;
long pangain = 1000;
long panval = PANZERO * SCALE;

// ---------------------------------------------

// Serial ports
//---------------------------------------------
#define SERIAL Serial1
#define SERIAL_USB Serial
// ---------------------------------------------

// Pins
//---------------------------------------------
int switchPin = 3; // choose the input pin (Encoder)
const int SERVO_PIN = 7; // Servo pin
const int speedPin = 5; // Engine pin
const int directionPin = 24; // Direction of engine pin
const int IRpin = A3; // Pin of the 4-30cm Side IR sensor
const int frontIRpin = A2; // Pin of the 10-80cm Front IR sensor
const int ch1Pin = 9; //Pin of RC (left-right)
const int ch2Pin = 10; //Pin of RC(backward-forward)
const int flagPin = 30; //Pin of RC blue light
const int blinkerRightPin =  46; // PIN of right blinkers
const int blinkerLeftPin = 28;//PIN of left blinker
const int stopLightPin = 32; //PIN of stop red lights
const int button1Pin = 48;     // the number of the pushbutton pin
const int button2Pin = 50;     // the number of the pushbutton pin

// ---------------------------------------------

// Initial servo values in degrees
//---------------------------------------------
int val = 0; // variable for reading the pin status
int currentState = 0;
int previousState = 0;
//---------------------------------------------

// Encoder
//---------------------------------------------
int startImpulses = 0;
int distanceFromStart = 0;
int encoderCounter = 0;
//---------------------------------------------

//Remote Control
//---------------------------------------------
int INITIAL_RC_DELAY = 1000; // Wait 1s before you take manual control over car
unsigned long ch1; //left-right
unsigned long ch2; //backward-forward
bool flagRC = false; //not in RC mode; true = RC mode
int rcState = LOW;
int countRCMode = 0; //to exit RCMode -> countRCMode > 3000
//--------------------------------------------

// Buffer for received serial port Jevois bytes
//---------------------------------------------
#define INLEN 128
char instr[INLEN + 1];
//---------------------------------------------


//lights
//----------------------------------------------
bool blinkerRightFlag = false; //flag for right blinker
bool blinkerLeftFlag = false;  //flag for left blinker

int blinkerRightState = LOW;   // ledState of right blinker
int blinkerLeftState = LOW;   //led state of left blinker

unsigned long previousMillisBlinkers = 0;  // will store last time LED was updated
unsigned long previousMillisRC = 0;
const long BLINKERSINTERVAL = 250;    // interval at which to blink (milliseconds) for blinkers = 2Hz
const long RCINTERVAL = 500; //interval at which blue light blinks = 1Hz
//----------------------------------------------

//buttons
//-----------------------------------------------
int button1State = 0;         // variable for reading the pushbutton status
int button2State = 0;         // variable for reading the pushbutton status
bool button1Flag = false;
bool button2Flag = false;

void setup()
{
  position = 75;
  speed = 0;
  carServo.attach(SERVO_PIN);
  pinMode(IRpin, INPUT);
  pinMode(frontIRpin, INPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  servo(panval / SCALE);
  SetupJevoisCamera();
  startImpulses = encoderImpulses(); // Record impulses from the start
  SetupRemoteControl();
  SetupLights();
  SetupButtons();
}

void SetupRemoteControl()
{
  pinMode(flagPin, OUTPUT); //blue light for remote control
  digitalWrite(flagPin, rcState);
  pinMode(ch1Pin, INPUT); //remote control
  pinMode(ch2Pin, INPUT); //remote control
}

void SetupJevoisCamera()
{
  SERIAL_USB.begin (9600);
  SERIAL.begin(115200);
  SERIAL.setTimeout(1000000);
  // We are ready to rock
  SERIAL.println("setpar serlog None");
  SERIAL.println("setpar serout Hard");
}

void SetupLights()
{
  pinMode(blinkerRightPin, OUTPUT);
  pinMode(blinkerLeftPin, OUTPUT);
  pinMode(stopLightPin, OUTPUT);
}

void SetupButtons()
{
  // initialize the pushbutton pin as an input:
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  // read the state of the pushbutton value:
  waitForButtons();
}

void waitForButtons() {
  do {
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);
    delay(100);
  } while (button1State == LOW && button2State == LOW);
  button1Flag = (button1State == HIGH);
  button2Flag = (button2State == HIGH);
  if (button1Flag)
  {
    Serial.println("Button 1");
  }
  if (button2Flag)
  {
    Serial.println("Button 2");
  }
}

void enableBlinker(bool left, bool right)
{
  blinkerRightFlag = right;
  blinkerLeftFlag = left;

  digitalWrite(blinkerLeftPin, left ? HIGH : LOW);
  digitalWrite(blinkerRightPin, right ? HIGH : LOW);
}

void enableRCLight(bool rc)
{
  bool init_Delay = rc && (rc != flagRC);
  bool manualFinish = !rc&& (rc!=flagRC);
  flagRC = rc;
  previousMillisRC= millis();
  if (init_Delay)
  {
    rcState = HIGH;
    //Serial.print("manual control first, rcState=");
    //Serial.println(rcState);
    digitalWrite(flagPin, rcState);
    motor (0,0);
    delay (INITIAL_RC_DELAY);
  }
  if (manualFinish)
  {
    rcState = LOW;
    //Serial.print("manual control first, rcState=");
    //Serial.println(rcState);
    digitalWrite(flagPin, rcState);
    motor (0,0);
    delay (INITIAL_RC_DELAY);
  }
}

bool calculateBlinkingInterval()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisBlinkers >= BLINKERSINTERVAL)
  {
    // save the last time you blinked the LED
    previousMillisBlinkers = currentMillis;
    return true;
  }
  else
  {
    return false;
  }
}

void checkBlinkers()
{
  if (calculateBlinkingInterval())
  {
    if (blinkerRightFlag)
    {
      changeRightBlinker();
    }
    if (blinkerLeftFlag)
    {
      changeLeftBlinker();
    }
  }
  if (flagRC)
  {
    if (calculateRCInterval())
    {
      changeBlueLight();
    }
  }
}

void changeRightBlinker()
{
  // if the LED is off turn it on and vice-versa:
  if (blinkerRightState == LOW)
  {
    blinkerRightState = HIGH;
  }
  else
  {
    blinkerRightState = LOW;
  }
  // set the LED with the ledState of the variable:
  digitalWrite(blinkerRightPin, blinkerRightState);
}

void changeLeftBlinker()
{
  // if the LED is off turn it on and vice-versa:
  if (blinkerLeftState == LOW)
  {
    blinkerLeftState = HIGH;
  }
  else
  {
    blinkerLeftState = LOW;
  }
  // set the LED with the ledState of the variable:
  digitalWrite(blinkerLeftPin, blinkerLeftState);
}

void changeBlueLight()
{
  // if the LED is off turn it on and vice-versa:
  if (rcState == LOW)
  {
    rcState = HIGH;
  }
  else
  {
    rcState = LOW;
  }
  // set the LED with the ledState of the variable:
  digitalWrite(flagPin, rcState);
}

bool calculateRCInterval()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisRC >= RCINTERVAL)
  {
    // save the last time you blinked the LED
    previousMillisRC = currentMillis;
    return true;
  }
  else
  {
    return false;
  }
}

void motor(int direction, int speed)
{
  if (speed < lastSpeed)
  {
    //brake
    digitalWrite(stopLightPin, HIGH);

  } else
  {
    digitalWrite(stopLightPin, LOW);
  }
  lastSpeed = speed;
  checkBlinkers();
  digitalWrite(directionPin, direction); //0=forward, 1=back
  analogWrite(speedPin, speed); // min= 1; max= 255
}

void servo (int value)
{
  checkBlinkers();
  carServo.write(value);
}


int encoderImpulses()
{
  checkBlinkers();
  //encoder
  val = digitalRead(switchPin); // read input value
  if (val == HIGH) { // check if the input is HIGH (button released)
    currentState = 1;
  }
  else
  {
    currentState = 0;
  }
  if (currentState != previousState)
  {
    if (currentState == 1)
    {
      encoderCounter = encoderCounter + 1;
      if (EncoderPrint) {
        Serial.print("Counter encoder ");
        Serial.println(encoderCounter);
      }
    }
  }
  previousState = currentState;
  delay(10);
  return encoderCounter;

}

void manevra(int direct, int impulse, int rotation) {
  direction = direct;
  speed = MANEVRASPEED;
  Serial.println("Rotate Maximum Left");
  position = 45;
  int manevraStart = encoderImpulses();
  int distanceCounter = 0;
  servo(rotation);
  motor(direction, speed);
  do
  {
    delay(20);
    encoderCounter = encoderImpulses();
    distanceCounter = encoderCounter - manevraStart;
    //    Serial.print("manevra distance counter = ");
    //    Serial.println(distanceCounter);
  } while (distanceCounter < impulse);

  Serial.println("Left Done!");
}

void AvoidanceManevri()
{
  Serial.println("Avoidance Start");
  enableBlinker(true, false);
  motor(0, 0);
  delay(500);
  manevra(0, 8, 25);
  manevra(0, 6, 125);
  enableBlinker(false, false);
  Serial.println("Overtaking start");
  manevra(0, 5, 75);
  bool objectOnRight = false;
  do
  {
    servo(followJevois(false, INITSPEED));
    objectOnRight = IRRightDistance();
    delay(100);//TODO check if there is a turn, maybe to followJevois
  } while (objectOnRight);
  Serial.println("Overtaking Done");
  enableBlinker(false, true);
  manevra(0, 8, 125);
  manevra(0, 6, 25);
  enableBlinker(false, false);
  servo(75);
  Serial.println("Avoidance complete");
  motor(0, INITSPEED);
}

bool frontIRDistance()
{

  int volts = analogRead(frontIRpin);
  int frontDistance = (2914 / (volts + 5)) - 2; //Sharp GP2D120 4-30CM
  //int frontDistance =  4800 / (volts - 20);
  //int frontDistance = (6787 / (volts - 3)) - 4; //Sharp GP2D12 10-80CM
  
  if (frontDistance > 4 && frontDistance < 40)
  {
    int delta = prevFrontDist[0] - frontDistance;
    if (delta<0) {delta = -delta;}
    if (delta<3)
    {
      Serial.print("Front object. Distance ");
      Serial.println(frontDistance);
      objectInfront = true;
    }
    prevFrontDist[1] = prevFrontDist[0];
    prevFrontDist[0] = frontDistance;
  }
  else
  { //no object
    //    Serial.println("No object");
    objectInfront = false;
  }
  return objectInfront;
}

bool IRRightDistance()
{
  bool objectOnRightSide = true;

  int volts = analogRead(IRpin);
  int distance = (2914 / (volts + 5)) - 2; //Sharp GP2D120 4-30CM
  //int distance = (6787 / (volts - 3)) - 4; //Sharp GP2D12 10-80CM
  //int distance =  4800 / (volts - 20);
  //Serial.print("Box found at Encoder count");
  //Serial.print(encoderCounter);
  // Serial.print("Distance to object= ");
  // Serial.println(distance);
  //if (true) {
  //return;//todo test only
  // }
  if (distance > 4 && distance < 30)
  {
    //if (prevDist[0] == distance && prevDist[1] == distance)
    //    if (prevDist[0] == distance)
    //    {
    if (PrintDistanceToSideBox) // Debugging
    {
      Serial.print("Right Distance to object= ");
      Serial.println(distance);
      objectOnRightSide = true;
    }
    //    }
    prevDist[1] = prevDist[0];
    prevDist[0] = distance;
  }
  else
  { //no object
    if (PrintDistanceToSideBox)
    {
      Serial.println("No object");
    }
    objectOnRightSide = false;
  }
  return objectOnRightSide;
}
void parkingManevri()
{
  motor(0, 0);
  enableBlinker(true, true);
  //servo(75);
  manevra (1, 2, 75);
  manevra (0, MANEVRA_0_IMP, 45);
  motor(0, 0);
  manevra (1, MANEVRA_1_IMP, 135);
  manevra (1, MANEVRA_2_IMP, 45);
  servo(75);
  motor(0, 0);
  Serial.println("Parking Complete");
  enableBlinker(false, false);
  delay(PARKINGDELAY);
  enableBlinker(true, false);
  manevra(0, MANEVRA_2_IMP, 45);
  manevra(0, MANEVRA_1_IMP + 1, 135);
  parkingFinished = true;
  enableBlinker(false, false);
  Serial.println("Car returned to Initial Start Location");
}



void IRDistance () {
  int volts = analogRead(IRpin);
  //int distance = (2914 / (volts + 5)) - 2; //Sharp GP2D120 4-30CM
  //int distance = (6787 / (volts - 3)) - 4; //Sharp GP2D12 10-80CM
  int distance =  4800 / (volts - 20);
  //Serial.print("Box found at Encoder count");
  //Serial.print(encoderCounter);
  // Serial.print("Distance to object= ");
  // Serial.println(distance);
  //if (true) {
  //return;//todo test only
  // }
  if (distance > 9 && distance < 100)
  {
    //if (prevDist[0] == distance && prevDist[1] == distance)
    if (prevDist[0] == distance)
    {
      if (PrintDistanceToSideBox) // Debugging
      {
        Serial.print("Box found at Encoder count");
        Serial.print(encoderCounter);
        Serial.print("Distance to object= ");
        Serial.println(distance);
      }
      if (!firstBoxFound) {
        firstBoxFound = true;
        encoderCounter = 0;
      } else {
        if (encoderCounter >= PARK_NEEDED_DISTANCE_IMPULSES)
        {
          motor(0, 0);
          parkingManevri();
        }
        else
        {
          encoderCounter = 0;
        }
      }
      //firstbox
    }
    prevDist[1] = prevDist[0];
    prevDist[0] = distance;
  }
  else
  { //no object
    servo(followJevois(true, PARKSCANNINGSPEED));
    if (PrintDistanceToSideBox)
    {
      Serial.println("No object");
    }
  }
}

void parkScanning() {
  encoderImpulses();
  servo(followJevois(true, PARKSCANNINGSPEED));
  if (!parkingFinished) {
    IRDistance();
  }
}

bool remoteControl()
{
  ch2 = pulseIn(ch2Pin, HIGH);
  //Serial.println(ch2);
  if (ch2 > 1520 && ch2 < 2000) //backwards
  {
    enableRCLight(true);
    countRCMode = 0;
    Serial.print(" Backwards ");
    Serial.println(ch2);
    direction = 1;
    speed = map(ch2, 1520, 1950, 10, 70);
    motor(direction, speed/2);
  }
  else if (ch2 < 1350 && ch2 > 0) // forward
  {
    enableRCLight(true);
    countRCMode = 0;
    Serial.print(" Forward ");
    Serial.println(ch2);
    direction = 0;
    speed = map(ch2, 1100, 1350, 70, 10);
    motor(direction, speed/2);

  }

  ch1 = pulseIn(ch1Pin, HIGH);

  if (ch1 < 1350 && ch1 > 0)
  {
    enableRCLight(true);
    countRCMode = 0;
    Serial.print(" Left");
    Serial.println(ch1);
    position = map(ch1, 1100, 1399, 30, 85);
    servo(position);

  }
  else if (ch1 > 1520)
  {
    enableRCLight(true);
    countRCMode = 0;
    Serial.print(" Right");
    Serial.println(ch1);
    position = map(ch1, 1520, 2000, 85, 150);
    servo(position);
  }
  else
  {
  servo (75);
  }

  //speed = 0;
  //motor(direction, speed);

  countRCMode += 50;
  

  if (countRCMode > 3000)
  {
    //Serial.println(countRCMode);
    countRCMode = 0;
    enableRCLight(false);
  }

  return flagRC;
}

int followJevois (bool Scanning, int followSpeed) {
  motor(0, followSpeed);
  //  digitalWrite(LEDPIN, LOW);
  byte len = SERIAL.readBytesUntil('\n', instr, INLEN);
  instr[len] = 0;
  //  digitalWrite(LEDPIN, HIGH);
  char * tok = strtok(instr, " \r\n");
  int state = 0;
  int targx = 0, targy = 0;
  while (tok)
  {
    switch (state)
    {
      case 0:                                            // 0: start parsing
        if (strcmp(tok, "T2") == 0) state = 1;           // 1: T2 command, parse targx
        else if (strcmp(tok, "T1") == 0) state = 4;      // 4: T1 command, parse targx
        else if (strcmp(tok, "PANGAIN") == 0) state = 6; // 6: PANGAIN command, parse pangain

        else state = 1000;                           // 1000: unknown command
        break;

      case 1: targx = atoi(tok); state = 2; break;   // 2: T2 command, parse targy
      case 2: targy = atoi(tok); state = 3; break;   // 3: T2 command complete
      case 4: targx = atoi(tok); state = 5; break;   // 5: T1 command complete
      case 6: pangain = atoi(tok); state = 7; break; // 7: PANGAIN command complete
      default: break; // Skip any additional tokens
    }
    tok = strtok(0, " \r\n");
  }
  if (state == 3 || state == 5)
  {
    if (Scanning)
    {
      targx = targx + TARGXCHANGE;
    }
    if (targx == 0)
    {
      panval = PANZERO * SCALE;
    }
    else
    {
      int correctionStep = (targx * pangain) / 1000;//correction step size
      panval = map(correctionStep, -1500, 1500, PANMIN * SCALE, PANMAX * SCALE);
      if (JEVOISSERVO)
      {
        SERIAL_USB.print("correctionStep=");
        SERIAL_USB.print(correctionStep);
        SERIAL_USB.print(" -> servo=");
        SERIAL_USB.println(panval);
      }
    }
    if (panval < PANMIN * SCALE) panval = PANMIN * SCALE; else if (panval > PANMAX * SCALE) panval = PANMAX * SCALE;
    //servo(panval / SCALE);
    return panval / SCALE;
  }
}

void loopAvoidFrontObjects() {
  if (!remoteControl())
  {
    if (frontIRDistance()) {
      AvoidanceManevri();
    }
    //    delay(300);//TODO remove
    servo(followJevois(false, INITSPEED));
  }
}

void loopParkingRound()
{

  if (!remoteControl())
  {
    distanceFromStart = encoderImpulses() - startImpulses;
    if (distanceFromStart < INITIALIMPULSES) {
      servo(followJevois(false, INITSPEED));
    }
    else
    {
      if (!parkingFinished)
      {
        parkScanning();
      }
      else
      {
        servo(followJevois(false, 25));
      }
    }
  }
}

void loop()
{
//servo(followJevois(false,0));
  //enableBlinker(true,true);
  if (button1Flag) {
    loopParkingRound();
  }
  if (button2Flag) {
    loopAvoidFrontObjects();
  }

}
