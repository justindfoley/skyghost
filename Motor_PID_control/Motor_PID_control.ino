#include <Metro.h>

// ================== Start PID include code ================== //
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
double aggKp=0.09, aggKi=0.2, aggKd=0.001;
double consKp=0.09, consKi=0.2, consKd=0.001;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
// ================== End PID include code ================== //

// Set speed
int desiredSpeed = 3000; // Desired speed in pulses per second

// Define pin assignments
#define encoder0PinA  2 // Encoder sensor feedback A
#define encoder0PinB  3 // Encoder sensor feedback B
int dir1PinA = 4; // Motor direction 1
int dir2PinA = 5; // Motor direction 2
int speedPinA = 9; //Motor speed PWM

// Loop time in ms
int loopTime = 100;

// Initialize variables
int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;
volatile long encoder0Pos = 0;
volatile long unknownvalue = 0;
long encoder0Previous = 0;
int encoder0Speed = 0;
int motorPWM = 0; // Initial speed PWM setting

// Serial input
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Set up loop timing function
Metro mainTimer = Metro(loopTime);

void setup() { 

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // ================== Start PID setup code ================== //
  //initialize the variables we're linked to
  Input = encoder0Speed;
  Setpoint = desiredSpeed;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  // ================== End PID setup code ================== //


  // Configure pins
  pinMode(encoder0PinA, INPUT); // Set encoder pin A as input
  digitalWrite(encoder0PinA, HIGH); // Turn on encoder pin A pullup resistor
  pinMode(encoder0PinB, INPUT); // Set encoder pin B as input
  digitalWrite(encoder0PinB, HIGH); // Turn on encoder pin B pullup resistor
  attachInterrupt(0, doEncoder, CHANGE); // Attach interrupt 0
  attachInterrupt(1, doEncoder, CHANGE); // Attach interrupt 1
  pinMode(dir1PinA,OUTPUT); // Set motor direction 1 pin as output
  pinMode(dir2PinA,OUTPUT); // Set motor direction 2 pin as output
  pinMode(speedPinA,OUTPUT); // Set motor speed PWM pin as output

  // Configure serial
  Serial.begin (115200);
  Serial.println("start");                // a personal quirk

} 

void loop(){

  if (mainTimer.check() == true) {
    Serial.print("Setpoint: ");
    Serial.print(Setpoint, DEC);
    Serial.print("     Speed: ");
    Serial.print(encoder0Speed, DEC);
    Serial.print("     PWM: ");
    Serial.print(motorPWM, DEC);
    Serial.println("");

    // Take speed input from serial
    //    while (Serial.available() > 0) {
    //      Setpoint = Serial.parseInt(); 
    //      if (Serial.read() == '\n') {
    //      }
    //    }

    if (stringComplete) {
      Setpoint = (double)inputString.toFloat();
      inputString = "";    // clear the string:
      stringComplete = false;
    }

    // Calculate current speed in pulses per second
    encoder0Speed = (encoder0Pos - encoder0Previous) * 1000 / loopTime;

    // Set encoder value for next loop iteration
    encoder0Previous = encoder0Pos;

    // ================== Start PID loop code ================== //
    Input = encoder0Speed;

    double gap = abs(Setpoint-Input); //distance away from setpoint
    if(gap<500)
    {  //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    myPID.Compute();
    motorPWM = Output;
    // ================== End PID loop code ================== //

    // Set motor speed and direction for testing
    analogWrite(speedPinA, motorPWM);
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, HIGH);
  }
}

void doEncoder() {
  // Interrupt encoder tracking function
  pinAState = digitalRead(encoder0PinA);
  pinBState = digitalRead(encoder0PinB);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      encoder0Pos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      encoder0Pos --;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
      unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
      unknownvalue ++;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      encoder0Pos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      encoder0Pos --;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
      unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
      unknownvalue ++;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      encoder0Pos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      encoder0Pos --;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
      unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
      unknownvalue ++;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      encoder0Pos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      encoder0Pos --;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
      unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
      unknownvalue ++;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}








