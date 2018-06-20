#include <Arduino.h>
#include <ClearPathMotorSD.h>
#include <ClearPathStepGen.h>

#include "Board.h"

#include "DMXInterface.h"

using namespace Foldhaus;

constexpr int VariablePin       = A3;

constexpr int RS485RxEnable     = 4; // PD4

constexpr int PinSpot = 3; // PD3

// ClearPath ENABLE;
constexpr int EnablePin         = 10; // PB2
// ClearPath Input A (DIRECTION);
constexpr int DirectionPin      = 9; // PB1
// ClearPath Input B (STEP);
constexpr int StepPin           = 11; // PB3

constexpr int Feedback          = A5; // PC5

constexpr int maxRPM            = 4000;
constexpr int maxAccel          = 8000;
constexpr int maxTravelInches   = 28;
constexpr int countsPerRotation = 200; //must be set to this in the Clearpath firmware
constexpr int rotationsPerInch  = 4;  //must be set per lead screw pitch
constexpr int maxCounts         = maxTravelInches * countsPerRotation * rotationsPerInch;

int Enabled                               = false;
int currentCount                          = 0;
int requestedCount                        = 0;
int dmxRequestedPosition                  = 0;

// const char endl[] PROGMEM = "\r\n";
constexpr char endl = '\n';

ClearPathMotorSD X;

//initialize the controller and pass the reference to the motor we are controlling
ClearPathStepGen machine(&X);


void setup()
{
  // Initialize blip
  DebugPin::on();
  DebugPin::off();
  
  pinMode(PinSpot, OUTPUT);
  digitalWrite(PinSpot, LOW);
  
  pinMode(RS485RxEnable, OUTPUT);
  digitalWrite(RS485RxEnable, LOW);
  
  DMXInterface::init();

  DMXInterface::debug << PSTR("Setup") << endl;
  
  X.attach(DirectionPin, StepPin, EnablePin, Feedback);          //Direction/A is pin 8, Step/B is pin 9, Enable is pin 6, HLFB is pin 4

  // Set max Velocity.  Parameter can be between 2 and 100,000 steps/sec
  X.setMaxVel(10000);
    
  // Set max Acceleration.  Parameter can be between 4000 and 2,000,000 steps/sec/sec
  X.setMaxAccel(50000);
    
  // Enable motor, reset the motor position to 0
  X.enable();

  delay(100);

  // Set up the ISR to constantly update motor position.  All motor(s) must be attached, and enabled before this function is called.
  machine.Start();


  DMXInterface::debug << PSTR("Init complete") << endl;
}


int positionToCount(int inputPosition) {
  return (inputPosition * (maxCounts / 256));
}

void getRequestedPosition() {
  // this next bit you shouldn't need if you can get a dmxRequestedPosition through another routine
  int variableValue = 1023 - analogRead(VariablePin);
  float variablePercentage = (variableValue / 1023.0);
  dmxRequestedPosition = variablePercentage * 256;
  ////////////////////////////////////////////////////////////////////////////////////////////////
  requestedCount = positionToCount(dmxRequestedPosition);
  // DMXInterface::debug
  //   << PSTR("requestedCount: ")
  //   << requestedCount
  //   << PSTR("\t currentPosition: ")
  //   << stepper1.currentPosition()
  //   << endl;
}

void testSteps() {
  static bool toggle = false;
  const long next = toggle ? -10000 : 10000;
  
  toggle = !toggle;
  
  DMXInterface::debug << PSTR("Moving to: ") << next << endl;
  
  X.move(next);
  
  // Wait for command to finish
  while(!X.commandDone());
  
  delay(1000);
}

void handleMessage() {
  static long lastPosition = 0x7fffffffL;

  if (auto msg = DMXInterface::getMessage()) {
    // DMXInterface::debug << PSTR("Message!") << endl;
    auto position = msg->getCommand();

    // stepper1.moveTo(position);

    if (position != lastPosition) {
      DMXInterface::debug << PSTR("Moving to: ") << position << endl;
      lastPosition = position;
    }
  }
}



void loop() {
  // handleMessage();
  
  
  testSteps();

  
  // digitalWrite(PinSpot, millis() % 1000 < 100 ? HIGH : LOW);
  
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }

}
