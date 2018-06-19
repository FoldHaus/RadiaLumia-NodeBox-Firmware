#include <Arduino.h>
#include <AccelStepper.h>

#include "Board.h"

#include "DMXInterface.h"

using namespace Foldhaus;

constexpr int VariablePin       = A3;

constexpr int RS485RxEnable     = 4; // PD4

constexpr int PinSpot = 2; // PD2

// ClearPath ENABLE;
constexpr int EnablePin         = 10; // PB2
// ClearPath Input A (DIRECTION);
constexpr int DirectionPin      = 9; // PB1
// ClearPath Input B (STEP);
constexpr int StepPin           = A2; // PC2

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

AccelStepper stepper1(AccelStepper::DRIVER, StepPin, DirectionPin);

void enableServo() {
  digitalWrite(EnablePin, HIGH);
  stepper1.setCurrentPosition(0);
  Enabled = true;
  // delay(6000);
}

void disableServo() {
  digitalWrite(EnablePin, LOW);
  Enabled = false;
}

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
  
  pinMode(EnablePin, OUTPUT);
  pinMode(DirectionPin, OUTPUT);
  pinMode(StepPin, OUTPUT);
  
  // while (1) {
  //     delay(1000);
  //     digitalWrite(EnablePin, HIGH);
  //     digitalWrite(DirectionPin, HIGH);
  //     digitalWrite(StepPin, HIGH);
  //     delay(1000);
  //     digitalWrite(EnablePin, LOW);
  //     digitalWrite(DirectionPin, LOW);
  //     digitalWrite(StepPin, LOW);
  // }
  
  disableServo();
  digitalWrite(DirectionPin, LOW);
  digitalWrite(StepPin, LOW);
  
  
  delay(400); // Just give things a chance to settle in before enabling motor;

  stepper1.setMaxSpeed((maxRPM/60)*countsPerRotation);
  stepper1.setPinsInverted(true, false, false);
  enableServo();

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
  DMXInterface::debug
    << PSTR("requestedCount: ")
    << requestedCount
    << PSTR("\t currentPosition: ")
    << stepper1.currentPosition()
    << endl;
}

void testSteps() {
  static unsigned long nextStep = 0;
  static bool toggle = false;

  const unsigned long now = millis();

  if (now < nextStep) return;
  
  nextStep += 10000;
  if (nextStep < now) {
    nextStep = now + 1000;
  }
  
  const long next = toggle ? 0 : 10000;
  
  toggle = !toggle;
  
  DMXInterface::debug << PSTR("Moving to: ") << next << endl;
  stepper1.runToNewPosition(next);
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
  
  
  // testSteps();

  
  // digitalWrite(PinSpot, millis() % 1000 < 100 ? HIGH : LOW);
  
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }

  stepper1.run();
}
