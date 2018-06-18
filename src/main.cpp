#include <Arduino.h>
#include <AccelStepper.h>

#include "DMXInterface.h"

using namespace Foldhaus;

constexpr int VariablePin       = A3;

constexpr int RS485RxEnable     = 4; // PD4

// ClearPath ENABLE;
constexpr int EnablePin         = 10; // PB2
// ClearPath Input A (DIRECTION);
constexpr int DirectionPin      = 9; // PB1
// ClearPath Input B (STEP);
constexpr int StepPin           = 8; // PB0

constexpr int maxRPM            = 2500;
constexpr int maxAccel          = 4000;
constexpr int maxTravelInches   = 28;
constexpr int countsPerRotation = 200; //must be set to this in the Clearpath firmware
constexpr int rotationsPerInch  = 4;  //must be set per lead screw pitch
constexpr int maxCounts         = maxTravelInches * countsPerRotation * rotationsPerInch;

int Enabled                               = false;
int currentCount                          = 0;
int requestedCount                        = 0;
int dmxRequestedPosition                  = 0;

// const char endl[] PROGMEM = "\n";
constexpr char endl = '\n';

AccelStepper stepper1(AccelStepper::DRIVER, StepPin, DirectionPin);

void enableServo() {
  digitalWrite(EnablePin, HIGH);
  stepper1.setCurrentPosition(0);
  Enabled = true;
  delay(6000);
}

void disableServo() {
  digitalWrite(EnablePin, LOW);
  Enabled = false;
}

void setup()
{
  pinMode(EnablePin, OUTPUT);
  disableServo();
  digitalWrite(DirectionPin, LOW);
  digitalWrite(StepPin, LOW);
  delay(400); // Just give things a chance to settle in before enabling motor;

  stepper1.setMaxSpeed((maxRPM/60)*countsPerRotation);
  stepper1.setAcceleration(maxAccel);
  stepper1.setPinsInverted(true, false, false);
  enableServo();

  DMXInterface::init();

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

void loop() {

  static long lastPosition = 0x7fffffffL;

  if (auto msg = DMXInterface::getMessage()) {
    auto position = msg->getCommand();

    stepper1.moveTo(position);

    if (position != lastPosition) {
      DMXInterface::debug << PSTR("Moving to: ") << position << endl;
      lastPosition = position;
    }
  }

  // if (millis() % 20 == 0) {
  //   getRequestedPosition();
  //   stepper1.moveTo(requestedCount);
  // }

  stepper1.run();
}
