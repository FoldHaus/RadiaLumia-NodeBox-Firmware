#include <Arduino.h>
#include <ClearPathMotorSD.h>
#include <ClearPathStepGen.h>

#include "Board.h"

#include "DMXInterface.h"

using namespace Foldhaus;

using namespace Board;

constexpr unsigned int maxRPM            = 4000;
constexpr unsigned int maxAccel          = 8000;
constexpr unsigned int maxTravelInches   = 28;
constexpr unsigned int countsPerRotation = 200; //must be set to this in the Clearpath firmware
constexpr unsigned int rotationsPerInch  = 4;  //must be set per lead screw pitch
constexpr unsigned int maxCounts         = maxTravelInches * countsPerRotation * rotationsPerInch;

// const char endl[] PROGMEM = "\r\n";
constexpr char endl = '\n';

ClearPathMotorSD X;

//initialize the controller and pass the reference to the motor we are controlling
ClearPathStepGen machine(&X);


void setup() {
  DMXInterface::init();
  
  DMXInterface::debug << PSTR("Setup") << endl;
  // Initialize blip
  DebugLED::on();

  pinMode(PinSpot, OUTPUT);
  digitalWrite(PinSpot, LOW);

  pinMode(RS485RxEnable, OUTPUT);
  digitalWrite(RS485RxEnable, LOW);


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

  delay(1000);

  DMXInterface::debug << PSTR("Init complete") << endl;
  DebugLED::off();
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

long doMotor(unsigned long position) {
  static unsigned long lastPosition = 0;

  if (position > maxCounts) {
    position = maxCounts;
  }
  
  const long delta = position - lastPosition;

  if (X.move(delta)) {
    lastPosition = position;
  }

  return delta;
}

void doPinspot(uint8_t ampl) {
  digitalWrite(PinSpot, ampl ? HIGH : LOW);
}

bool handleMessage() {
  auto msg = DMXInterface::getMessage();

  if (!msg) return false;

  // DebugLED::on();
  unsigned long position = msg->getCommand();
  
  uint8_t ps = msg->getPinspot();

  DMXInterface::debug << PSTR("Motor: ") << position << PSTR("\tSpot: ") << ps;

  auto delta = doMotor(position);

  if (delta) {
    DMXInterface::debug << PSTR("\tdelta: ") << delta;
  }

  doPinspot(ps);

  DMXInterface::debug << endl;
  // DebugLED::off();

  return true;
}


void loop() {
  static unsigned long lastMessageTime = 0;
  static bool timeout = true;

  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
  } else {
    if (millis() - lastMessageTime >= 250) {
      if (!timeout) {
        DMXInterface::debug << PSTR("Timeout") << endl;
        timeout = true;
      }
    }
  }
  
  // DebugLED::set(digitalRead(Feedback) == LOW);


  // testSteps();


  // digitalWrite(PinSpot, millis() % 3000 < 1000 ? HIGH : LOW);

  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }

}
