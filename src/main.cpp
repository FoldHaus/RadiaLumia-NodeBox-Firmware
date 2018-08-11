#include <Arduino.h>
#include <AccelStepper.h>

#include "Board.h"

#include "testPinSpot.h"

#include "DMXInterface.h"

using namespace Foldhaus;

using namespace Board;

// "Pulses" are traditional stepper motor steps (Tied to "Input Resolution")
// "Counts" are clearpath internal encoder counts (fixed at 800)

constexpr unsigned long maxRPM            = 2000;
constexpr unsigned long maxAccel          = 8000;

constexpr unsigned long maxTravelInches   = 28; // Old and possibly wrong
constexpr unsigned long rotationsPerInch  = 4;  //must be set per lead screw pitch

constexpr unsigned long pulsesPerRevolution = 200; //must be set to this in the Clearpath firmware
constexpr unsigned long countsPerRevolution = 800; // Motor encoder resolution

constexpr unsigned int overstep = 7;

// 108.3k counts ~ range for hexa-nodes
// 86.5k counts ~ range for penta-nodes
// 98k counts ~ range for test rig
constexpr unsigned long countsToOpen = 108300;

constexpr unsigned long backoffCounts = 1200;

// Full range in pulses
constexpr unsigned long maxPulses = pulsesPerRevolution * (countsToOpen - backoffCounts) / countsPerRevolution / (1 + overstep);
constexpr unsigned long maxPulsesCalc = maxTravelInches * rotationsPerInch * pulsesPerRevolution / (1 + overstep);

constexpr unsigned long maxPulsesPerSecond = (maxRPM/60) * pulsesPerRevolution / (1 + overstep);
constexpr unsigned long maxPulsesPerSecSec = maxAccel / (1 + overstep);

enum class State : u1 {
  Init,
  Homing,
  Normal,
};

State state = State::Init;

// const char endl[] PROGMEM = "\r\n";
constexpr char endl = '\n';

void setup() {
  // Initialize blip on LED
  DebugLED::on();

  // Enable the external RS485 hardware before enabling serial interface
  pinMode(RS485RxEnable, OUTPUT);
  digitalWrite(RS485RxEnable, LOW);

  // Initialize DMX serial port
  DMXInterface::init();
  
  // Tell the world we're alive
  DMXInterface::debug << PSTR("Setup") << endl;

  setupPinspot();
  
  setupMotorWithAccelStepperLib();

  // Delay some more for no real reason
  delay(1000);

  // Tell the world we're all set
  DMXInterface::debug << PSTR("Init complete") << endl;

  DMXInterface::debug << PSTR("Count limit: ") << countsToOpen << PSTR(" Pulse limit: ") << maxPulses << endl;

  // Turn off led to indicate end of init
  DebugLED::off();
}

bool handleMessage() {
  auto msg = DMXInterface::getMessage();

  // If we don't have new message, don't do anything.
  if (!msg) return false;
  // DebugLED::on();

  unsigned long position = msg->getCommand();

  if (Debug::DMX::Messages) {
    DMXInterface::debug
      << PSTR("PSpot: ")
      << handleNewPinSpotBrightness(msg->getPinspot())
      << PSTR("\tMotor: ")
      << position
      ;
  }

  auto delta = handleNewMotorPosition(position);

  // Might as well only print deltas when they're non-zero
  if (Debug::DMX::Messages && delta) {
    DMXInterface::debug << PSTR("\tDelta: ") << delta;
  }

  // Don't forget to finish out output lines
  
  if (Debug::DMX::Messages) {
    DMXInterface::debug << endl;
  }

  // DebugLED::off();
  return true;
}

void loopDoMain() {
  static unsigned long lastMessageTime = 0;
  // Timeout flag defaults to on
  static bool timeout = false;
  static bool off = true;

  // If we've just received a valid message, mark the time. No Timeout! Yay!
  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
    off = false;

    if (state == State::Init) {
      home();
    }
  } else {
    // If we've gone 10 seconds since a message, turn off pinspot
    if (!off && millis() - lastMessageTime >= 10 * 1000) {
      DMXInterface::debug << PSTR("Shutting down pinspot for safety") << endl;
      handleNewPinSpotBrightness(0);
      off = true;
    }
    // If we've gone 0.25 seconds since a message, indicate timeout but don't spam
    if (!timeout && millis() - lastMessageTime >= 250) {
      DMXInterface::debug << PSTR("Timeout") << endl;
      timeout = true;
    }
  }

  if (state == State::Normal && !Feedback::isActive()) {
    state = State::Init;
    digitalWrite(EnablePin, LOW);
    stepper1.disableOutputs();
    DMXInterface::debug << PSTR("Fault! At: ") << stepper1.currentPosition() << endl;
    delay(3000);
  }
}

void loop() {

  loopDoDebug();

  loopDoMain();

  // Call the non-blocking pinspot main
  loopDoPinSpot();

  // Call non-blocking stepper main
  loopDoMotor();

  // testPinSpot();
  // testMotorSteps();

  // Toggle the pinspot on and off
  // digitalWrite(PinSpot, millis() % 3000 < 1000 ? HIGH : LOW);

  // Send a 1Hz "Hello" to serial serminal
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }
}
