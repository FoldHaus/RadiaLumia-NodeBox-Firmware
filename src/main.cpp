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
  // Initialize blip on LED
  DebugLED::on();

  // Enable the external RS485 hardware before enabling serial interface
  pinMode(RS485RxEnable, OUTPUT);
  digitalWrite(RS485RxEnable, LOW);

  // Initialize DMX serial port
  DMXInterface::init();
  
  // Tell the world we're alive
  DMXInterface::debug << PSTR("Setup") << endl;

  // Initialize the pinspot, off
  pinMode(PinSpot, OUTPUT);
  digitalWrite(PinSpot, LOW);

  // Setup motor
  X.attach(DirectionPin, StepPin, EnablePin, Feedback);

  // Set max Velocity.  Parameter can be between 2 and 100,000 steps/sec
  X.setMaxVel(10000);

  // Set max Acceleration.  Parameter can be between 4000 and 2,000,000 steps/sec/sec
  X.setMaxAccel(50000);

  // Enable motor
  X.enable();

  // Delay a tad. Require before the "machine" is started (I think)
  delay(100);

  // Set up the ISR to constantly update motor position.
  // All motor(s) must be attached and enabled before this function is called.
  machine.Start();

  // Delay some more for no real reason
  delay(1000);

  // Tell the world we're all set
  DMXInterface::debug << PSTR("Init complete") << endl;

  // Turn off led to indicate end of init
  DebugLED::off();
}

void testMotorSteps() {
  static bool toggle = false;
  const long next = toggle ? -10000 : 10000;

  toggle = !toggle;

  DMXInterface::debug << PSTR("Moving to: ") << next << endl;

  DebugLED::on();

  X.move(next);

  // Wait for command to finish
  while(!X.commandDone());
  
  DebugLED::off();

  delay(1000);
}

long handleNewMotorPosition(unsigned long position) {
  static unsigned long lastPosition = 0;

  // Limit motor position to some range
  if (position > maxCounts) {
    position = maxCounts;
  }
  
  const long delta = position - lastPosition;

  // Our motor library only accepts delta commands
  // It also only accepts commands if it is not still moving
  // This probably needs to change
  // Regardless, if it does not accept the delta, it tells us,
  // and we need to only record the new position if it was accpected by the driver
  if (X.move(delta)) {
    lastPosition = position;
  }

  return delta;
}

constexpr auto PinSpotAmplitudeBits = 12;
constexpr auto PinSpotAmplitudeMax = (1 << PinSpotAmplitudeBits) - 1;
uint16_t PinSpotAmplitude = 0;

/**
 * @return the actual brightness used
 */
inline uint16_t handleNewPinSpotBrightness(uint16_t ampl) {
  // Limit anyone from accidentally setting something too high,
  // even though it would not really hurt anything with the current implementation
  if (ampl > PinSpotAmplitudeMax) {
    return PinSpotAmplitude = PinSpotAmplitudeMax;
  }
  
  return PinSpotAmplitude = ampl;
}

inline void loopDoPinSpot() {
  // If sawtooth function is ever less than the current desired amplitude, turn on, otherwise off.
  digitalWrite(PinSpot, (micros() & PinSpotAmplitudeMax) < PinSpotAmplitude ? HIGH : LOW);
}

bool handleMessage() {
  auto msg = DMXInterface::getMessage();

  // If we don't have new message, don't do anything.
  if (!msg) return false;
  // DebugLED::on();

  unsigned long position = msg->getCommand();

  DMXInterface::debug
    << PSTR("PSpot: ")
    << handleNewPinSpotBrightness(msg->getPinspot())
    << PSTR("\tMotor: ")
    << position
    ;

  auto delta = handleNewMotorPosition(position);

  // Might as well only print deltas when they're non-zero
  if (delta) {
    DMXInterface::debug << PSTR("\tDelta: ") << delta;
  }

  // Don't forget to finish out output lines
  DMXInterface::debug << endl;

  // DebugLED::off();
  return true;
}

void testPinSpot() {
  // The lowest bits of time generate a nice sawtooth function.
  // At 12-bits it's about 4Hz
  uint16_t testNum = millis();

  // Grab the next bit from the time to reverse the direction so we can generate
  // a triangle wave instead of a sawtooth
  bool dir = testNum >> PinSpotAmplitudeBits & 1;

  // Get just the bits we want to test with
  testNum &= PinSpotAmplitudeMax;

  // Generate triangle wave from sawtooth
  if (dir) testNum = PinSpotAmplitudeMax - testNum;

  // Do a logarithmic dimming of linear input time makes dimmer seem more natural
  constexpr auto logMaxOverMax = log(PinSpotAmplitudeMax) / PinSpotAmplitudeMax;
  // We also don't want a brightness of "1" when we actually want trully 0.
  if (testNum) testNum = round(exp(logMaxOverMax * testNum));

  handleNewPinSpotBrightness(testNum);
}


void loop() {
  static unsigned long lastMessageTime = 0;
  // Timeout flag defaults to on
  static bool timeout = true;

  // If we've just received a valid message, mark the time. No Timeout! Yay!
  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
  } else {
    // If we've gone 10 seconds since a message, turn off pinspot
    if (millis() - lastMessageTime >= 10 * 1000) {
      handleNewPinSpotBrightness(0);
    }
    // If we've gone 0.25 seconds since a message, indicate timeout but don't spam
    if (!timeout && millis() - lastMessageTime >= 250) {
      DMXInterface::debug << PSTR("Timeout") << endl;
      timeout = true;
    }
  }

  // Call the non-blocking pinspot main
  loopDoPinSpot();
  
  // Test the motor's Feedback line
  // DebugLED::set(digitalRead(Feedback) == LOW);

  // testPinSpot();
  // testMotorSteps();

  // Toggle the pinspot on and off
  // digitalWrite(PinSpot, millis() % 3000 < 1000 ? HIGH : LOW);

  // Send a 1Hz "Hello" to serial serminal
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }

}
