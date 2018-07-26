#include <Arduino.h>
#include <AccelStepper.h>

#include "Board.h"

#include "testPinSpot.h"

#include "DMXInterface.h"

using namespace Foldhaus;

using namespace Board;

namespace Debug {
  namespace DMX {
    constexpr bool Messages = true;
  }
}

// "Pulses" are traditional stepper motor steps (Tied to "Input Resolution")
// "Counts" are clearpath internal encoder counts (fixed at 800)

constexpr unsigned long maxRPM            = 2000;
constexpr unsigned long maxAccel          = 8000;

constexpr unsigned long maxTravelInches   = 28; // Old and possibly wrong
constexpr unsigned long rotationsPerInch  = 4;  //must be set per lead screw pitch

constexpr unsigned long pulsesPerRevolution = 200; //must be set to this in the Clearpath firmware
constexpr unsigned long countsPerRevolution = 800; // Motor encoder resolution

// 103k counts ~ range for hexa-nodes
// 88k counts ~ range for penta-nodes
constexpr unsigned long countsToOpen = 103000;

// Full range in pulses
constexpr unsigned long maxPulses = pulsesPerRevolution * countsToOpen / countsPerRevolution;
constexpr unsigned long maxPulsesCalc = maxTravelInches * rotationsPerInch * pulsesPerRevolution;

enum class State : u1 {
  Init,
  Homing,
  Normal,
};

State state = State::Init;

// const char endl[] PROGMEM = "\r\n";
constexpr char endl = '\n';

// Setup stepper, but don't initialize the outputs
AccelStepper stepper1(AccelStepper::DRIVER, StepPin, DirectionPin, 0xff, 0xff, false);

void setupMotorWithAccelStepperLib() {
  digitalWrite(EnablePin, LOW);
  pinMode(EnablePin, OUTPUT);

  stepper1.setPinsInverted(true, false, false);

  stepper1.setMaxSpeed((maxRPM/60/8)*pulsesPerRevolution);
  stepper1.setAcceleration(maxAccel);
}

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

  setupMotorWithAccelStepperLib();

  // Delay some more for no real reason
  delay(1000);

  // Tell the world we're all set
  DMXInterface::debug << PSTR("Init complete") << endl;

  // Turn off led to indicate end of init
  DebugLED::off();
}

long handleNewMotorPosition(unsigned long position);

void testMotorSteps() {
  static bool toggle = false;
  static unsigned long lastTime = 0;

  auto time = millis();

  if (time - lastTime < 4000) return;
  lastTime = time;

  const long next = toggle ? 0 : 10000;

  toggle = !toggle;

  DMXInterface::debug << PSTR("Moving to: ") << next << endl;

  handleNewMotorPosition(next);
}

long handleNewMotorPosition(unsigned long position) {
  static unsigned long lastPosition = 0;

  // Limit motor position to some range
  if (position > maxPulses) {
    position = maxPulses;
  }

  stepper1.moveTo(position);
  
  const long delta = position - lastPosition;
  
  lastPosition = position;

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
  digitalWrite(PinSpot,
      // If we're max, really be full on and skip the micros() check
      PinSpotAmplitude >= PinSpotAmplitudeMax
      ||
      // Use micros() to generate our base PWM sawtooth that we're comparing to
      // If sawtooth function is ever less than the current desired amplitude, turn on, otherwise off.
      // Therefore, if PinSpotAmplitude is 0, this does not turn on.
      (micros() & PinSpotAmplitudeMax) < PinSpotAmplitude
    ?
      HIGH
    :
      LOW
  );
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

void testPinSpot() {
  // The lowest bits of time generate a nice sawtooth function.
  // At 12-bits it's about 4Hz
  uint16_t testNum = millis();

  static bool prevDir = false;
  // Grab the next bit from the time to reverse the direction so we can generate
  // a triangle wave instead of a sawtooth
  bool dir = testNum >> PinSpotAmplitudeBits & 1;

  if (dir != prevDir) {
    DMXInterface::debug << (dir ? PSTR("Up") : PSTR("Down")) << endl;

    prevDir = dir;
  }

  // Get just the bits we want to test with
  testNum &= PinSpotAmplitudeMax;

  // Generate triangle wave from sawtooth
  if (dir) testNum = PinSpotAmplitudeMax - testNum;

  // Do a logarithmic dimming of linear input time makes dimmer seem more natural
  testNum = curvePS(testNum);

  handleNewPinSpotBrightness(testNum);
}


void loop() {
  static unsigned long lastMessageTime = 0;
  // Timeout flag defaults to on
  static bool timeout = false;
  static bool off = true;

  if (Board::DebugButton::isActive()) {
    if (state == State::Init) {
      state = State::Homing;
      digitalWrite(EnablePin, HIGH);
      DMXInterface::debug << PSTR("Homeing") << endl;
      delay(100);
    }
    if (state == State::Normal) {
      if (!stepper1.isRunning()) {
        DMXInterface::debug << PSTR("Test Move 1000") << endl;
        stepper1.move(1000/8);
      } else {
        DMXInterface::debug << PSTR("Already moving. Rejected") << endl;
      }
    }
    
    while (Board::DebugButton::isActive());
  }

  // If we've just received a valid message, mark the time. No Timeout! Yay!
  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
    off = false;

    if (state == State::Init) {
      state = State::Homing;
      digitalWrite(EnablePin, HIGH);
      DMXInterface::debug << PSTR("Homeing") << endl;
      delay(100);
    }
  } else {
    // If we've gone 10 seconds since a message, turn off pinspot
    if (!off && millis() - lastMessageTime >= 10 * 1000) {
      handleNewPinSpotBrightness(0);
      off = true;
    }
    // If we've gone 0.25 seconds since a message, indicate timeout but don't spam
    if (!timeout && millis() - lastMessageTime >= 250) {
      DMXInterface::debug << PSTR("Timeout") << endl;
      timeout = true;
    }
  }

  if (state == State::Homing && digitalRead(Feedback) == LOW) {
    state = State::Normal;
    stepper1.setCurrentPosition(0);
    stepper1.enableOutputs();
    DMXInterface::debug << PSTR("Homed") << endl;
  }

  if (state == State::Normal && digitalRead(Feedback) == HIGH) {
    state = State::Init;
    digitalWrite(EnablePin, LOW);
    stepper1.disableOutputs();
    DMXInterface::debug << PSTR("Fault!") << endl;
    delay(10000);
  }

  // Call the non-blocking pinspot main
  loopDoPinSpot();

  // Call non-blocking stepper main
  stepper1.run();
  
  // Test the motor's Feedback line
  // DebugLED::set(digitalRead(Feedback) == LOW);

  // testPinSpot();
  testMotorSteps();

  // Toggle the pinspot on and off
  // digitalWrite(PinSpot, millis() % 3000 < 1000 ? HIGH : LOW);

  // Send a 1Hz "Hello" to serial serminal
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }

}
