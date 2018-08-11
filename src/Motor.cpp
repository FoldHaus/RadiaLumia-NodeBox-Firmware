#include <Arduino.h>
#include <AccelStepper.h>

#include "Board.h"
#include "Debug.h"
#include "DMXInterface.h"
#include "Motor.h"
#include "util.h"

using namespace FoldHaus;


enum class State : u1 {
  Init,
  Homing,
  Normal,
};

State state = State::Init;

// Setup stepper, but don't initialize the outputs
AccelStepper stepper1(AccelStepper::DRIVER, Board::StepPin, Board::DirectionPin, 0xff, 0xff, false);

void Motor::setup() {
  digitalWrite(Board::EnablePin, LOW);
  pinMode(Board::EnablePin, OUTPUT);

  stepper1.setPinsInverted(true, false, false);
  stepper1.setOverstepCount(overstep);

  stepper1.setMaxSpeed(maxPulsesPerSecond);
  stepper1.setAcceleration(maxPulsesPerSecSec);
  
  DMXInterface::debug << PSTR("Pulse limit: ") << maxPulses << endl;
}

unsigned long homeStartedAt;

void Motor::home() {
  state = State::Homing;

  digitalWrite(Board::EnablePin, LOW);
  
  delay(10);

  if (Board::Feedback::isActive()) {
    DMXInterface::debug << PSTR("Short on HLFB or motor missconfigured") << endl;
    Board::DebugLED::on();
  }

  digitalWrite(Board::EnablePin, HIGH);
  homeStartedAt = millis();
  DMXInterface::debug << PSTR("Homing") << endl;

  delay(100);
  
  if (!Board::Feedback::isActive()) {
    DMXInterface::debug << PSTR("No Motor Present") << endl;

    Board::DebugLED::on();
  }
}

void Motor::printPositionIfChanged() {
  static typeof(stepper1.currentPosition()) last = -1;

  auto const pos = stepper1.currentPosition();

  if (last == pos) return;

  last = pos;

  if (Debug::Motor::PositionUpdates) {
    DMXInterface::debug << PSTR("At pos: ") << pos << endl;
  }
}

void Motor::loop() {

  if (state == State::Homing) {
    if (millis() - homeStartedAt > maxHomingTimeMillis) {
      state = State::Normal;
      stepper1.setCurrentPosition(0);
      stepper1.enableOutputs();
      DMXInterface::debug << PSTR("Homed") << endl;
    }
    return;
  }
  
  stepper1.run();
  printPositionIfChanged();

  if (state == State::Normal && !Board::Feedback::isActive()) {
    state = State::Init;
    digitalWrite(Board::EnablePin, LOW);
    stepper1.disableOutputs();
    DMXInterface::debug << PSTR("Fault! At: ") << stepper1.currentPosition() << endl;
    delay(3000);
  }
  
  // Test the motor's Feedback line
  // DebugLED::set(digitalRead(Feedback) == LOW);

  if (Debug::Motor::TestWithButton) {

    if (Board::DebugButton::isActive()) {
      if (state == State::Init) {
        home();
      }
      else

      if (state == State::Normal) {
        if (!stepper1.isRunning()) {
          DMXInterface::debug << PSTR("Test Move") << endl;
          selfTest();
        } else {
          DMXInterface::debug << PSTR("Already moving. Rejected") << endl;
        }
      }
    }
    while (Board::DebugButton::isActive());
    delay(10);
  }
}

void Motor::selfTest() {
  static bool toggle = false;
  static unsigned long lastTime = 0;

  const auto time = millis();

  if (stepper1.isRunning()) {
    lastTime = time;
    return;
  }

  if (time - lastTime < 1000) {
    return;
  }

  const long next = toggle ? 0 : maxPulses;

  toggle = !toggle;

  DMXInterface::debug << PSTR("Moving to: ") << next << endl;

  handleNewPosition(next);
}

long Motor::handleNewPosition(unsigned long position) {

  if (state == State::Init) {
    home();
    return 0;
  }

  if (state != State::Normal) return 0;

  static unsigned long lastPosition = 0;
  static bool activeWarning = false;

  if (stepper1.isEnabled()) {
    activeWarning = false;
  } else {
    if (!activeWarning) {
      activeWarning = true;
      DMXInterface::debug << PSTR("Rejected move") << endl;
    }
    return 0;
  }

  // Limit motor position to some range
  if (position > maxPulses) {
    position = maxPulses;
  }

  stepper1.moveTo(position);
  
  const long delta = position - lastPosition;
  
  lastPosition = position;

  return delta;
}