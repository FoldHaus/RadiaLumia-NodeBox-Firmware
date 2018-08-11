#include <Arduino.h>
#include <AccelStepper.h>
#include <avr/eeprom.h>

#include "Board.h"
#include "Debug.h"
#include "DMXInterface.h"
#include "Motor.h"
#include "util.h"

using namespace FoldHaus;
using namespace Motor;

uint16_t EEMEM maxPulsesEE = defaultMaxPulses;
uint16_t EEMEM maxPulsesPerSecondEE = defaultMaxPulsesPerSecond;
uint16_t EEMEM maxPulsesPerSecSecEE = defaultMaxPulsesPerSecSec;

uint16_t Motor::maxPulses;
uint16_t Motor::maxPulsesPerSecond;
uint16_t Motor::maxPulsesPerSecSec;

uint16_t EEMEM maxHomingTimeMillisEE = defaultMaxHomingTimeMillis;
uint16_t Motor::maxHomingTimeMillis;

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

  auto ee = eeprom_read_word(&maxPulsesEE);

  if (ee > absoluteMaxPulses) {
    eeprom_write_word(&maxPulsesEE, maxPulses = defaultMaxPulses);
    DMXInterface::debug << PSTR("Pulse limit set to default") << endl;
  } else {
    maxPulses = ee;
    DMXInterface::debug << PSTR("Pulse limit loaded from EEPROM: ") << ee << endl;
  }

  ee = eeprom_read_word(&maxPulsesPerSecondEE);

  if (ee > absoluteMaxPulsesPerSecond) {
    eeprom_write_word(&maxPulsesPerSecondEE, maxPulsesPerSecond = defaultMaxPulsesPerSecond);
    DMXInterface::debug << PSTR("PulsePerSec limit set to default") << endl;
  } else {
    maxPulsesPerSecond = ee;
    DMXInterface::debug << PSTR("PulsePerSec limit loaded from EEPROM: ") << ee << endl;
  }
  stepper1.setMaxSpeed(maxPulsesPerSecond);

  ee = eeprom_read_word(&maxPulsesPerSecSecEE);

  if (ee > absoluteMaxPulsesPerSecSec) {
    eeprom_write_word(&maxPulsesPerSecSecEE, maxPulsesPerSecSec = defaultMaxPulsesPerSecSec);
    DMXInterface::debug << PSTR("PulsePerSecSec limit set to default") << endl;
  } else {
    maxPulsesPerSecSec = ee;
    DMXInterface::debug << PSTR("PulsePerSecSec limit loaded from EEPROM: ") << ee << endl;
  }
  stepper1.setAcceleration(maxPulsesPerSecSec);

  ee = eeprom_read_word(&maxHomingTimeMillisEE);

  if (ee > absoluteMaxHomingTimeMillis) {
    eeprom_write_word(&maxHomingTimeMillisEE, maxHomingTimeMillis = defaultMaxHomingTimeMillis);
    DMXInterface::debug << PSTR("Homing time limit set to default") << endl;
  } else {
    maxHomingTimeMillis = ee;
    DMXInterface::debug << PSTR("Homing time limit loaded from EEPROM: ") << ee << endl;
  }

}

uint8_t Motor::updateMaxPulses(const uint16_t max) {
  if (max > absoluteMaxPulses) return 2;

  if (maxPulses == max) return 1;

  eeprom_write_word(&maxPulsesEE, maxPulses = max);
  return 0;
}

uint8_t Motor::updateMaxPulsesPerSec(const uint16_t max) {
  if (max > absoluteMaxPulsesPerSecond) return 2;

  if (maxPulsesPerSecond == max) return 1;

  eeprom_write_word(&maxPulsesPerSecondEE, maxPulsesPerSecond = max);
  stepper1.setMaxSpeed(maxPulsesPerSecond);
  return 0;
}

uint8_t Motor::updateMaxPulsesPerSecPerSec(const uint16_t max) {
  if (max > absoluteMaxPulsesPerSecSec) return 2;

  if (maxPulsesPerSecSec == max) return 1;

  eeprom_write_word(&maxPulsesPerSecSecEE, maxPulsesPerSecSec = max);
  stepper1.setAcceleration(maxPulsesPerSecSec);
  return 0;
}

uint8_t Motor::updateMaxHomingTimeMillis(const uint16_t max) {
  if (max > absoluteMaxHomingTimeMillis) return 2;

  if (maxHomingTimeMillis == max) return 1;

  eeprom_write_word(&maxHomingTimeMillisEE, maxHomingTimeMillis = max);
  return 0;
}

unsigned long homeStartedAt;

uint8_t Motor::home(bool verbose) {
  if (state == State::Homing) return 1;

  uint8_t err = 0;

  state = State::Homing;

  digitalWrite(Board::EnablePin, LOW);
  
  delay(10);

  if (Board::Feedback::isActive()) {
    if (verbose) {
      DMXInterface::debug << PSTR("Short on HLFB or motor missconfigured") << endl;
    }
    err = 2;
    Board::DebugLED::on();
  }

  digitalWrite(Board::EnablePin, HIGH);
  homeStartedAt = millis();
  if (verbose) {
    DMXInterface::debug << PSTR("Homing") << endl;
  }

  delay(100);
  
  if (!Board::Feedback::isActive()) {
    if (verbose) {
      DMXInterface::debug << PSTR("No Motor Present") << endl;
    }
    err = 3;
    Board::DebugLED::on();
  }

  return err;
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
  if (state == State::Init) return;

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

long Motor::handleNewPosition(uint16_t position) {

  // Home when we get any position message but are not already homed.
  if (state == State::Init) {
    home();
    return 0;
  }

  if (state != State::Normal) return 0;

  static uint16_t lastPosition = 0;
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
  
  const int16_t delta = position - lastPosition;
  
  lastPosition = position;

  return delta;
}