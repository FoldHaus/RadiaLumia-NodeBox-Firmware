#include <Arduino.h>
#include <avr/eeprom.h>

#include "Board.h"
#include "Debug.h"
#include "DMXInterface.h"
#include "Motor.h"
#include "PinSpot.h"
#include "util.h"

using namespace FoldHaus;

uint16_t shutdownPos;
uint16_t shutdownPosDefault = 0;
uint16_t shutdownPosMax = Motor::absoluteMaxPulses + 1;
uint16_t EEMEM shutdownPosEE = shutdownPosDefault;

uint16_t shutdownTime;
constexpr uint16_t shutdownTimeDefault = 10000;
constexpr uint16_t shutdownTimeMax = 30000;
uint16_t EEMEM shutdownTimeEE = shutdownTimeDefault;

bool disableMotorOnShutdown;
constexpr uint16_t disableMotorOnShutdownDefault = false;
uint8_t EEMEM disableMotorOnShutdownEE = disableMotorOnShutdownDefault;


using namespace Board;

// const char endl[] PROGMEM = "\r\n";

void setup() {
  // Initialize blip on LED
  DebugLED::on();

  // To make sure things have time to settle,
  // especially the button for EEPROM reset reasons and
  // to let the LED be ON for a second
  delay(1000);

  // Enable the external RS485 hardware before enabling serial interface
  pinMode(RS485RxEnable, OUTPUT);
  digitalWrite(RS485RxEnable, LOW);

  // Initialize DMX serial port
  DMXInterface::init();
  
  // Tell the world we're alive
  DMXInterface::debug << PSTR("Setup") << endl;

  PinSpot::setup();
  
  Motor::setup();

  auto ee = eeprom_read_word(&shutdownPosEE);
  const bool reset = Board::DebugButton::isActive();

  if (reset || ee > shutdownPosMax) {
    eeprom_write_word(&shutdownPosEE, shutdownPos = shutdownPosDefault);
    DMXInterface::debug << PSTR("Shutdown Position set to default") << endl;
  } else {
    shutdownPos = ee;
    DMXInterface::debug << PSTR("Shutdown Position loaded from EEPROM: ") << ee << endl;
  }

  ee = eeprom_read_word(&shutdownTimeEE);

  if (reset || ee > shutdownTimeMax) {
    eeprom_write_word(&shutdownTimeEE, shutdownTime = shutdownTimeDefault);
    DMXInterface::debug << PSTR("Shutdown time set to default") << endl;
  } else {
    shutdownTime = ee;
    DMXInterface::debug << PSTR("Shutdown time loaded from EEPROM: ") << ee << endl;
  }

  ee = eeprom_read_byte(&disableMotorOnShutdownEE);

  if (reset || ee == 0xff) {
    eeprom_write_byte(&disableMotorOnShutdownEE, disableMotorOnShutdown = disableMotorOnShutdownDefault);
    DMXInterface::debug << PSTR("Disable Motor on Shutdown set to default") << endl;
  } else {
    disableMotorOnShutdown = ee;
    DMXInterface::debug << PSTR("Disable Motor on Shutdown loaded from EEPROM: ") << ee << endl;
  }

  // Tell the world we're all set
  DMXInterface::debug << PSTR("Init complete") << endl;

  // If we're using the button for other things,
  // make sure we don't trigger them
  if (Debug::Motor::TestWithButton || Debug::PinSpot::TestWithButton) {
    while (Board::DebugButton::isActive());

    delay(20);
  }

  // Turn off led to indicate end of init
  DebugLED::off();
}

uint8_t updateShutdownPosition(uint16_t pos) {
  // Easy shortcut for controller to set disabled
  if (pos == 0xffff) pos = shutdownPosMax;

  if (pos > shutdownPosMax) return 2;

  if (shutdownPos == pos) return 1;

  eeprom_write_word(&shutdownPosEE, shutdownPos = pos);
  return 0;
}

uint8_t updateShutdownTime(const uint16_t time) {
  if (time > shutdownTimeMax) return 2;

  if (shutdownTime == time) return 1;

  eeprom_write_word(&shutdownTimeEE, shutdownTime = time);
  return 0;
}

uint8_t updateMotorShutdownOnShutdown(const bool val) {
  if (disableMotorOnShutdown == val) return 1;

  eeprom_write_byte(&disableMotorOnShutdownEE, disableMotorOnShutdown = val);
  return 0;
}

bool handleMessage() {
  auto msg = DMXInterface::getMessage();

  // If we don't have new message, don't do anything.
  if (!msg) return false;
  // DebugLED::on();

  const auto command = msg->getCommand();
  const unsigned long position = msg->getMotor();

  if (Debug::DMX::Messages) {
    DMXInterface::debug
      << PSTR("PSpot: ")
      << PinSpot::handleNewBrightness(msg->getPinspot())
      << PSTR("\tCommand: ")
      << command
      << PSTR("\tMotor: ")
      << position
      ;
  }

  if (command == 0) {
    auto delta = Motor::handleNewPosition(position);

    // Might as well only print deltas when they're non-zero
    if (Debug::DMX::Messages && delta) {
      DMXInterface::debug << PSTR("\tDelta: ") << delta;
    }
  }

  if (command == 1) {
    const auto res = Motor::updateMaxPulses(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tMax Pulses updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tMax Pulses update rejected");
      }
    }
  }

  if (command == 2) {
    const auto res = Motor::updateMaxPulsesPerSec(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tMax Pulses perSec updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tMax Pulses perSec update rejected");
      }
    }
  }

  if (command == 3) {
    const auto res = Motor::updateMaxPulsesPerSecPerSec(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tMax Pulses perSec perSec updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tMax Pulses perSec perSec update rejected");
      }
    }
  }

  if (command == 4) {
    const auto res = Motor::updateMaxHomingTimeMillis(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tMax Homeing time updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tMax Homeing time update rejected");
      }
    }
  }

  if (command == 5) {
    const auto res = Motor::updateAutoHomeDelay(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tAuto Home Delay updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
    }
  }

  if (command == 6) {
    const auto res = Motor::updateHomeOnMessage(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tHome on Message updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
    }
  }

  if (command == 10) {
    const auto res = updateShutdownPosition(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tShutdown position updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tShutdown position update rejected");
      }
    }
  }

  if (command == 11) {
    const auto res = updateShutdownTime(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tShutdown Timeout updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
      if (res == 2) {
        DMXInterface::debug << PSTR("\tShutdown Timeout update rejected");
      }
    }
  }

  if (command == 12) {
    const auto res = updateMotorShutdownOnShutdown(position);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tMotor Shutdown On Shutdown updated");
      }
      if (res == 1) {
        DMXInterface::debug << PSTR("\tNo Change");
      }
    }
  }

  if (command == 0xff) {
    const auto res = Motor::home(false);

    if (Debug::DMX::Messages) {
      if (res == 0) {
        DMXInterface::debug << PSTR("\tHomeing Started");
      }
      else if (res == 1) {
        DMXInterface::debug << PSTR("\tAlready Homeing");
      }
      else {
        DMXInterface::debug << PSTR("\tHoming Error: ") << res;
      }
    }
  }

  // Don't forget to finish out output lines
  if (Debug::DMX::Messages) {
    DMXInterface::debug << endl;
  }

  // DebugLED::off();
  return true;
}

void messageLoop() {
  static unsigned long lastMessageTime = 0;
  // Timeout flag defaults to on
  static bool timeout = false;
  static bool off = true;
  static bool motorShutdown = false;

  // If we've just received a valid message, mark the time. No Timeout! Yay!
  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
    off = false;
    motorShutdown = false;
  } else {
    // If we've gone a while since a message, shutdown
    if (!off && millis() - lastMessageTime >= shutdownTime) {
      DMXInterface::debug << PSTR("Shutting down pinspot for safety") << endl;
      PinSpot::handleNewBrightness(0);

      if (shutdownPos <= Motor::absoluteMaxPulses) {
        DMXInterface::debug << PSTR("Moving to default location") << endl;
        Motor::handleNewPosition(shutdownPos, false);
      }
      if (disableMotorOnShutdown) {
        motorShutdown = true;
      }

      off = true;
    }
    // If we've gone 0.25 seconds since a message, indicate timeout but don't spam
    if (!timeout && millis() - lastMessageTime >= 250) {
      DMXInterface::debug << PSTR("Timeout") << endl;
      timeout = true;
    }
    if (motorShutdown && !Motor::isMoving()) {
      Motor::disable();
      motorShutdown = false;
    }
  }
}

void loop() {

  messageLoop();

  // Call the non-blocking pinspot main
  PinSpot::loop();

  // Call non-blocking stepper main
  Motor::loop();

  // PinSpot::selfTest();
  // Motor::selfTest();

  // Toggle the pinspot on and off
  // digitalWrite(PinSpot, millis() % 3000 < 1000 ? HIGH : LOW);

  // Send a 1Hz "Hello" to serial serminal
  // if (millis() % 1000 == 0) {
  //   DMXInterface::debug << PSTR("Hello") << millis() << endl;
  // }
}
