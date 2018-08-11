#include <Arduino.h>

#include "Board.h"
#include "Debug.h"
#include "DMXInterface.h"
#include "Motor.h"
#include "PinSpot.h"
#include "util.h"

using namespace FoldHaus;

using namespace Board;

// const char endl[] PROGMEM = "\r\n";

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

  PinSpot::setup();
  
  Motor::setup();

  // Delay some more for no real reason
  delay(1000);

  // Tell the world we're all set
  DMXInterface::debug << PSTR("Init complete") << endl;

  // Turn off led to indicate end of init
  DebugLED::off();
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
      << PSTR("\tPSpot: ")
      << PinSpot::handleNewBrightness(msg->getPinspot())
      << PSTR("Command: ")
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

  // If we've just received a valid message, mark the time. No Timeout! Yay!
  if (handleMessage()) {
    lastMessageTime = millis();
    timeout = false;
    off = false;
  } else {
    // If we've gone 10 seconds since a message, turn off pinspot
    if (!off && millis() - lastMessageTime >= 10 * 1000) {
      DMXInterface::debug << PSTR("Shutting down pinspot for safety") << endl;
      PinSpot::handleNewBrightness(0);
      off = true;
    }
    // If we've gone 0.25 seconds since a message, indicate timeout but don't spam
    if (!timeout && millis() - lastMessageTime >= 250) {
      DMXInterface::debug << PSTR("Timeout") << endl;
      timeout = true;
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
