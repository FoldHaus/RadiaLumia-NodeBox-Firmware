#include <Arduino.h>

#include "Board.h"
#include "Debug.h"
#include "DMXInterface.h"
#include "PinSpot.h"
#include "expCurve.h"
#include "util.h"

using namespace FoldHaus;

constexpr auto PinSpotAmplitudeBits = 8;
constexpr auto PinSpotAmplitudeMax = 255;
uint16_t PinSpotAmplitude = 0;


void PinSpot::selfTest() {
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

  DMXInterface::debug << handleNewBrightness(testNum) << endl;
}


/**
 * @return the actual brightness used
 */
uint8_t PinSpot::handleNewBrightness(uint8_t ampl) {
  // Limit anyone from accidentally setting something too high
  if (ampl > PinSpotAmplitudeMax) {
    ampl = PinSpotAmplitudeMax;
  }

  pinMode(Board::PinSpot, ampl ? OUTPUT : INPUT);
  
  return OCR2B = PinSpotAmplitude = ampl;
}

void PinSpot::loop() {
  
  if (Debug::PinSpot::TestWithButton) {
    if (Board::DebugButton::isActive()) {
      const auto ocr = OCR2B;

      if (ocr == 0xff) {
        handleNewBrightness(0);
      }
      else if (ocr == 0) {
        handleNewBrightness(1);
      }
      else {
        handleNewBrightness((ocr << 1) + 1);
      }
      DMXInterface::debug << PSTR("New PinSpot: ") << OCR2B << endl;
    }
    
    while (Board::DebugButton::isActive());
    delay(10);
  }

  return;
  digitalWrite(Board::PinSpot,
      // If we're max, really be full on and skip the micros() check
      PinSpotAmplitude >= PinSpotAmplitudeMax
      ||
      // Use micros() to generate our base PWM sawtooth that we're comparing to
      // If sawtooth function is ever less than the current desired amplitude, turn on, otherwise off.
      // Therefore, if PinSpotAmplitude is 0, this does not turn on.
      (micros() >> 4 & PinSpotAmplitudeMax) < PinSpotAmplitude
    ?
      HIGH
    :
      LOW
  );
}

void PinSpot::setup() {
  // Initialize the pinspot, off
  // pinMode(PinSpot, OUTPUT);
  digitalWrite(Board::PinSpot, LOW);

  ASSR = 0;

  OCR2B = 0;

  TCCR2A = 0b00100011;
  TCCR2B = 0b00000110;
}