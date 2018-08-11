

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

  DMXInterface::debug << handleNewPinSpotBrightness(testNum) << endl;
}

constexpr auto PinSpotAmplitudeBits = 12;
constexpr auto PinSpotAmplitudeMax = 255;
uint16_t PinSpotAmplitude = 0;

/**
 * @return the actual brightness used
 */
inline uint8_t handleNewPinSpotBrightness(uint8_t ampl) {
  // Limit anyone from accidentally setting something too high
  if (ampl > PinSpotAmplitudeMax) {
    ampl = PinSpotAmplitudeMax;
  }

  pinMode(PinSpot, ampl ? OUTPUT : INPUT);
  
  return OCR2B = PinSpotAmplitude = ampl;
}

inline void loopDoPinSpot() {
  return;
  digitalWrite(PinSpot,
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

void setupPinspot() {
  // Initialize the pinspot, off
  // pinMode(PinSpot, OUTPUT);
  digitalWrite(PinSpot, LOW);

  ASSR = 0;

  OCR2B = 0;

  TCCR2A = 0b00100011;
  TCCR2B = 0b00000110;
}