

void loopDoDebug() {

  if (Board::DebugButton::isActive()) {

    if (Debug::Motor::TestWithButton) {
      if (state == State::Init) {
        home();
      }
      else

      if (state == State::Normal) {
        if (!stepper1.isRunning()) {
          DMXInterface::debug << PSTR("Test Move") << endl;
          testMotorSteps();
        } else {
          DMXInterface::debug << PSTR("Already moving. Rejected") << endl;
        }
      }
    }

    if (Debug::PinSpot::TestWithButton) {
      const auto ocr = OCR2B;

      if (ocr == 0xff) {
        handleNewPinSpotBrightness(0);
      }
      else if (ocr == 0) {
        handleNewPinSpotBrightness(1);
      }
      else {
        handleNewPinSpotBrightness((ocr << 1) + 1);
      }
      DMXInterface::debug << PSTR("New PinSpot: ") << OCR2B << endl;
    }
    
    while (Board::DebugButton::isActive());
    delay(10);
  }
}