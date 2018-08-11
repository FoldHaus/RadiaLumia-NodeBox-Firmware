

// Setup stepper, but don't initialize the outputs
AccelStepper stepper1(AccelStepper::DRIVER, StepPin, DirectionPin, 0xff, 0xff, false);

void setupMotorWithAccelStepperLib() {
  digitalWrite(EnablePin, LOW);
  pinMode(EnablePin, OUTPUT);

  stepper1.setPinsInverted(true, false, false);
  stepper1.setOverstepCount(overstep);

  stepper1.setMaxSpeed(maxPulsesPerSecond);
  stepper1.setAcceleration(maxPulsesPerSecSec);
}

void home() {
  state = State::Homing;

  digitalWrite(EnablePin, LOW);
  
  delay(10);

  if (Board::Feedback::isActive()) {
    DMXInterface::debug << PSTR("Short on HLFB or motor missconfigured") << endl;
    Board::DebugLED::on();
    while(1);
  }

  digitalWrite(EnablePin, HIGH);
  DMXInterface::debug << PSTR("Homing") << endl;

  delay(100);
  
  if (!Board::Feedback::isActive()) {
    DMXInterface::debug << PSTR("No Motor Present") << endl;

    Board::DebugLED::on();

    while(1);
  }
  
  delay(25 * 1000);
  
  state = State::Normal;
  stepper1.setCurrentPosition(0);
  stepper1.enableOutputs();
  DMXInterface::debug << PSTR("Homed") << endl;
}

void printPositionIfChanged() {
  static typeof(stepper1.currentPosition()) last = -1;

  auto const pos = stepper1.currentPosition();

  if (last == pos) return;

  last = pos;

  if (Debug::Motor::PositionUpdates) {
    DMXInterface::debug << PSTR("At pos: ") << pos << endl;
  }
}

inline void loopDoMotor() {
  stepper1.run();
  printPositionIfChanged();
  
  // Test the motor's Feedback line
  // DebugLED::set(digitalRead(Feedback) == LOW);
}

void testMotorSteps() {
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

  handleNewMotorPosition(next);
}

long handleNewMotorPosition(unsigned long position) {
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