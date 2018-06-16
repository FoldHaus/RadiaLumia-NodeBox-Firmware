#include <AccelStepper.h>

const int VariablePin       = A3;
const int EnablePin         = 5;  // ClearPath ENABLE;
const int DirectionPin      = 4;  // ClearPath Input A (DIRECTION);
const int StepPin           = 3;  // ClearPath Input B (STEP);
const int maxRPM            = 2500; 
const int maxAccel          = 4000;
const int maxTravelInches   = 28;
const int countsPerRotation = 200; //must be set to this in the Clearpath firmware
const int rotationsPerInch  = 4;  //must be set per lead screw pitch
const int maxCounts         = maxTravelInches * countsPerRotation * rotationsPerInch;

int Enabled                               = false;
int currentCount                          = 0;
int requestedCount                        = 0;
int dmxRequestedPosition                  = 0;

AccelStepper stepper1(AccelStepper::DRIVER, StepPin, DirectionPin);

void enableServo() {
  digitalWrite(EnablePin, HIGH);
  stepper1.setCurrentPosition(0);
  Enabled = true;
  delay(6000);
}

void disableServo() {
  digitalWrite(EnablePin, LOW);
  Enabled = false;
}

void setup()
{  
  pinMode(EnablePin, OUTPUT);
  disableServo();
  digitalWrite(DirectionPin, LOW);
  digitalWrite(StepPin, LOW);
  delay(400); // Just give things a chance to settle in before enabling motor;
  
  //Serial.begin(115200);
  stepper1.setMaxSpeed((maxRPM/60)*countsPerRotation);
  stepper1.setAcceleration(maxAccel);
  stepper1.setPinsInverted(true, false, false);
  enableServo();
}



int positionToCount(int inputPosition) {
  return (inputPosition * (maxCounts / 256));
}

void getRequestedPosition() {
  // this next bit you shouldn't need if you can get a dmxRequestedPosition through another routine
  int variableValue = 1023 - analogRead(VariablePin);
  float variablePercentage = (variableValue / 1023.0);
  dmxRequestedPosition = variablePercentage * 256;
  ////////////////////////////////////////////////////////////////////////////////////////////////
  requestedCount = positionToCount(dmxRequestedPosition);
  //Serial.print("requestedCount: ");
  //Serial.print(requestedCount);
  //Serial.print("   currentPosition: ");
  //Serial.println(stepper1.currentPosition());
}

void loop() {
  if (millis() % 20 == 0) {
    getRequestedPosition();
    stepper1.moveTo(requestedCount);
  }
  stepper1.run();  
}
