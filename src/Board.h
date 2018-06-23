

#include <AVR++/IOpin.h>

namespace Foldhaus {

namespace Board {

  using namespace AVR;

  using DebugPin = Output<Ports::C, 0>;

  constexpr int VariablePin       = A3;

  constexpr int RS485RxEnable     = 4; // PD4

  constexpr int PinSpot = 3; // PD3

  // ClearPath ENABLE;
  constexpr int EnablePin         = 10; // PB2
  // ClearPath Input A (DIRECTION);
  constexpr int DirectionPin      = 9; // PB1
  // ClearPath Input B (STEP);
  constexpr int StepPin           = 11; // PB3

  constexpr int Feedback          = A5; // PC5


}

}
