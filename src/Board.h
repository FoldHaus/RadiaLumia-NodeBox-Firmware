

#include <AVR++/IOpin.h>

namespace Foldhaus {

namespace Board {

  using namespace AVR;

  using DebugLED = Output<Ports::D, 2, false, false>;

  using DebugButton = Input<Ports::C, 0>;

  constexpr int RS485RxEnable     = A1; // PC1

  constexpr int PinSpot = 3; // PD3

  // ClearPath ENABLE;
  constexpr int EnablePin         = 10; // PB2
  // ClearPath Input A (DIRECTION);
  constexpr int DirectionPin      = 9; // PB1
  // ClearPath Input B (STEP);
  constexpr int StepPin           = 8; // PB0

  constexpr int Feedback          = 7; // PD7


}

}
