
/**
 * This file contains all of the things that might change if the board design changes
 */

#ifndef BOARD_H
#define BOARD_H

#include <AVR++/IOpin.h>

namespace Foldhaus {

namespace Board {

  using namespace AVR;

  // To use IOpin (Input & Output), I like keeping them as "types"

  using DebugLED = Output<Ports::D, 2, false, false>;

  using DebugButton = Input<Ports::C, 0>;

  using Feedback = Input<Ports::D, 7>;

  constexpr int RS485RxEnable     = A1; // PC1

  constexpr int PinSpot = 3; // PD3

  // ClearPath ENABLE;
  constexpr int EnablePin         = 10; // PB2
  // ClearPath Input A (DIRECTION);
  constexpr int DirectionPin      = 9; // PB1
  // ClearPath Input B (STEP);
  constexpr int StepPin           = 8; // PB0
}

}

#endif
