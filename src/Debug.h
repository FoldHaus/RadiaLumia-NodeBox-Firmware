#ifndef DEBUG_H
#define DEBUG_H


namespace Debug {
  namespace DMX {
    constexpr bool Messages = true;
  }
  namespace Motor {
    constexpr bool PositionUpdates = false;
    constexpr bool TestWithButton = false;
  }

  namespace PinSpot {
    constexpr bool TestWithButton = true;
  }
}

#endif
