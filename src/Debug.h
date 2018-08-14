/**
 * Flags for enabling certain built-in test modes and things
 */

#ifndef DEBUG_H
#define DEBUG_H

namespace FoldHaus {
namespace Debug {
  namespace DMX {
    constexpr bool Messages = false;
    constexpr bool CRC = false;
  }
  namespace Motor {
    constexpr bool PositionUpdates = false;
    constexpr bool TestWithButton = false;
  }

  namespace PinSpot {
    constexpr bool TestWithButton = false;
  }
}
}

#endif
