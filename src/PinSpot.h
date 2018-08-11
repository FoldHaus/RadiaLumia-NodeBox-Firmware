#ifndef PINSPOT_H
#define PINSPOT_H

#include <stdint.h>

namespace FoldHaus {
namespace PinSpot {
        

void selfTest();
uint8_t handleNewBrightness(uint8_t ampl);
void loop();
void setup();

}
}

#endif
