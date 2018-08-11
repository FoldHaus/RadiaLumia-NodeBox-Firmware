
#ifndef MOTOR_H
#define MOTOR_H

namespace FoldHaus {
namespace Motor {

// "Pulses" are traditional stepper motor steps (Tied to "Input Resolution")
// "Counts" are clearpath internal encoder counts (fixed at 800)

constexpr unsigned long maxRPM            = 2000;
constexpr unsigned long maxAccel          = 8000;

constexpr unsigned long maxTravelInches   = 28; // Old and possibly wrong
constexpr unsigned long rotationsPerInch  = 4;  //must be set per lead screw pitch

constexpr unsigned long pulsesPerRevolution = 200; //must be set to this in the Clearpath firmware
constexpr unsigned long countsPerRevolution = 800; // Motor encoder resolution

constexpr unsigned int overstep = 7;

// 108.3k counts ~ range for hexa-nodes
// 86.5k counts ~ range for penta-nodes
// 98k counts ~ range for test rig
constexpr unsigned long countsToOpen = 108300;

constexpr unsigned long backoffCounts = 1200;

// Full range in pulses
constexpr unsigned long maxPulses = pulsesPerRevolution * (countsToOpen - backoffCounts) / countsPerRevolution / (1 + overstep);
constexpr unsigned long maxPulsesCalc = maxTravelInches * rotationsPerInch * pulsesPerRevolution / (1 + overstep);

constexpr unsigned long maxPulsesPerSecond = (maxRPM/60) * pulsesPerRevolution / (1 + overstep);
constexpr unsigned long maxPulsesPerSecSec = maxAccel / (1 + overstep);

void setup();
void home();
void printPositionIfChanged();
void loop();
void selfTest();
long handleNewPosition(unsigned long position);

}
}

#endif
