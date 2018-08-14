
#ifndef MOTOR_H
#define MOTOR_H

namespace FoldHaus {
namespace Motor {

// "Pulses" are traditional stepper motor steps (Tied to "Input Resolution")
// "Counts" are clearpath internal encoder counts (fixed at 800)

constexpr unsigned long defaultMaxRPM            = 2000;
constexpr unsigned long defaultMaxAccelPPSPS     = 8000;

constexpr unsigned long absoluteMaxRPM            = 2200;
constexpr unsigned long absoluteMaxAccelPPSPS     = 20000;

constexpr unsigned long maxTravelInches   = 28; // Old and possibly wrong
constexpr unsigned long rotationsPerInch  = 4;  //must be set per lead screw pitch

constexpr unsigned long pulsesPerRevolution = 200; //must be set to this in the Clearpath firmware
constexpr unsigned long countsPerRevolution = 800; // Motor encoder resolution

// This relates to a hack inside of the local copy of AccelStepper
constexpr unsigned int overstep = 7;

// 108.3k counts ~ range for hexa-nodes
// 86.5k counts ~ range for penta-nodes
// 98k counts ~ range for test rig
constexpr unsigned long defaultCountsToOpen = 86500;
constexpr unsigned long absoluteMaxCounts = 120000;

constexpr unsigned long backoffCounts = 1200;

// Full range in pulses
extern uint16_t maxPulses;
constexpr uint16_t defaultMaxPulses = (defaultCountsToOpen - backoffCounts) * pulsesPerRevolution / countsPerRevolution / (1 + overstep);
constexpr uint16_t absoluteMaxPulses = (absoluteMaxCounts) * pulsesPerRevolution / countsPerRevolution / (1 + overstep);
// Holdover...
constexpr uint16_t maxPulsesCalc = maxTravelInches * rotationsPerInch * pulsesPerRevolution / (1 + overstep);

extern uint16_t maxPulsesPerSecond;
constexpr uint16_t defaultMaxPulsesPerSecond = (defaultMaxRPM/60) * pulsesPerRevolution / (1 + overstep);
constexpr uint16_t absoluteMaxPulsesPerSecond = (absoluteMaxRPM/60) * pulsesPerRevolution / (1 + overstep);

extern uint16_t maxPulsesPerSecSec;
constexpr uint16_t defaultMaxPulsesPerSecSec = defaultMaxAccelPPSPS / (1 + overstep);
constexpr uint16_t absoluteMaxPulsesPerSecSec = absoluteMaxAccelPPSPS / (1 + overstep);

extern uint16_t maxHomingTimeMillis;
constexpr uint16_t defaultMaxHomingTimeMillis = 25000;
constexpr uint16_t absoluteMaxHomingTimeMillis = 65000;

extern bool homeOnMessage;
constexpr bool homeOnMessageDefault = true;

// 0 == off
extern uint16_t autoHomeDelay;
constexpr uint16_t autoHomeDelayDefault = 0;

void setup(bool resetEEPROM);
/**
 * Home unless already homing.
 * 
 * @return 0 homing. 1 already homing. Other = Error with motor connection
 */ 
uint8_t home(bool verbose = true);
void printPositionIfChanged();
void loop();
void selfTest();
long handleNewPosition(uint16_t position, bool allowHomeOnMessage = true);
/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateMaxPulses(uint16_t max);
/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateMaxPulsesPerSec(uint16_t max);
/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateMaxPulsesPerSecPerSec(uint16_t max);
/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateMaxHomingTimeMillis(uint16_t max);

/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateAutoHomeDelay(const uint16_t delay);

/**
 * Update live and saved values
 * 
 * @return 0 updated. 1 no change. 2 invalid value
 */ 
uint8_t updateHomeOnMessage(const bool hom);

bool isMoving();
void disable();
}
}

#endif
