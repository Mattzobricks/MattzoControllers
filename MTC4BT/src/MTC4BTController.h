#pragma once

#include "MTC4BTConfiguration.h"
#include "MCLed.h"

class MTC4BTController
{
public:
    MTC4BTController(MTC4BTConfiguration *config);

    std::vector<MCLed *> Leds;

    DeviceConfiguration *GetStatusLed();
    void EmergencyBreak(const bool enabled);
    BLELocomotive *GetLocomotive(uint address);
    void HandleFn(int locoAddress, MTC4BTFunction f, const bool on);

private:
    std::vector<Fn *> getFunctions(MTC4BTFunction f);
    MCLed *getLed(int pin, bool inverted);

    MTC4BTConfiguration *_config;
};