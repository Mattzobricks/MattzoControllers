#pragma once

#include "MTC4BTConfiguration.h"
#include "MCLedBase.h"

class MTC4BTController
{
public:
    MTC4BTController(MTC4BTConfiguration *config);

    std::vector<MCLedBase *> Leds;

    void EmergencyBreak(const bool enabled);
    BLELocomotive *GetLocomotive(uint address);
    void HandleFn(int locoAddress, MTC4BTFunction f, const bool on);

private:
    void initStatusLeds();
    std::vector<Fn *> getFunctions(MTC4BTFunction f);
    MCLedBase *getLed(int pin, bool inverted);    

    MTC4BTConfiguration *_config;
};