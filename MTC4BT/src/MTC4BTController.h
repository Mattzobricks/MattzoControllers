#pragma once

#include "MCController.h"
#include "MTC4BTConfiguration.h"

class MTC4BTController : public MCController
{
public:
    MTC4BTController(MTC4BTConfiguration *config);

    void EmergencyBreak(const bool enabled);
    BLELocomotive *GetLocomotive(uint address);
    void HandleFn(int locoAddress, MCFunction f, const bool on);

private:
    MTC4BTConfiguration *_config;
};