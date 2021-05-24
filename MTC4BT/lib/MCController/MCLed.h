#pragma once

#include "MCLedBase.h"

class MCLed : public MCLedBase
{
public:
    MCLed(int led_pin, bool inverted);
    void Update(bool ebrakeEnabled);
};