#pragma once

#include "MCLedBase.h"

class MCStatusLed : public MCLedBase
{
public:
    MCStatusLed(int led_pin, bool inverted);
    void Update(bool ebrakeEnabled);
};