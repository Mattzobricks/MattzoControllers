#pragma once

#include "MCLedBase.h"

class MCLed : public MCLedBase
{
  public:
    MCLed(int pwmChannel, int pin, bool inverted);
};