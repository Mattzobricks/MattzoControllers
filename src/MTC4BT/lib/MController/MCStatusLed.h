#pragma once

#include "MCLedBase.h"

class MCStatusLed : public MCLedBase
{
  public:
    MCStatusLed(int pwmChannel, int pin, bool inverted);
};