#include <Arduino.h>

#include "MCStatusLed.h"

MCStatusLed::MCStatusLed(int pwmChannel, int pin, bool inverted)
    : MCLedBase(pwmChannel, pin, inverted) {}