#include <Arduino.h>

#include "MCLed.h"

MCLed::MCLed(int pwmChannel, int pin, bool inverted)
    : MCLedBase(pwmChannel, pin, inverted) {}