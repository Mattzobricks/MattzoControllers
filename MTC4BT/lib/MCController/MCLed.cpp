#include <Arduino.h>

#include "MCLed.h"

MCLed::MCLed(int led_pin, bool inverted)
    : MCLedBase(led_pin, inverted) {}

// Updates the status of the LED.
void MCLed::Update(bool ebrakeEnabled)
{
    if (!ebrakeEnabled)
    {
        // Write state to pin.
        Write(_on);
    }
}