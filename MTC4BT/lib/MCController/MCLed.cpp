#include <Arduino.h>

#include "MCLed.h"

MCLed::MCLed(int led_pin, bool inverted)
    : MCLedBase(led_pin, inverted) {}


// Updates the status of the LED.
void MCLed::Update()
{
    // Write state to pin.
    Write(_on);
}