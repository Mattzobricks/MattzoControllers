#include <Arduino.h>

#include "MCLed.h"

MCLedBase::MCLedBase(int led_pin, bool inverted)
    : _led_pin{led_pin}, _inverted{inverted}
{
    pinMode(led_pin, OUTPUT);
}

// Returns the pin number the LED is attached to.
int MCLedBase::GetPin()
{
    return _led_pin;
}

// Switches the status of the LED to on (true) or off (false).
void MCLedBase::Switch(bool on)
{
    _on = on;
}

// Writes the status to the LED pin (to be used in a loop).
void MCLedBase::Write(bool on)
{
    if (_led_pin > 0)
    {
        if (!_inverted)
        {
            digitalWrite(_led_pin, on ? HIGH : LOW);
        }
        else
        {
            digitalWrite(_led_pin, on ? LOW : HIGH);
        }
    }
}