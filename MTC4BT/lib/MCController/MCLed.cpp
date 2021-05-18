#include <Arduino.h>

#include "MCLed.h"

MCLed::MCLed(int led_pin, bool inverted)
    : _led_pin{led_pin}, _inverted{inverted}
{
    pinMode(led_pin, OUTPUT);
}

int MCLed::GetPin()
{
    return _led_pin;
}

void MCLed::Switch(bool on)
{
    _on = on;
}

void MCLed::Update()
{
    if (_led_pin > 0)
    {
        if (!_inverted)
        {
            digitalWrite(_led_pin, _on ? HIGH : LOW);
        }
        else
        {
            digitalWrite(_led_pin, _on ? LOW : HIGH);
        }
    }
}