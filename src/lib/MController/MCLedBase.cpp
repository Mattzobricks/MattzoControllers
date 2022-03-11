#include <Arduino.h>

#include "MCLed.h"

// Global PWM properties.
const int pwmMinPwr = 0;
const int pwmMaxPwr = 255;
const int freq = 4000;
const int resolution_bits = 8;

MCLedBase::MCLedBase(int pwmChannel, int pin, bool inverted)
    : _pwmChannel{pwmChannel}, _pin{pin}, _inverted{inverted}
{
    init();
}

// Returns the pin number the LED is attached to.
int MCLedBase::GetPin()
{
    return _pin;
}

void MCLedBase::SetCurrentPwrPerc(int16_t pwrPerc)
{
    // Control pin polarity.
    // int pinValue = digitalRead(_pin);
    // if (pinValue != (_inverted ? HIGH : LOW))
    // {
    //     digitalWrite(_pin, _inverted ? HIGH : LOW);
    // }

    ledcWrite(_pwmChannel, mapPwrPercToRaw(pwrPerc));
}

void MCLedBase::init()
{
    pinMode(_pin, OUTPUT);

    // Initialize channel.
    ledcSetup(_pwmChannel, freq, resolution_bits);

    // Attach pin to channel.
    ledcAttachPin(_pin, _pwmChannel);

    // Initialize the pin so it's off.
    digitalWrite(_pin, _inverted ? HIGH : LOW);
}

int16_t MCLedBase::mapPwrPercToRaw(int pwrPerc)
{
    // Map absolute perc (no matter the direction) to raw channel pwr.
    return map(abs(pwrPerc), 0, 100, pwmMinPwr, pwmMaxPwr);
}