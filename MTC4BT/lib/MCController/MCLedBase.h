#pragma once

class MCLedBase
{
public:
    MCLedBase(int led_pin, bool inverted);
    int GetPin();
    void Switch(bool on);
    bool IsOn();
    void Write(bool on);
    virtual void Update(bool ebrakeEnabled) = 0;

private:
    int _led_pin;
    bool _inverted;
    bool _on;

    // The following classes can access private members of MCLedBase.
    friend class MCLed;
    friend class MCStatusLed;
};