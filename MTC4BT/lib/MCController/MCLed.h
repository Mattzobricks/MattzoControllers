#pragma once

class MCLed
{
public:
    MCLed(int led_pin, bool inverted);
    int GetPin();
    void Switch(bool on);
    void Update();

private:
    int _led_pin; 
    bool _inverted;
    bool _on;
};