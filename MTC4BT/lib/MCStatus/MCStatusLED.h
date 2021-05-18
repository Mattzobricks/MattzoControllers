#pragma once

#include "MCLed.h"

enum MCConnectionStatus
{
    uninitialized = 0x0,
    initializing,
    connecting_wifi,
    connecting_mqtt,
    connected
};

class MCStatusLED : MCLed
{
public:
    MCStatusLED(int led_pin, bool inverted);
    void UpdateByStatus();

private:
    MCConnectionStatus getConnectionStatus();
    bool _statusLEDState;
};