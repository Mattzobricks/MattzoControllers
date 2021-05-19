#pragma once

#include "MCLedBase.h"

enum MCConnectionStatus
{
    uninitialized = 0,
    initializing,
    connecting_wifi,
    connecting_mqtt,
    connected
};

class MCStatusLed : public MCLedBase
{
public:
    MCStatusLed(int led_pin, bool inverted);
    void Update();

private:
    MCConnectionStatus getConnectionStatus();
};