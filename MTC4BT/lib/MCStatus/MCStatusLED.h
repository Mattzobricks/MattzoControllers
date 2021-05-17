#pragma once

enum MCConnectionStatus
{
    uninitialized = 0x0,
    initializing,
    connecting_wifi,
    connecting_mqtt,
    connected
};

class MCStatusLED
{
public:
    MCStatusLED(int led_pin, bool inverted);
    void UpdateStatusLED();
    MCConnectionStatus getConnectionStatus();

private:
    void setStatusLED(bool on);
    int _status_led_pin; 
    bool _statusLEDState;
    bool _inverted;
};