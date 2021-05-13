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
    int status_led_pin; 
    bool statusLEDState;
    bool inverted;
    bool ledOn = false;
    long previousMillis = 0;        // will store last time LED was updated
    long OnTime = 0;
    long OffTime = 0;
};