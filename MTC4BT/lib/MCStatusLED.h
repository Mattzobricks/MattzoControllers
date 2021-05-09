#pragma once

// #include <MattzoWifiClient.h>
//#include <MattzoMQTT.h>
#include <MCConfig.h>

enum MCConnectionStatus {
    connecting_wifi = 0x0,    connecting_mqtt, connected, noconfig
};

class MCStatusLED {
    public:
        MCStatusLED(int led_pin, bool inverted);
        void updateStatusLED();
        MCConnectionStatus getConnectionStatus();
    private:
//        MCConnectionStatus getConnectionStatus();
        void setStatusLED(bool ledState);
        int status_led_pin;
        bool  statusLEDState;
        bool inverted;
};