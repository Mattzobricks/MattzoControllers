#include <Arduino.h>

#include "MCConfiguration.h"
#include "MattzoWiFiClient.h"
#include "MattzoMQTTSubscriber.h"
#include "MCLedBase.h"

enum MCConnectionStatus
{
    uninitialized = 0,
    initializing,
    connecting_wifi,
    connecting_mqtt,
    connected
};

class MCController
{
public:
    MCController();

    std::vector<MCLedBase *> Leds;

    static MCConnectionStatus GetConnectionStatus();

    void BaseSetup(MCConfiguration *config);

    // Update emergency brake looking at the current controller connection status and controls leds.
    void BaseLoop();

    MCLedBase *GetLed(int pin, bool inverted);
    bool GetEmergencyBrake();
    virtual void EmergencyBrake(const bool enabled) = 0;

private:
    void initStatusLeds();
    std::vector<Fn *> getFunctions(MCFunction f);

    bool _ebrake;
    MCConfiguration *_config;
};