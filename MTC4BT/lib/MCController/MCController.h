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
    MCController(MCConfiguration *config);

    std::vector<MCLedBase *> Leds;

    static MCConnectionStatus GetConnectionStatus();

    void Setup();
    MCLedBase *GetLed(int pin, bool inverted);
    bool GetEmergencyBreak();
    virtual void EmergencyBreak(const bool enabled) = 0;

private:
    void initStatusLeds();
    static void loop(void *parm);
    std::vector<Fn *> getFunctions(MCFunction f);

    bool _ebrake;
    MCConfiguration *_config;
};