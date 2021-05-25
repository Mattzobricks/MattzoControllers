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

    void Setup(MCConfiguration *config);

    // Update emergency brake looking at the current controller connection status and controls leds.
    void Loop();

    // Return an led instance for the requested pin, or creates one if it doesn't exist yet.
    MCLedBase *GetLed(int pin, bool inverted);

    // Returns a boolean value indicating whether the e-brake flag is currently set or not.
    bool GetEmergencyBrake();

    // Sets the emergency brake flag to the given value.
    void SetEmergencyBrake(const bool enabled);

    // Handles the given function.
    void HandleFn(Fn *fn, const bool on);

    // Abstract method required for derived controller implementations to handle e-brake.
    virtual void HandleEmergencyBrake(const bool enabled) = 0;    

private:
    void initStatusLeds();
    std::vector<Fn *> getFunctions(MCFunction f);

    bool _ebrake;
    MCConfiguration *_config;
};