#include <Arduino.h>

#include "MCConfiguration.h"
#include "MattzoWifiClient.h"
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

class MController
{
public:
    MController();

    // Returns the current controller connection status. 
    static MCConnectionStatus GetConnectionStatus();

    // Mattzo Controller setup initialization method.
    void Setup(MCConfiguration *config);

    // Updates emergency brake based on the current controller connection status and controls leds.
    void Loop();

    // Returns an led instance for the requested pin, or creates one if it doesn't exist yet.
    MCLedBase *GetLed(int pin, bool inverted);

    // Returns a boolean value indicating whether the e-brake flag is currently set or not.
    bool GetEmergencyBrake();

    // Sets the emergency brake flag to the given value.
    void SetEmergencyBrake(const bool enabled);

    // Handles the given function locally on this controller.
    void HandleFn(Fn *fn, const bool on);

    // Abstract method required for derived controller implementations to handle e-brake.
    virtual void HandleSys(const bool enabled) = 0;

private:
    // Initialized the status leds attached to this controller.
    void initStatusLeds();

    // Returns a list of references to functions of the given type configured for this controller.
    std::vector<Fn *> getFunctions(MCFunction f);

    // List of references to leds attached to this controller.
    std::vector<MCLedBase *> _espLeds;

    // Boolean value indicating whether emergency brake is currently enabled or not.
    bool _ebrake;

    // Reference to the configuration of this controller.
    MCConfiguration *_config;
};