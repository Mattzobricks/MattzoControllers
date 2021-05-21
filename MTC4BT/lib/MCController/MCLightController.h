#include <Arduino.h>

class MCLightController
{
public:
    // Two flashes per second.
    static bool TwoFlashesPerSecond();
    
    // One short flash per second (on 10%).
    static bool OneFlashPerSecond();

    // Blink (on 50%).
    static bool Blink();

private:
    static bool on(uint flashesPerSecond, uint flashDurationInMs);
};