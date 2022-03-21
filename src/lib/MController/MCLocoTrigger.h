#pragma once

#include <Arduino.h>
#include "enums.h"

// Loco event trigger
class MCLocoTrigger
{
public:
    MCLocoTrigger(MCTriggerSource source, std::string eventType, std::string eventId, std::string value, int8_t delayInMs = 0);

    // Returns a boolean value indicating whether the given trigger details match this trigger.
    bool Matches(MCTriggerSource source, std::string eventType, std::string eventId, std::string value);

private:
    // Holds the source that initialized the trigger (Loco, RocRail, ...).
    MCTriggerSource _source;

    // Holds the event that occurred (fnchanged/dirchanged/velochanged).
    std::string _eventType;

    // Holds the event identifier of the trigger (e.g. 0 or 2 for fn0 or fn2).
    std::string _eventId;

    // Holds the value of the event trigger (on/off, forward/backward/stop, accelerating/decelerating, ...).
    std::string _value;

    // Holds the delay in milliseconds between receiving the trigger and executing any related actions.
    int8_t _delayInMs;
};