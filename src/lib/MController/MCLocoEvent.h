#pragma once

#include "MCLocoTrigger.h"
#include "MCLocoAction.h"
#include <vector>

// Loco event
class MCLocoEvent
{
public:
    MCLocoEvent(std::vector<MCLocoTrigger *> triggers, std::vector<MCLocoAction *> actions);

    bool HasTrigger(MCTriggerSource source, std::string eventType, std::string eventId, std::string value);

    // Returns a list of actions to execute when triggered by this event.
    std::vector<MCLocoAction *> GetActions();

private:
    std::vector<MCLocoTrigger *> _triggers;
    std::vector<MCLocoAction *> _actions;
};