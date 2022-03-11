#include "MCLocoEvent.h"

MCLocoEvent::MCLocoEvent(std::vector<MCLocoTrigger *> triggers, std::vector<MCLocoAction *> actions)
    : _triggers{triggers}, _actions{actions} {}

bool MCLocoEvent::HasTrigger(MCTriggerSource source, std::string eventType, std::string eventId, std::string value)
{
    for (MCLocoTrigger *trigger : _triggers)
    {
        if (trigger->Matches(source, eventType, eventId, value))
        {
            return true;
        }
    }

    return false;
}

std::vector<MCLocoAction *> MCLocoEvent::GetActions()
{
    return _actions;
}