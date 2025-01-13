#include "MCLocoTrigger.h"

MCLocoTrigger::MCLocoTrigger(MCTriggerSource source, std::string eventType, std::string eventId, std::string value, int8_t delayInMs)
	: _source{source}, _eventType{eventType}, _eventId{eventId}, _value{value}, _delayInMs{delayInMs} {}

bool MCLocoTrigger::Matches(MCTriggerSource source, std::string eventType, std::string eventId, std::string value)
{
	return _source == source && _eventType.compare(eventType) == 0 && _eventId.compare(eventId) == 0 && _value.compare(value) == 0;
}