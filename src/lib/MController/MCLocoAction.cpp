#include "MCLocoAction.h"

MCLocoAction::MCLocoAction(MCChannel *channel, int16_t targetPerc)
    : _channel{channel}, _targetPerc{targetPerc} {}

MCChannel *MCLocoAction::GetChannel()
{
    return _channel;
}

int16_t MCLocoAction::GetTargetPowerPerc()
{
    return _targetPerc;
}