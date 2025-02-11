#include "MCLocoAction.h"

MCLocoAction::MCLocoAction(MCChannel *channel, int16_t targetPerc, HubLedColor color)
	: _channel{channel}, _targetPerc{targetPerc}, _hubLedColor{color} {}

MCChannel *MCLocoAction::GetChannel()
{
	return _channel;
}

int16_t MCLocoAction::GetTargetPowerPerc()
{
	return _targetPerc;
}

HubLedColor MCLocoAction::GetColor()
{
	return _hubLedColor;
}