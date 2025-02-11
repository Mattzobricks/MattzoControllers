#pragma once

#include <Arduino.h>

#include "MCChannel.h"

// Loco action
class MCLocoAction
{
  public:
	MCLocoAction(MCChannel *channel, int16_t targetPerc, HubLedColor color);

	MCChannel *GetChannel();

	int16_t GetTargetPowerPerc();

	HubLedColor GetColor();

  private:
	// Holds the target channel.
	MCChannel *_channel;

	// Holds the target power percentage to apply to the port.
	int16_t _targetPerc;

	// Holds the requested color for the onboard hub's LED.
	HubLedColor _hubLedColor = HubLedColor::NONE;
};