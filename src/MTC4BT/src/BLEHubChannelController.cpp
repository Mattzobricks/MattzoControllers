#include "BLEHubChannelController.h"

BLEHubChannelController::BLEHubChannelController(MCChannelConfig *config)
	: MCChannelController(config) {}

BLEHubChannel BLEHubChannelController::GetHubChannel()
{
	return bleHubChannelMap()[_config->GetChannel()->GetAddress()];
}