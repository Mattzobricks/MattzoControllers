#include "BLEHubChannelController.h"

BLEHubChannelController::BLEHubChannelController(MCChannelConfig *config)
    : MCChannelController(config) {}

BLEHubChannel BLEHubChannelController::GetChannel()
{
    return bleHubChannelMap()[_config->GetChannel()->GetAddress()];
}