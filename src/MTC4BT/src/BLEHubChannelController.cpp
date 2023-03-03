#include "BLEHubChannelController.h"

BLEHubChannelController::BLEHubChannelController(MCChannelConfig *config)
    : MCChannelController(config) {}

BLEHubChannel BLEHubChannelController::GetHubChannel()
{
    return _config->GetChannel()->GetHubChannel();
}