#include "MCPinController.h"

MCPinController::MCPinController(MCChannelConfig *config)
    : MCChannelController(config) {}

int MCPinController::GetEspPinNumber()
{
    return _config->GetChannel()->GetAddressAsEspPinNumber();
}