#include "BLELocomotive.h"
#include "MCLightController.h"
#include "MController.h"
#include "PUHub.h"
#include "SBrickHub.h"
#include "log4MC.h"

BLELocomotive::BLELocomotive(BLELocomotiveConfiguration *config, MController *controller)
    : _config{config}, _controller{controller}
{
    initHubs();
}

bool BLELocomotive::IsEnabled()
{
    return _config->_enabled;
}

bool BLELocomotive::AllHubsConnected()
{
    if (!IsEnabled()) {
        return false;
    }

    for (BLEHub *hub : Hubs) {
        if (!hub->IsConnected()) {
            return false;
        }
    }

    return true;
}

void BLELocomotive::Drive(const int16_t minSpeed, const int16_t pwrPerc)
{
    if (!AllHubsConnected()) {
        // Ignore drive command.
        log4MC::vlogf(LOG_INFO, "Loco: %s ignored drive command because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    for (BLEHub *hub : Hubs) {
        int16_t currentPwrPerc = hub->GetCurrentDrivePwrPerc();
        hub->Drive(minSpeed, pwrPerc);

        if (currentPwrPerc == 0 && pwrPerc > 0) {
            // If we go from stand still (0%) to moving forward (> 0%), we trigger this event because we must possibly handle it.
            TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "forward");
        }

        if (currentPwrPerc == 0 && pwrPerc < 0) {
            // If we go from stand still (0%) to moving backward (< 0%), we trigger this event because we must possibly handle it.
            TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "backward");
        }

        if (currentPwrPerc != 0 && pwrPerc == 0) {
            // If we go from moving (not 0%) to stand still (0%), we trigger this event because we must possibly handle it.
            TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "stopped");
        }
    }
}

void BLELocomotive::TriggerEvent(MCTriggerSource source, std::string eventType, std::string eventId, std::string value)
{
    for (MCLocoEvent *event : _config->_events) {
        if (event->HasTrigger(source, eventType, eventId, value)) {
            for (MCLocoAction *action : event->GetActions()) {
                ChannelType portType = action->GetChannel()->GetChannelType();
                switch (portType) {
                case ChannelType::BleHubChannel: {
                    // Ask hub to execute action.
                    BLEHub *hub = getHubByAddress(action->GetChannel()->GetParentAddress());
                    if (hub) {
                        hub->Execute(action);
                    }
                    break;
                }
                case ChannelType::EspPinChannel: {
                    // Ask controller to execute action.
                    _controller->Execute(action);
                    break;
                }
                }
            }
        }
    }
}

void BLELocomotive::BlinkLights(int durationInMs)
{
    if (!AllHubsConnected()) {
        // Ignore blink request.
        // log4MC::vlogf(LOG_INFO, "Loco: %s ignored blink lights request because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    // Handle blink on lights attached to channels of our hubs.
    for (BLEHub *hub : Hubs) {
        hub->BlinkLights(durationInMs);
    }
}

void BLELocomotive::SetEmergencyBrake(const bool enabled)
{
    // Handle e-brake on all channels of our hubs.
    for (BLEHub *hub : Hubs)
    {
        hub->SetEmergencyBrake(enabled);
    }
}

std::string BLELocomotive::GetLocoName()
{
    return _config->_name;
}

uint BLELocomotive::GetLocoAddress()
{
    return _config->_address;
}

uint BLELocomotive::GetHubCount()
{
    return Hubs.size();
}

BLEHub *BLELocomotive::GetHub(uint index)
{
    return Hubs.at(index);
}

void BLELocomotive::initHubs()
{
    for (BLEHubConfiguration *hubConfig : _config->_hubs)
    {
        BLEHub *hub;

        switch (hubConfig->HubType)
        {
        case BLEHubType::SBrick:
            hub = new SBrickHub(hubConfig);
            break;
        case BLEHubType::PU:
            hub = new PUHub(hubConfig);
            break;
        }

        if (hub)
        {
            hub->SetConnectCallback([this](bool connected) -> void
                                    { handleConnectCallback(connected); });
            Hubs.push_back(hub);
        }
    }

    // log4MC::vlogf(LOG_INFO, "Loco: %s hub config initialized.", _config->_name.c_str());
}

void BLELocomotive::handleConnectCallback(bool connected)
{
    if (connected)
    {
        // log4MC::info("connected");
        if (AllHubsConnected())
        {
            // All hubs are connected. We can lift the manual brake.
            this->setManualBrake(false);
        }
    }
    else
    {
        // At least one hub is disconnected, so we activate the manual brake.
        // log4MC::info("disconnected");
        this->setManualBrake(true);
    }
}

void BLELocomotive::setManualBrake(const bool enabled)
{
    // Handle manual brake on all channels of our hubs.
    for (BLEHub *hub : Hubs)
    {
        hub->SetManualBrake(enabled);
    }
}

BLEHub *BLELocomotive::getHubByAddress(std::string address)
{
    for (BLEHub *hub : Hubs) {
        if (hub->GetRawAddress().compare(address) == 0) {
            return hub;
        }
    }

    return nullptr;
}