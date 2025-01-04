/*
 * BLELocomotive
 */

#include "BLELocomotive.h"
#include "BuWizz2Hub.h"
#include "MCLightController.h"
#include "MController.h"
#include "PUHub.h"
#include "PURemote.h"
#include "SBrickHub.h"
#include "log4MC.h"

BLELocomotive::BLELocomotive(BLELocomotiveConfiguration *config, MController *controller)
	: BLEbaseDevice(controller), _config{config}
{
	initHubs();
}

bool BLELocomotive::AllHubsConnected()
{
	for (BLEHub *hub : Hubs) {
		if (!hub->IsConnected()) {
			return false;
		}
	}

	return true;
}

void BLELocomotive::Drive(const int16_t minSpeed, const int16_t pwrPerc)
{
	for (BLEHub *hub : Hubs) {
		int16_t currentPwrPerc = hub->GetCurrentDrivePwrPerc();
		hub->Drive(minSpeed, pwrPerc);

		if ((currentPwrPerc < 0 && pwrPerc > 0) ||
			(currentPwrPerc > 0 && pwrPerc < 0) ||
			(currentPwrPerc != 0 && pwrPerc == 0)) {
			// If we go from moving backward (< 0%) to moving forward (> 0%) or
			// if we go from moving forward (> 0) to moving backward (< 0%) or
			// if we go from moving (not 0%) to stand still (0%)
			// THEN we trigger the "stopped" event.
			TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "stopped");
		}

		if (currentPwrPerc <= 0 && pwrPerc > 0) {
			// If we go from moving backward (< 0%) or stand still (0%) to moving forward (> 0%)
			// THEN we trigger the "forward" event.
			TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "forward");
		}

		if (currentPwrPerc >= 0 && pwrPerc < 0) {
			// If we go from moving forward (> 0) or stand still (0%) to moving backward (< 0%)
			// THEN we trigger the "backward" event.
			TriggerEvent(MCTriggerSource::Loco, "dirchanged", "", "backward");
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

void BLELocomotive::SetEmergencyBrake(const bool enabled)
{
	// Handle e-brake on all channels of our hubs.
	for (BLEHub *hub : Hubs) {
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

void BLELocomotive::initHubs()
{
	for (BLEHubConfiguration *hubConfig : _config->_hubs) {
		BLEHub *hub;

		switch (hubConfig->HubType) {
		case BLEHubType::SBrick:
			hub = new SBrickHub(hubConfig);
			break;
		case BLEHubType::PU:
			hub = new PUHub(hubConfig);
			break;
		case BLEHubType::BuWizz2:
			hub = new BuWizz2Hub(hubConfig);
			break;
		case BLEHubType::PUController:
			hub = new PURemote(hubConfig);
			break;
		}

		if (hub) {
			hub->SetConnectCallback([this](bool connected) -> void { handleConnectCallback(connected); });
			Hubs.push_back(hub);
		}
	}

	// log4MC::vlogf(LOG_INFO, "Loco: %s hub config initialized.", _config->_name.c_str());
}

void BLELocomotive::handleConnectCallback(bool connected)
{
	if (connected) {
		// log4MC::info("connected");
		if (AllHubsConnected()) {
			// All hubs are connected. We can lift the manual brake.
			this->setManualBrake(false);
		}
	} else {
		// At least one hub is disconnected, so we activate the manual brake.
		// log4MC::info("disconnected");
		this->setManualBrake(true);
	}
}

void BLELocomotive::setManualBrake(const bool enabled)
{
	// Handle manual brake on all channels of our hubs.
	for (BLEHub *hub : Hubs) {
		hub->SetManualBrake(enabled);
	}
}