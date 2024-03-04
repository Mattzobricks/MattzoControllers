#include "MController.h"
#include "MCChannelConfig.h"
#include "MCLed.h"
#include "MCLightController.h"
#include "MCLocoAction.h"
#include "MCStatusLed.h"
#include "log4MC.h"

MController::MController()
{
}

MCConnectionStatus MController::GetConnectionStatus()
{
    if (MattzoWifiClient::GetStatus() == WL_UNINITIALIZED) {
        return MCConnectionStatus::uninitialized;
    }

    if (MattzoWifiClient::GetStatus() == WL_INITIALIZING) {
        return MCConnectionStatus::initializing;
    }

    if (MattzoWifiClient::GetStatus() != WL_CONNECTED) {
        return MCConnectionStatus::connecting_wifi;
    }

    if (MattzoMQTTSubscriber::GetStatus() != MQTT_CONNECTED) {
        return MCConnectionStatus::connecting_mqtt;
    }

    return MCConnectionStatus::connected;
}

void MController::Setup(MCConfiguration *config)
{
    // Setup controller configuration.
    _config = config;
    _ebrake = false;

    // Initialize local channel controllers.
    initChannelControllers();
}

void MController::Loop()
{
    int currentPwrPerc;

    for (MCChannelController *channel : _channelControllers) {
        // Update channel e-brake status.
        channel->EmergencyBrake(GetEmergencyBrake());

        // Update current channel pwr (continue if request was ignored).
        if (!channel->UpdateCurrentPwrPerc()) {
            continue;
        }

        // Get new pwr perc.
        currentPwrPerc = channel->GetCurrentPwrPerc();

        if (channel->GetAttachedDevice() == DeviceType::Light) {
            MCLedBase *led = findLedByPinNumber(channel->GetChannel()->GetAddressAsEspPinNumber());
            if (led) {
                led->SetCurrentPwrPerc(currentPwrPerc);
            }
        }

        if (channel->GetAttachedDevice() == DeviceType::StatusLight) {
            MCLedBase *led = findLedByPinNumber(channel->GetChannel()->GetAddressAsEspPinNumber());
            if (led) {
                switch (GetConnectionStatus()) {
                case uninitialized:
                case initializing: {
                    // Two flashes per second.
                    currentPwrPerc = MCLightController::TwoFlashesPerSecond() ? 100 : 0;
                    break;
                }
                case MCConnectionStatus::connecting_wifi: {
                    // One short flash per second (on 10%).
                    currentPwrPerc = MCLightController::OneFlashPerSecond() ? 100 : 0;
                    break;
                }
                case MCConnectionStatus::connecting_mqtt: {
                    // Blink (on 50%).
                    currentPwrPerc = MCLightController::Blink() ? 100 : 0;
                    break;
                }
                case connected: {
                    // Off.
                    currentPwrPerc = 0;
                    break;
                }
                };

                led->SetCurrentPwrPerc(currentPwrPerc);
            }
        }
    }
}

bool MController::GetEmergencyBrake()
{
    // E-brake is enabled when specifically requested (through MQTT) or when the controller is not connected.
    return _ebrake || GetConnectionStatus() != MCConnectionStatus::connected;
}

void MController::SetEmergencyBrake(const bool enabled)
{
    _ebrake = enabled;
}

void MController::Execute(MCLocoAction *action)
{
    MCChannelController *channel = findControllerByChannel(action->GetChannel());

    if (channel) {
        channel->SetTargetPwrPerc(action->GetTargetPowerPerc());
    }
}

void MController::initChannelControllers()
{
    // TODO: This method should be made more robust to prevent config errors, like configuring the same channel twice.

    for (int i = 0; i < _config->EspPins.size(); i++) {
        if (i >= 16) {
            // There are only 16 PWM channels available, so we must ignore the rest.
            log4MC::warn("CTRL: Local channel initialization failed (too many ESP pins configured, max. is 16)!");
            break;
        }

        MCChannelConfig *espPinConfig = _config->EspPins.at(i);
        if (espPinConfig->GetAttachedDeviceType() == DeviceType::Light) {
            initLed(i, espPinConfig->GetChannel()->GetAddressAsEspPinNumber(), espPinConfig->IsInverted());
        }

        if (espPinConfig->GetAttachedDeviceType() == DeviceType::StatusLight) {
            initStatusLed(i, espPinConfig->GetChannel()->GetAddressAsEspPinNumber());
        }

        _channelControllers.push_back(new MCChannelController(espPinConfig));
    }

    log4MC::info("CTRL: Local channels initialized.");
}

MCChannelController *MController::findControllerByChannel(MCChannel *channel)
{
    for (MCChannelController *controller : _channelControllers) {
        if (controller->GetChannel() == channel) {
            return controller;
        }
    }

    return nullptr;
}

MCLedBase *MController::findLedByPinNumber(int pin)
{
    for (MCLedBase *led : _espLeds) {
        if (led->GetPin() == pin) {
            return led;
        }
    }

    return nullptr;
}

void MController::initLed(int pwmChannel, int pin, bool inverted)
{
    if (findLedByPinNumber(pin) != nullptr) {
        return;
    }

    // If not found, define, initialize and add a new LED.
    _espLeds.push_back(new MCLed(pwmChannel, pin, inverted));
}

void MController::initStatusLed(int pwmChannel, int pin)
{
    if (findLedByPinNumber(pin) != nullptr) {
        return;
    }

    // If not found, define, initialize and add a new status LED.
    _espLeds.push_back(new MCStatusLed(pwmChannel, pin, false));
}
