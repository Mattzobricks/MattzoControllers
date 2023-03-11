#include <Arduino.h>

#include "MCChannelController.h"
#include "MCLightController.h"

#define MIN_PWR_PERC -100
#define MAX_PWR_PERC 100

MCChannelController::MCChannelController(MCChannelConfig *config)
{
    _config = config;

    // Initially activate manual brake;
    _mbrake = true;
    _ebrake = false;
    _blinkUntil = 0;
    _minPwrPerc = 0;
    _targetPwrPerc = 0;
    _currentPwrPerc = 0;
}

HubLedColor MCChannelController::GetHubLedColor()
{
    return _hubLedColor;
}

void MCChannelController::SetHubLedColor(HubLedColor color)
{
    _hubLedColor = color;
}

DeviceType MCChannelController::GetAttachedDevice()
{
    return _config->GetAttachedDeviceType();
}

MCChannel *MCChannelController::GetChannel()
{
    return _config->GetChannel();
}

void MCChannelController::SetMinPwrPerc(int16_t minPwrPerc)
{
    _minPwrPerc = minPwrPerc;
}

int16_t MCChannelController::GetTargetPwrPerc()
{
    return _targetPwrPerc;
}

void MCChannelController::SetTargetPwrPerc(int16_t targetPwrPerc)
{
    _targetPwrPerc = normalizePwrPerc(_config->IsInverted()
                                          ? targetPwrPerc * -1
                                          : targetPwrPerc);
}

int16_t MCChannelController::GetCurrentPwrPerc()
{
    if (_ebrake || _mbrake)
    {
        switch (GetAttachedDevice())
        {
        case DeviceType::Light:
        {
            // Force blinking lights (50% when on, 0% when off) when e-brake is enabled.
            return MCLightController::Blink() ? 50 : 0;
        }
        case DeviceType::Motor: {
            // Force motor off (0%).
            return 0;
        }
        }
    }

    if (_blinkUntil > millis() && GetAttachedDevice() == DeviceType::Light) {
        // Force blinking lights (50% when on, 0% when off) when requested.
        return MCLightController::Blink() ? 50 : 0;
    }

    return _currentPwrPerc;
}

void MCChannelController::SetCurrentPwrPerc(int16_t currentPwrPerc)
{
    _currentPwrPerc = normalizePwrPerc(currentPwrPerc);
}

bool MCChannelController::UpdateCurrentPwrPerc()
{
    unsigned long timeStamp = millis();

    if (_ebrake || _mbrake)
    {
        // Update of current pwr required (directly to zero), if we're e-braking.
        return true;
    }

    // Update timestamp of last update.
    _lastUpdate = timeStamp;

    if (isAtTargetPwrPerc()) {
        // No need to update current pwr, if we've already reached target pwr.
        return false;
    }

    int16_t dirMultiplier = _targetPwrPerc > _currentPwrPerc ? 1 : -1;
    int16_t pwrStep = (isAccelarating() ? _config->GetPwrIncStep() : _config->GetPwrDecStep()) * dirMultiplier;

    if (!isAccelarating() && abs(pwrStep) > abs(_currentPwrPerc)) {
        // We can't switch from one drive direction to the other directly. Enforce stop first.
        _currentPwrPerc = 0;
        return true;
    }

    // Adjust pwr with one step.
    int16_t newPwrPerc = _currentPwrPerc + pwrStep;

    // We are not allowed to go beyond the set target pwr. Enforce target pwr, if we would.
    if ((_targetPwrPerc < 0 && newPwrPerc < _targetPwrPerc && newPwrPerc < _currentPwrPerc) ||
        (_targetPwrPerc >= 0 && newPwrPerc > _targetPwrPerc && newPwrPerc > _currentPwrPerc) ||
        (_targetPwrPerc < 0 && newPwrPerc > _targetPwrPerc && newPwrPerc > _currentPwrPerc) ||
        (_targetPwrPerc >= 0 && newPwrPerc < _targetPwrPerc && newPwrPerc < _currentPwrPerc)) {
        _currentPwrPerc = _targetPwrPerc;
        return true;
    }

    if (abs(newPwrPerc) < _minPwrPerc) {
        // New pwr is slower than min pwr, force to min pwr or stop immediately (dependend on wether we're accelarating or decelerating).
        dirMultiplier = newPwrPerc >= 0 ? 1 : -1;
        _currentPwrPerc = isAccelarating() ? _minPwrPerc * dirMultiplier : 0;
        return true;
    }

    // We haven't reached the target pwr yet.
    _currentPwrPerc = normalizePwrPerc(newPwrPerc);
    return true;
}

uint8_t MCChannelController::GetAbsCurrentPwrPerc()
{
    return abs(_currentPwrPerc);
}

bool MCChannelController::IsDrivingForward()
{
    return _currentPwrPerc >= 0;
}

void MCChannelController::ManualBrake(bool enabled)
{
    _mbrake = enabled;
}

void MCChannelController::EmergencyBrake(bool enabled)
{
    _ebrake = enabled;
}

bool MCChannelController::isAccelarating()
{
    // Current pwr equals target pwr.
    if (isAtTargetPwrPerc()) {
        return false;
    }

    // Current pwr and target pwr in opposite directions means we must be decelerating first.
    if (_currentPwrPerc * _targetPwrPerc < 0) {
        return false;
    }

    // Current pwr and target pwr in same direction.
    // Accelerating if target pwr greater than current pwr, else decelerating.
    return abs(_targetPwrPerc) > abs(_currentPwrPerc);
}

bool MCChannelController::isAtTargetPwrPerc()
{
    return _currentPwrPerc == _targetPwrPerc;
}

int16_t MCChannelController::normalizePwrPerc(int16_t pwrPerc)
{
    // We can never go beyond the absolute min pwr value.
    if (pwrPerc < MIN_PWR_PERC) {
        return MIN_PWR_PERC;
    }

    // We can never go beyond the absolute max pwr value.
    if (pwrPerc > MAX_PWR_PERC) {
        return MAX_PWR_PERC;
    }

    // Given pwr percentage is within the valid boundaries.
    return pwrPerc;
}