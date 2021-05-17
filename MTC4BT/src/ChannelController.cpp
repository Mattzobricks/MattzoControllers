#include <Arduino.h>

#include "ChannelController.h"

#define MIN_SPEED_PERC -100
#define MAX_SPEED_PERC 100

ChannelController::ChannelController(DeviceConfiguration *config, int16_t speedStep, int16_t brakeStep)
{
  _config = config;
  _speedStep = speedStep;
  _brakeStep = brakeStep;

  _minSpeedPerc = 0;
  _targetSpeedPerc = 0;
  _currentSpeedPerc = 0;
}

HubChannel ChannelController::GetChannel()
{
  return _config->GetAddressAsHubChannel();
}

AttachedDevice ChannelController::GetAttachedDevice()
{
  return _config->GetAttachedDevice();
}

void ChannelController::SetMinSpeedPerc(int16_t minSpeedPerc)
{
  _minSpeedPerc = minSpeedPerc;
}

void ChannelController::SetTargetSpeedPerc(int16_t speedPerc)
{
  int16_t newSpeedPerc = _config->IsInverted() ? speedPerc * -1 : speedPerc;
  _targetSpeedPerc = normalizeSpeedPerc(newSpeedPerc);
}

int16_t ChannelController::GetCurrentSpeedPerc()
{
  return _currentSpeedPerc;
}

void ChannelController::SetCurrentSpeedPerc(int16_t speedPerc)
{
  _currentSpeedPerc = speedPerc;
}

uint8_t ChannelController::GetAbsCurrentSpeedPerc()
{
  return abs(_currentSpeedPerc);
}

bool ChannelController::IsDrivingForward()
{
  return _currentSpeedPerc >= 0;
}

bool ChannelController::UpdateCurrentSpeedPerc()
{
  if (isAtTargetSpeedPerc())
  {
    // No need to update current speed, if we've already reached target speed.
    return false;
  }

  int16_t dirMultiplier = _targetSpeedPerc > _currentSpeedPerc ? 1 : -1;
  int16_t speedStep = (isAccelarating() ? _speedStep : _brakeStep) * dirMultiplier;

  if (!isAccelarating() && abs(speedStep) > abs(_currentSpeedPerc))
  {
    // We can't switch from one drive direction to the other directly. Force stop first.
    _currentSpeedPerc = 0;
    return true;
  }

  // Adjust speed with one step.
  int16_t newSpeedPerc = _currentSpeedPerc + speedStep;

  // We are not allowed to go beyond the set target speed. Enforce target speed, if we would.
  if ((_targetSpeedPerc < 0 && newSpeedPerc < _targetSpeedPerc && newSpeedPerc < _currentSpeedPerc) ||
      (_targetSpeedPerc >= 0 && newSpeedPerc > _targetSpeedPerc && newSpeedPerc > _currentSpeedPerc) ||
      (_targetSpeedPerc < 0 && newSpeedPerc > _targetSpeedPerc && newSpeedPerc > _currentSpeedPerc) ||
      (_targetSpeedPerc >= 0 && newSpeedPerc < _targetSpeedPerc && newSpeedPerc < _currentSpeedPerc))
  {
    _currentSpeedPerc = _targetSpeedPerc;
    return true;
  }

  if (abs(newSpeedPerc) < _minSpeedPerc)
  {
    // New speed is slower than min speed, force to min speed or stop immediately (dependend on wether we're accelarating or decelerating).
    dirMultiplier = newSpeedPerc >= 0 ? 1 : -1;
    _currentSpeedPerc = isAccelarating() ? _minSpeedPerc * dirMultiplier : 0;
    return true;
  }

  // We haven't reached the target speed yet.
  _currentSpeedPerc = normalizeSpeedPerc(newSpeedPerc);
  return true;
}

void ChannelController::EmergencyBreak()
{
  _currentSpeedPerc = 0;
}

bool ChannelController::isAccelarating()
{
  // Current speed equals target speed.
  if (isAtTargetSpeedPerc())
    return false;

  // Current speed and target speed in opposite directions means we must be decelerating first.
  if (_currentSpeedPerc * _targetSpeedPerc < 0)
    return false;

  // Current speed and target speed in same direction.
  return abs(_targetSpeedPerc) > abs(_currentSpeedPerc);
}

bool ChannelController::isAtTargetSpeedPerc()
{
  return _currentSpeedPerc == _targetSpeedPerc;
}

int16_t ChannelController::normalizeSpeedPerc(int16_t speedPerc)
{
  // We can never go beyond the absolute min speed value.
  if (speedPerc < MIN_SPEED_PERC)
  {
    return MIN_SPEED_PERC;
  }

  // We can never go beyond the absolute max speed value.
  if (speedPerc > MAX_SPEED_PERC)
  {
    return MAX_SPEED_PERC;
  }

  // Given speed percentage is within the valid boundaries.
  return speedPerc;
}