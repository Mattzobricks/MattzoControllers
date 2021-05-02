#include <Arduino.h>

#include "ChannelController.h"

#define MIN_SPEED_PERC -100
#define MAX_SPEED_PERC 100

ChannelController::ChannelController(ChannelConfiguration *config)
{
  _config = config;

  _minSpeedPerc = 0;
  _targetSpeedPerc = 0;
  _currentSpeedPerc = 0;
}

HubChannel ChannelController::GetChannel()
{
  return _config->channel;
}

AttachedDevice ChannelController::GetAttachedDevice()
{
  return _config->device;
}

void ChannelController::SetMinSpeedPerc(int16_t minSpeedPerc)
{
  _minSpeedPerc = minSpeedPerc;
}

void ChannelController::SetTargetSpeedPerc(int16_t speedPerc)
{
  int16_t newSpeedPerc = _config->direction == HubChannelDirection::REVERSE ? speedPerc * -1 : speedPerc;
  _targetSpeedPerc = normalizeSpeedPerc(newSpeedPerc);
}

int16_t ChannelController::GetCurrentSpeedPerc()
{
  return _currentSpeedPerc;
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
  int16_t speedStep = (isAccelarating() ? _config->speedStep : _config->brakeStep) * dirMultiplier;

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

  // Serial.print(_channel);
  // Serial.print(": tarspd=");
  // Serial.print(_targetSpeedPerc);
  // Serial.print(" curspd=");
  // Serial.print(_currentSpeedPerc);
  // Serial.print(" step=");
  // Serial.print(speedStep);
  // Serial.print(" newspd=");
  // Serial.print(newSpeedPerc);
  // Serial.print(" minspd=");
  // Serial.print(_minSpeedPerc);
  // Serial.println();

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