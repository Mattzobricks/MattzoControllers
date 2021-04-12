#include <Arduino.h>

#include "ChannelController.h"

#define MIN_SPEED_PERC -100
#define MAX_SPEED_PERC 100

ChannelController::ChannelController(HubChannel channel, HubChannelDirection direction, AttachedDevice device, int16_t speedStep, int16_t brakeStep)
{
  _channel = channel;
  _direction = direction;
  _device = device;
  _speedStep = speedStep;
  _brakeStep = brakeStep;

  _targetSpeedPerc = 0;
  _currentSpeedPerc = 0;
}

HubChannel ChannelController::GetChannel()
{
  return _channel;
}

AttachedDevice ChannelController::GetAttachedDevice()
{
  return _device;
}

void ChannelController::SetTargetSpeedPerc(int16_t speedPerc)
{
  int16_t newSpeedPerc = _direction == HubChannelDirection::REVERSE ? speedPerc * -1 : speedPerc;
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
    // No need to update current speed if we've already reached target speed.
    return false;
  }

  int16_t dirMultiplier = _targetSpeedPerc > _currentSpeedPerc ? 1 : -1;
  int16_t speedStep = (isAccelarating() ? _speedStep : _brakeStep) * dirMultiplier;
  int16_t newSpeedPerc;

  if (!isAccelarating() && abs(speedStep) > abs(_currentSpeedPerc))
  {
    // We can't switch from one drive direction to the other directly. We must stop first.
    newSpeedPerc = 0;
  }
  else
  {
    // Adjust speed with one step.
    newSpeedPerc = _currentSpeedPerc + speedStep;
  }

  // Serial.print(_channel);
  // Serial.print(": curspd=");
  // Serial.print(_currentSpeedPerc);
  // Serial.print(" tarspd=");
  // Serial.print(_targetSpeedPerc);
  // Serial.print(" step=");
  // Serial.print(speedStep);
  // Serial.print(" newspd=");
  // Serial.print(newSpeedPerc);
  // Serial.println();

  // We are not allowed to go beyond the set target speed.
  if ((_targetSpeedPerc < 0 && newSpeedPerc < _targetSpeedPerc && newSpeedPerc < _currentSpeedPerc) ||
      (_targetSpeedPerc >= 0 && newSpeedPerc > _targetSpeedPerc && newSpeedPerc > _currentSpeedPerc))
  {
    _currentSpeedPerc = _targetSpeedPerc;
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