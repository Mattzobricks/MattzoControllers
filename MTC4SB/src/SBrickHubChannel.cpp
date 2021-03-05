#include <Arduino.h>

#include "SBrickHubChannel.h"

SBrickHubChannel::SBrickHubChannel(SBrickChannel channel)
{
  _channel = channel;
  _setTargetSpeed = 0;
  _currentTargetSpeed = 0;
}

void SBrickHubChannel::SetTargetSpeed(int16_t speed)
{
  if (speed < MIN_CHANNEL_SPEED)
  {
    _setTargetSpeed = MIN_CHANNEL_SPEED;
    return;
  }

  if (speed > MAX_CHANNEL_SPEED)
  {
    _setTargetSpeed = MAX_CHANNEL_SPEED;
    return;
  }

  _setTargetSpeed = speed;
}

void SBrickHubChannel::SetSpeed(int16_t speed)
{
  _currentTargetSpeed = speed;
}

uint8_t SBrickHubChannel::GetCurrentTargetSpeed()
{
  return abs(_currentTargetSpeed);
}

bool SBrickHubChannel::GetCurrentTargetDirection()
{
  return _setTargetSpeed >= 0;
}

void SBrickHubChannel::SetCurrentTargetSpeed(int16_t speed)
{
  // We are not allowed to go beyond the set target speed.
  if ((_setTargetSpeed < 0 && speed < _setTargetSpeed) ||
      (_setTargetSpeed >= 0 && speed > _setTargetSpeed))
  {
    _currentTargetSpeed = _setTargetSpeed;
    return;
  }

  // We can never go beyond the SBrick min speed.
  if (speed < SBrickHubChannel::MIN_CHANNEL_SPEED)
  {
    _currentTargetSpeed = SBrickHubChannel::MIN_CHANNEL_SPEED;
    return;
  }

  // We can never go beyond the SBrick max speed.
  if (speed > SBrickHubChannel::MAX_CHANNEL_SPEED)
  {
    _currentTargetSpeed = SBrickHubChannel::MAX_CHANNEL_SPEED;
    return;
  }

  _currentTargetSpeed = speed;
}

bool SBrickHubChannel::IsAtSetTargetSpeed()
{
  return _setTargetSpeed == _currentTargetSpeed;
}