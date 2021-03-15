#include <Arduino.h>

#include "SBrickHubChannel.h"

SBrickHubChannel::SBrickHubChannel(SBrickChannel channel)
{
  _channel = channel;
  _targetSpeed = 0;
  _currentSpeed = 0;
}

int16_t SBrickHubChannel::GetTargetSpeed() {
  return _targetSpeed;
}

void SBrickHubChannel::SetTargetSpeed(int16_t speed)
{
  if (speed < MIN_CHANNEL_SPEED)
  {
    _targetSpeed = MIN_CHANNEL_SPEED;
    return;
  }

  if (speed > MAX_CHANNEL_SPEED)
  {
    _targetSpeed = MAX_CHANNEL_SPEED;
    return;
  }

  _targetSpeed = speed;
}

void SBrickHubChannel::SetSpeed(int16_t speed)
{
  _currentSpeed = speed;
}

uint16_t SBrickHubChannel::GetCurrentSpeed()
{
  return _currentSpeed;
}

uint8_t SBrickHubChannel::GetAbsCurrentSpeed()
{
  return abs(_currentSpeed);
}

void SBrickHubChannel::SetCurrentSpeed(int16_t speed)
{
  // We are not allowed to go beyond the set target speed.
  // TODO: Causes issue where speed is adjusted immediately when decelarating!!!
  if ((_targetSpeed < 0 && speed < _targetSpeed && speed < _currentSpeed) ||
      (_targetSpeed >= 0 && speed > _targetSpeed && speed > _currentSpeed))
  {
    _currentSpeed = _targetSpeed;
    return;
  }

  // We can never go beyond the SBrick min speed.
  if (speed < SBrickHubChannel::MIN_CHANNEL_SPEED)
  {
    _currentSpeed = SBrickHubChannel::MIN_CHANNEL_SPEED;
    return;
  }

  // We can never go beyond the SBrick max speed.
  if (speed > SBrickHubChannel::MAX_CHANNEL_SPEED)
  {
    _currentSpeed = SBrickHubChannel::MAX_CHANNEL_SPEED;
    return;
  }

  _currentSpeed = speed;
}

bool SBrickHubChannel::GetTargetDirection()
{
  return _targetSpeed >= 0;
}

bool SBrickHubChannel::GetCurrentDirection()
{
  return _currentSpeed >= 0;
}

bool SBrickHubChannel::IsAtTargetSpeed()
{
  return _currentSpeed == _targetSpeed;
}