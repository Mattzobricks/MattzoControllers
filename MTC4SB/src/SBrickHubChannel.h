#pragma once

#include <Arduino.h>

class SBrickHubChannel
{
public:
  static const int16_t MIN_CHANNEL_SPEED = -254;
  static const int16_t MAX_CHANNEL_SPEED = 254;

  enum SBrickChannel
  {
    A,
    B,
    C,
    D
  };

  SBrickHubChannel(SBrickChannel channel);
  
  int16_t GetTargetSpeed();
  void SetTargetSpeed(int16_t speed);
  void SetSpeed(int16_t speed);
  uint16_t GetCurrentSpeed();
  uint8_t GetAbsCurrentSpeed();
  void SetCurrentSpeed(int16_t speed);
  bool GetTargetDirection();
  bool GetCurrentDirection();
  bool IsAtTargetSpeed();

private:
  SBrickChannel _channel;
  int16_t _targetSpeed;
  int16_t _currentSpeed;
};