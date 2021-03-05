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
  
  void SetTargetSpeed(int16_t speed);
  void SetSpeed(int16_t speed);
  uint8_t GetCurrentTargetSpeed();
  void SetCurrentTargetSpeed(int16_t speed);
  bool GetCurrentTargetDirection();
  bool IsAtSetTargetSpeed();

private:
  SBrickChannel _channel;
  int16_t _setTargetSpeed;
  int16_t _currentTargetSpeed;
};