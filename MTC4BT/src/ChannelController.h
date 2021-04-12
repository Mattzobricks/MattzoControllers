#pragma once

#include <Arduino.h>

enum HubChannel
{
  A,
  B,
  C,
  D
};

enum HubChannelDirection
{
  FORWARD,
  REVERSE
};

enum AttachedDevice
{
  NOTHING,
  MOTOR,
  LIGHT
};

class ChannelController
{
public:
  ChannelController(HubChannel channel, HubChannelDirection direction, AttachedDevice device, int16_t speedStep, int16_t _brakeStep);

  // Returns the controlled channel.
  HubChannel GetChannel();

  // Returns the device attached to the channel.
  AttachedDevice GetAttachedDevice();

  // Sets a new target speed percentage (sign indicates direction: >0: forward, <0: backwards).
  void SetTargetSpeedPerc(int16_t speedPerc);

  // Returns the current speed percentage (sign indicates direction: >0: forward, <0: backwards).
  int16_t GetCurrentSpeedPerc();

  // Returns the absolute current speed percentage (no direction indication).
  uint8_t GetAbsCurrentSpeedPerc();

  // Returns a boolean value indicating whether we're driving forward or not.
  bool IsDrivingForward();

  // Updates the current speed percentage one step towards the target speed.
  bool UpdateCurrentSpeedPerc();

  // Sets the current channel speed percentage to zero immediately, stopping any motion.
  void EmergencyBreak();

private:
  // Returns a boolean value indicating whether the channel is currently accelarating in either direction.
  bool isAccelarating();

  // Returns a boolean value indicating whether the channel has reached its target speed percentage.
  bool isAtTargetSpeedPerc();

  // Return the normalized speed value, bounded by the min and max channel speed.
  int16_t normalizeSpeedPerc(int16_t speedPerc);

  HubChannel _channel;
  HubChannelDirection _direction;
  AttachedDevice _device;
  int16_t _speedStep;
  int16_t _brakeStep;

  int16_t _targetSpeedPerc;
  int16_t _currentSpeedPerc;
};