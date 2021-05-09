#pragma once

#include <Arduino.h>
#include "ChannelConfiguration.h"

class ChannelController
{
public:
  ChannelController(ChannelConfiguration *config);

  // Returns the controlled channel.
  HubChannel GetChannel();

  // Returns the device attached to the channel.
  AttachedDevice GetAttachedDevice();

  // Set the min speed percentage. When a speed percentage is set below this value, we either set current speed to zero (when slowing down) or to the given value (when speeding up).
  void SetMinSpeedPerc(int16_t minSpeedPerc);

  // Sets a new target speed percentage (sign indicates direction: >0: forward, <0: backwards).
  void SetTargetSpeedPerc(int16_t speedPerc);

  // Returns the current speed percentage (sign indicates direction: >0: forward, <0: backwards).
  int16_t GetCurrentSpeedPerc();

  // Sets the current speed to the specified value (sign indicates direction: >0: forward, <0: backwards).
  void SetCurrentSpeedPerc(int16_t speedPerc);

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

  ChannelConfiguration *_config;

  int16_t _minSpeedPerc;
  int16_t _targetSpeedPerc;
  int16_t _currentSpeedPerc;
};