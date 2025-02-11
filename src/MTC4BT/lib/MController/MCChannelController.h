#pragma once

#include <Arduino.h>

#include "MCChannelConfig.h"

// Channel controller
class MCChannelController
{
  public:
	MCChannelController(MCChannelConfig *config);

	// Returns the current hub led color.
	HubLedColor GetHubLedColor();

	// Sets a new hub led color.
	void SetHubLedColor(HubLedColor color);

	// Returns the device attached to the channel.
	DeviceType GetAttachedDevice();

	// Returns the channel controlled by this controller.
	MCChannel *GetChannel();

	// Set the min pwr percentage. When a pwr percentage is set below this value, we either set current pwr to zero (when decreasing power) or to the given value (when increasing power).
	void SetMinPwrPerc(int16_t minPwrPerc);

	// Gets the current target pwr percentage (sign indicates direction: >0: forward, <0: backwards).
	int16_t GetTargetPwrPerc();

	// Sets a new target pwr percentage (sign indicates direction: >0: forward, <0: backwards).
	void SetTargetPwrPerc(int16_t targetPwrPerc);

	// Returns the current pwr percentage (sign indicates direction: >0: forward, <0: backwards).
	int16_t GetCurrentPwrPerc();

	// Sets the current pwr percentage (directly) to the specified value (sign indicates direction: >0: forward, <0: backwards).
	void SetCurrentPwrPerc(int16_t currentPwrPerc);

	// Updates the current pwr percentage one step towards the target pwr (to be used in a loop).
	bool UpdateCurrentPwrPerc();

	// Returns the absolute current pwr percentage (no direction indication).
	uint8_t GetAbsCurrentPwrPerc();

	// Returns a boolean value indicating whether we're driving forward or not.
	bool IsDrivingForward();

	void Blink();

	// Sets the current manual brake status.
	void ManualBrake(bool enabled);

	// Sets the current e-brake status.
	void EmergencyBrake(bool enabled);

  private:
	// Returns a boolean value indicating whether the channel is currently accelarating in either direction.
	bool isAccelarating();

	// Returns a boolean value indicating whether the channel has reached its target pwr percentage.
	bool isAtTargetPwrPerc();

	// Return the normalized pwr value, bounded by the min and max channel pwr.
	int16_t normalizePwrPerc(int16_t pwrPerc);

	// Reference to the configuration of the channel controlled by this channel controller.
	MCChannelConfig *_config;

	bool _mbrake;
	bool _ebrake;
	ulong _blinkUntil;
	unsigned long _lastUpdate = 0;
	int16_t _minPwrPerc;
	int16_t _targetPwrPerc;
	int16_t _currentPwrPerc;
	HubLedColor _hubLedColor = HubLedColor::WHITE; // on, later in the code it is forced to turn WHITE so make it initial WHITE

	// The following (derived) class can access private members of MCChannelController.
	friend class BLEHubChannelController;
	friend class MCPinController;
};