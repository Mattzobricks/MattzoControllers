#pragma once
#include <Arduino.h>

#include "MCChannelController.h"
#include "MCConfiguration.h"
#include "MCLedBase.h"
#include "MCLocoAction.h"
#include "MattzoMQTTSubscriber.h"
#include "MattzoWifiClient.h"

enum MCConnectionStatus {
	uninitialized = 0,
	initializing,
	connecting_wifi,
	connecting_mqtt,
	connected
};

class MController
{
  public:
	MController();

	// Returns the current controller connection status.
	static MCConnectionStatus GetConnectionStatus();

	// Mattzo Controller setup initialization method.
	void Setup(MCConfiguration *config);

	// Updates emergency brake based on the current controller connection status and controls leds.
	void Loop();

	// Loop is not running in setup but the led needs to be set to indicate searching for network (or something)
	void setStatusLedInSetup(int powerPerc);

	// Returns a boolean value indicating whether the e-brake flag is currently set or not.
	bool GetEmergencyBrake();

	// Sets the emergency brake flag to the given value.
	void SetEmergencyBrake(const bool enabled);

	// Executes the given action locally on this controller.
	void Execute(MCLocoAction *action);

	// Abstract method required for derived controller implementations to handle e-brake.
	virtual void HandleSys(const bool ebrake) = 0;

	// Abstract method required to handle the given trigger (if loco is under control of this controller).
	virtual void HandleTrigger(int locoAddress, MCTriggerSource source, std::string eventType, std::string eventId, std::string value) = 0;

  private:
	// Initializes the pin channels.
	void initChannelControllers();

	// Returns the controller for the requested channel.
	MCChannelController *findControllerByChannel(MCChannel *channel);

	// Returns the led instance for the requested pin.
	MCLedBase *findLedByPinNumber(int pin);

	// Initializes an led instance, if it doesn't exist yet.
	void initLed(int pwmChannel, int pin, bool inverted);

	// Initializes a status led instance, if it doesn't exist yet.
	void initStatusLed(int pwmChannel, int pin);

	// List of references to leds attached to this controller.
	std::vector<MCLedBase *> _espLeds;

	// List of references to led controllers.
	std::vector<MCChannelController *> _channelControllers;

	// Boolean value indicating whether emergency brake is currently enabled or not.
	bool _ebrake;

	// Reference to the configuration of this controller.
	MCConfiguration *_config;
};