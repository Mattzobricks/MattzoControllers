#pragma once

#include "MCWiFiConfiguration.h"

#define WL_UNINITIALIZED -2
#define WL_INITIALIZING -1

// Class used to connect to WiFi.
class MattzoWifiClient
{
  public:
	// Setup the WiFi client.
	static void Setup(MCWiFiConfiguration *config);
	static void Loop();

	// return either wifi or ethernet status
	static int status();
	// Returns the current WiFi connection status.
	static int GetStatus();

	// Blocking call waiting for a WiFi connection. It also handles OTA updates.
	static void Assert();
	static bool useWifiStatus;

  private:
	static void startOTA();

	// Private static members.
	static MCWiFiConfiguration *_config;
	static bool _setupInitiated;
	static bool _setupCompleted;
	static bool _wasConnected;
};