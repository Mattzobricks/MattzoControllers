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

    // Returns the current WiFi connection status.
    static int GetStatus();

    // Blocking call waiting for a WiFi connection. It also handles OTA updates.
    static void Assert();

  private:
    static void startOTA();

    // Private static members.
    static MCWiFiConfiguration *_config;
    static bool _setupInitiated;
    static bool _setupCompleted;
    static bool _wasConnected;
};