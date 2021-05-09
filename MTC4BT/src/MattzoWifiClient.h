#pragma once

#include "MCWiFiConfiguration.h"

// Class used to connect to WiFi.
class MattzoWifiClient
{
public:
  // Setup the WiFi client.
  static void Setup(MCWiFiConfiguration *config);

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