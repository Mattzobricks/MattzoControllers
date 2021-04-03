#pragma once

/// <summary>
/// Class used to connect to WiFi.
/// </summary>
class MattzoWifiClient {

public:
  /// <summary>
  /// Setup the WiFi client.
  /// </summary>
  static void Setup();

  /// <summary>
  /// Blocking call waiting for a WiFi connection. It also handles OTA updates.
  /// </summary>
  static void Assert();

private:
  
  // Private static members.
  static bool setupInitiated;
  static bool setupCompleted;
  static bool wasConnected;
};