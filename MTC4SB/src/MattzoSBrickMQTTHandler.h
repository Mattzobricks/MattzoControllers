#pragma once

#include <Arduino.h>

#include "SBrickHubClient.h"

/// <summary>
/// Class used to translate MQTT messages to BLE commands.
/// </summary>
class MattzoSBrickMQTTHandler
{

public:
    static const int16_t LIGHT_ON_SPEED = 100;
    static const int16_t LIGHT_OFF_SPEED = 0;

    // Methods

    /// <summary>
    /// Handle the given MQTT message and apply it to the applicable SBrick(s).
    /// </summary>
    /// <param name="message">MQTT message.</param>
    /// <param name="sbricks">SBricks.</param>
    static void Handle(const char *message, ulong numSBricks, SBrickHubClient *sbricks[]);

private:
    // Methods

    static bool isNodeType(const String message, const char *nodeName);
    static void handleSys(const String message, ulong numSBricks, SBrickHubClient *sbricks[]);
    static void handleLc(const String message, ulong numSBricks, SBrickHubClient *sbricks[]);
    static void handleFn(const String message, ulong numSBricks, SBrickHubClient *sbricks[]);
    static String getAttr(const String message, const String attrName);
};