#pragma once

#include <Arduino.h>

#include "SBrickHubClient.h"

/// <summary>
/// Class used to translate MQTT messages to BLE commands.
/// </summary>
class MattzoSBrickMQTTHandler
{

public:
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
    static void handleLc(const String message, ulong numSBricks, SBrickHubClient *sbricks[]);
    static String getAttr(const String message, const String attrName);
};