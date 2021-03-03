#pragma once

#include <Arduino.h>

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

// WiFi library for ESP-32
#include <Wifi.h>

extern WiFiClient wifiSubscriberClient;
extern PubSubClient mqttSubscriberClient;

/// <summary>
/// Class used to publish messages to an MQTT broker.
/// </summary>
class MattzoMQTTSubscriber
{

public:
  // Public static members

  /// <summary>
  /// Boolean value indicating whether we should emergency break when we get disconnected.
  /// </summary>
  static bool TriggerBreakOnDisconnect;

  /// <summary>
  /// Reconnect delay in milliseconds. This configures the delay between reconnect attempts.
  /// </summary>
  static int ReconnectDelayInMilliseconds;

  /// <summary>
  /// Ping delay in milliseconds. This configures the delay between ping messages (0 = don't send ping messages).
  /// </summary>
  static int PingDelayInMilliseconds;

  /// <summary>
  /// Send message delay in milliseconds. This configures the delay between send message attempts.
  /// </summary>
  static int HandleMessageDelayInMilliseconds;

  /// <summary>
  /// The priority at which the task should run.
  /// Systems that include MPU support can optionally create tasks in a privileged (system) mode by setting bit portPRIVILEGE_BIT of the priority parameter.
  /// For example, to create a privileged task at priority 2 the uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
  /// </summary>
  static uint8_t TaskPriority;

  /// <summary>
  /// If the value is tskNO_AFFINITY, the created task is not pinned to any CPU, and the scheduler can run it on any core available.
  /// Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
  /// Specifying values larger than (portNUM_PROCESSORS - 1) will cause the function to fail.
  /// </summary>
  static int8_t CoreID;

  /// <summary>
  /// The size of the task stack specified as the number of bytes.
  /// </summary>
  static uint32_t StackDepth;

  // Methods

  /// <summary>
  /// Setup the MQTT Subscriber.
  /// </summary>
  /// <param name="topic">Topic to subscribe to.</param>
  /// <param name="callback">Callback method to call when a message arrives.</param>
  static void Setup(char *topic, void (*callback)(char *, uint8_t *, unsigned int));

private:
  static String _subscriberName;
  static char *_topic;
  static bool _setupCompleted;

  // Time of the last sent ping.
  static unsigned long lastPing;

  /// <summary>
  /// Sends the given message to the MQTT broker.
  /// </summary>
  /// <param name="parm">Message to send.</param>
  static void sendMessage(char *topic, const char *message);

  /// <summary>
  /// Reconnects the MQTT client to the broker (blocking).
  /// </summary>
  static void reconnect();

  /// <summary>
  /// The main (endless) task loop.
  /// </summary>
  /// <param name="parm"></param>
  static void taskLoop(void *parm);
};