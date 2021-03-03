#pragma once

#include <Arduino.h>

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

// WiFi library for ESP-32
#include <WiFi.h>

extern WiFiClient wifiPublisherClient;
extern PubSubClient mqttPublisherClient;

/// <summary>
/// Class used to publish messages to an MQTT broker.
/// </summary>
class MattzoMQTTPublisher {

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
  static int SendMessageDelayInMilliseconds;

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
  /// Setup the MQTT Publisher.
  /// </summary>
  /// <param name="messageQueueLength">Message queue length. This configures the number of messages the queue can hold, before the publisher starts dropping them.</param>
  static void Setup(const int messageQueueLength = 10);

  /// <summary>
  /// Puts the given message on the queue.
  /// </summary>
  /// <param name="message">Message to add to the queue.</param>
  /// <returns>Boolean value indicating whether the given message was added to the queue. Returns `false` when the queue was full.</returns>
  static bool QueueMessage(const char* message);

private:

  static bool _setupCompleted;
  static char _publisherName[60];

  // Time of the last sent ping.
  static unsigned long lastPing;

  /// <summary>
  /// Sends all messages that are currently queued to the MQTT broker.
  /// </summary>
  static void sendMessages();

  /// <summary>
  /// Sends the given message to the MQTT broker.
  /// </summary>
  /// <param name="parm">Message to send.</param>
  static void sendMessage(const char* topic, const char* message);
  
  /// <summary>
  /// Reconnects the MQTT client to the broker (blocking).
  /// </summary>
  static void reconnect();

  /// <summary>
  /// The main (endless) task loop.
  /// </summary>
  /// <param name="parm"></param>
  static void taskLoop(void* parm);
};