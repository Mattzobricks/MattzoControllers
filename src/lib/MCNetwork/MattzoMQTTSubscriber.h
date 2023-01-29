#pragma once

#include <Arduino.h>

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

// WiFi library for ESP-32
#include "MCMQTTConfiguration.h"
#include <WiFi.h>

// MQTT task priority.
#define MQTT_TASK_PRIORITY 2

// MQTT handle message task core ID.
#define MQTT_HANDLE_MESSAGE_TASK_COREID 1

// The size of the MQTT task stack specified as the number of bytes.
#define MQTT_TASK_STACK_DEPTH 3072

// Number of message received the MQTT queue can hold before we start dropping them.
// Please note the more messages allowed to be queued, the more (heap) memory we consume.
#define MQTT_INCOMING_QUEUE_LENGTH 100

// Number of message to send the MQTT queue can hold before we start dropping them.
// Please note the more messages allowed to be queued, the more (heap) memory we consume.
#define MQTT_OUTGOING_QUEUE_LENGTH 1000

#define MQTT_UNINITIALIZED -10

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
    /// Reconnect delay in milliseconds. This configures the delay between reconnect attempts.
    /// </summary>
    static int ReconnectDelayInMilliseconds;

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

    /// <summary>
    /// The maximum message size, including header, specified as the number of bytes.
    /// Messages larger than this are ignored!
    /// </summary>
    static uint16_t MaxBufferSize;

    // Methods

    /// <summary>
    /// Setup the MQTT Subscriber.
    /// </summary>
    //char *topic, byte *payload, unsigned int length
    static void Setup(MCMQTTConfiguration *config, void (*mqttCallback)(char *topic, byte *payload, unsigned int length));

    // just the loop, which should be called from the main loop()
    static void loop();
    // Returns the current MQTT connection status.
    static int GetStatus();

  private:
    static MCMQTTConfiguration *_config;
    static char _subscriberName[60];
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
};