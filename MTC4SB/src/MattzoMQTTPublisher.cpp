// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>
#include "MattzoMQTTPublisher.h"
#include "MattzoWifiClient.h"
#include "MattzoController_Library.h"
#include "MattzoController_Network_Configuration.h"

static QueueHandle_t msg_queue;

/// <summary>
/// Class used to publish messages to an MQTT broker.
/// </summary>
void MattzoMQTTPublisher::Setup(const int messageQueueLength)
{
#if !defined(ESP32)
#error "Error: this sketch is designed for ESP32 only."
#endif

  if (_setupCompleted)
  {
    Serial.println("Setup already completed!");
    return;
  }

  // Setup and connect to WiFi.
  MattzoWifiClient::Setup();

  // Setup MQTT client.
  mqttPublisherClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  mqttPublisherClient.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);

  // Construct publisher name.
  publisherName = String(mattzoControllerName_char);
  publisherName.concat("Publisher");

  // Create a queue with a fixed length that will hold pointers to MQTT messages.
  msg_queue = xQueueCreate(messageQueueLength, sizeof(char *));

  // Start task loop.
  xTaskCreatePinnedToCore(taskLoop, "MQTTPublisher", StackDepth, NULL, TaskPriority, NULL, CoreID);

  // Setup completed.
  _setupCompleted = true;
}

/// <summary>
/// Puts the given message on the queue.
/// </summary>
/// <param name="message">Message to add to the queue.</param>
/// <returns>Boolean value indicating whether the given message was added to the queue. Returns `false` when the queue was full.</returns>
bool MattzoMQTTPublisher::QueueMessage(const char *message)
{
  if (!_setupCompleted)
  {
    Serial.println("Setup not completed. Execute .Setup() first!");
    throw "Setup not completed. Execute .Setup() first!";
  }

  // Allocate memory to hold the message.
  char *messagePtr = (char *)malloc(strlen(message) + 1);

  // Copy the message to the allocated memory block.
  strcpy(messagePtr, message);

  // Store pointer to message in queue (don't block if the queue is full).
  return xQueueSendToBack(msg_queue, (void *)&messagePtr, (TickType_t)0) == pdTRUE;
}

/// <summary>
/// Sends all messages that are currently queued to the MQTT broker.
/// </summary>
void MattzoMQTTPublisher::sendMessages()
{
  char *message;

  // See if there's a message in the queue (do not block).
  while (xQueueReceive(msg_queue, (void *)&message, (TickType_t)0) == pdTRUE)
  {
    // Send queued message to broker.
    sendMessage("rocrail/service/command", message);

    // Erase message from memory by freeing it.
    free(message);
  }
}

/// <summary>
/// Sends the given message to the MQTT broker.
/// </summary>
/// <param name="parm">Message to send.</param>
void MattzoMQTTPublisher::sendMessage(const char *topic, const char *message)
{
  Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: [" + topic + "] " + message);
  mqttPublisherClient.publish(topic, message);
}

/// <summary>
/// Reconnects the MQTT client to the broker (blocking).
/// </summary>
void MattzoMQTTPublisher::reconnect()
{
  while (!mqttPublisherClient.connected())
  {
    Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Publisher attempting to connect...");

    String lastWillMessage;
    if (TriggerBreakOnDisconnect)
    {
      lastWillMessage = "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
    }
    else
    {
      lastWillMessage = "<info msg=\"mc_disconnected\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
    }
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    if (mqttPublisherClient.connect(publisherName.c_str(), "rocrail/service/command", 0, false, lastWillMessage_char))
    {
      Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Publisher connected");
    }
    else
    {
      Serial.print("[" + String(xPortGetCoreID()) + "] MQTT: Publisher connect failed, rc=");
      Serial.print(mqttPublisherClient.state());
      Serial.println(". Try again in a few seconds...");

      // Wait a litte while before retrying.
      vTaskDelay(ReconnectDelayInMilliseconds / portTICK_PERIOD_MS);
    }
  }
}

/// <summary>
/// The main (endless) task loop.
/// </summary>
/// <param name="parm"></param>
void MattzoMQTTPublisher::taskLoop(void *parm)
{
  if (!_setupCompleted)
  {
    throw "Setup not completed. Execute .Setup() first.";
  }

  // Loop forever
  while (1)
  {
    // Wait for connection to Wifi (because we need it to contact the broker).
    MattzoWifiClient::Assert();

    // Wait for connection to MQTT (because we need it to send messages to the broker).
    if (!mqttPublisherClient.connected())
    {
      reconnect();
    }

    if (PingDelayInMilliseconds > 0 && millis() - lastPing >= PingDelayInMilliseconds)
    {
      lastPing = millis();

      // Send a ping message.
      sendMessage("roc2bricks/ping", publisherName.c_str());
    }

    // Send any queued messages.
    sendMessages();

    // Allow the MQTT client to process incoming messages (not relevant for a publisher) and maintain its connection to the server.
    mqttPublisherClient.loop();

    // Wait a while before trying again (allowing other tasks to do their work).
    vTaskDelay(SendMessageDelayInMilliseconds / portTICK_PERIOD_MS);
  }
}