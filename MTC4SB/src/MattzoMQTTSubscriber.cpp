// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>
#include "MattzoMQTTSubscriber.h"
#include "MattzoWifiClient.h"
#include "MattzoController_Library.h"
#include "MattzoController_Network_Configuration.h"

WiFiClient wifiSubscriberClient;
PubSubClient mqttSubscriberClient(wifiSubscriberClient);

/// <summary>
/// Setup the MQTT Subscriber.
/// </summary>
/// <param name="topic">Topic to subscribe to.</param>
/// <param name="callback">Callback method to call when a message arrives.</param>
void MattzoMQTTSubscriber::Setup(char *topic, void (*callback)(char *, uint8_t *, unsigned int))
{
#if !defined(ESP32)
#error "Error: this sketch is designed for ESP32 only."
#endif

  if (_setupCompleted)
  {
    Serial.println("Setup already completed!");
    return;
  }

  // Keep the topic, so we can resubscribe to it later, if we need to.
  _topic = topic;

  // Setup and connect to WiFi.
  MattzoWifiClient::Setup();

  // Setup MQTT client.
  mqttSubscriberClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  mqttSubscriberClient.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);
  mqttSubscriberClient.setCallback(callback);

  // Construct subscriber name.
  String _subscriberName = String(mattzoControllerName_char);
  _subscriberName.concat("Subscriber");

  // Start task loop.
  xTaskCreatePinnedToCore(taskLoop, "MQTTSubscriber", StackDepth, NULL, TaskPriority, NULL, CoreID);

  // Setup completed.
  _setupCompleted = true;
}

/// <summary>
/// Sends the given message to the MQTT broker.
/// </summary>
/// <param name="parm">Message to send.</param>
void MattzoMQTTSubscriber::sendMessage(char *topic, const char *message)
{
  Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: [" + topic + "] " + message);
  mqttSubscriberClient.publish(topic, message);
}

/// <summary>
/// Reconnects the MQTT client to the broker (blocking).
/// </summary>
void MattzoMQTTSubscriber::reconnect()
{
  while (!mqttSubscriberClient.connected())
  {
    Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber attempting to connect...");

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

    if (mqttSubscriberClient.connect(_subscriberName.c_str(), "rocrail/service/command", 0, false, lastWillMessage_char))
    {
      Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber connected");
      mqttSubscriberClient.subscribe(_topic);
      Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber subscribed to topic '" + _topic + "'");
    }
    else
    {
      Serial.print("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber connect failed, rc=");
      Serial.print(mqttSubscriberClient.state());
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
void MattzoMQTTSubscriber::taskLoop(void *parm)
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
    if (!mqttSubscriberClient.connected())
    {
      reconnect();
    }

    if (PingDelayInMilliseconds > 0 && millis() - lastPing >= PingDelayInMilliseconds)
    {
      lastPing = millis();

      // Send a ping message.
      sendMessage("roc2bricks/ping", _subscriberName.c_str());
    }

    // Allow the MQTT client to process incoming messages and maintain its connection to the server.
    mqttSubscriberClient.loop();

    // Wait a while before trying again (allowing other tasks to do their work).
    vTaskDelay(HandleMessageDelayInMilliseconds / portTICK_PERIOD_MS);
  }
}

// Initialize static members.
bool MattzoMQTTSubscriber::TriggerBreakOnDisconnect = false;
int MattzoMQTTSubscriber::ReconnectDelayInMilliseconds = 1000;
int MattzoMQTTSubscriber::PingDelayInMilliseconds = 1000;
int MattzoMQTTSubscriber::HandleMessageDelayInMilliseconds = 100;
uint8_t MattzoMQTTSubscriber::TaskPriority = 1;
int8_t MattzoMQTTSubscriber::CoreID = 0;
uint32_t MattzoMQTTSubscriber::StackDepth = 2048;
bool MattzoMQTTSubscriber::_setupCompleted = false;
unsigned long MattzoMQTTSubscriber::lastPing = millis();
String MattzoMQTTSubscriber::_subscriberName = String("Unknown");
char *MattzoMQTTSubscriber::_topic = (char *)"Unknown";