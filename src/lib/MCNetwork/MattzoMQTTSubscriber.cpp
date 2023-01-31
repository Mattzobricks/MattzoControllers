#include "MattzoMQTTSubscriber.h"
#include "MattzoWifiClient.h"
#include "log4MC.h"
#include <PubSubClient.h>

WiFiClient wifiSubscriberClient;
PubSubClient mqttSubscriberClient(wifiSubscriberClient);

void MattzoMQTTSubscriber::Setup(MCMQTTConfiguration *config, void (*mqttcallback)(char *topic, byte *payload, unsigned int length))
{
#if !defined(ESP32)
#error "Error: this sketch is designed for ESP32 only."
#endif

    _config = config;

    if (_setupCompleted) {
        log4MC::warn("MQTT: Subscriber setup already completed!");
        return;
    }

    // Setup MQTT client.
    log4MC::vlogf(LOG_INFO, "MQTT: Connecting to %s:%u...", _config->ServerAddress.c_str(), _config->ServerPort);
    mqttSubscriberClient.setServer(_config->ServerAddress.c_str(), _config->ServerPort);
    mqttSubscriberClient.setKeepAlive(_config->KeepAlive);
    mqttSubscriberClient.setBufferSize(MaxBufferSize);
    mqttSubscriberClient.setCallback(mqttcallback);

    // Construct subscriber name.
    strcpy(_subscriberName, _config->SubscriberName);
    strcat(_subscriberName, "Subscriber");

    // Setup completed.
    _setupCompleted = true;
}

int MattzoMQTTSubscriber::GetStatus()
{
    return _setupCompleted ? mqttSubscriberClient.state() : MQTT_UNINITIALIZED;
}

/// <summary>
/// Sends the given message to the MQTT broker.
/// </summary>
/// <param name="parm">Message to send.</param>
void MattzoMQTTSubscriber::sendMessage(char *topic, const char *message)
{
    log4MC::vlogf(LOG_DEBUG, "MQTT: Sending message [%s] %s", topic, message);
    mqttSubscriberClient.publish(topic, message);
}

/// <summary>
/// Reconnects the MQTT client to the broker (blocking).
/// </summary>
void MattzoMQTTSubscriber::reconnect()
{
    while (!mqttSubscriberClient.connected()) {
        log4MC::info("MQTT: Subscriber configuring last will...");

        String lastWillMessage;
        // if (_config->EbrakeOnDisconnect)
        {
            lastWillMessage = "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"" + String(_config->SubscriberName) + "\"/>";
        }
        // else
        // {
        //   lastWillMessage = "<info msg=\"mc_disconnected\" source=\"lastwill\" mc=\"" + String(_config->SubscriberName) + "\"/>";
        // }
        char lastWillMessage_char[lastWillMessage.length() + 1];
        lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

        log4MC::info("MQTT: Subscriber attempting to connect...");

        if (mqttSubscriberClient.connect(_subscriberName, _config->Topic, 0, false, lastWillMessage_char)) {
            log4MC::info("MQTT: Subscriber connected");
            mqttSubscriberClient.subscribe(_config->Topic);
            log4MC::vlogf(LOG_INFO, "MQTT: Subscriber subscribed to topic '%s'", _config->Topic);
        } else {
            log4MC::vlogf(LOG_WARNING, "MQTT: Subscriber connect failed, rc=%u. Try again in a few seconds...", mqttSubscriberClient.state());

            // Wait a litte while before retrying.
            delay(ReconnectDelayInMilliseconds);
        }
    }
}

void MattzoMQTTSubscriber::loop()
{
    // Wait for connection to Wifi (because we need it to contact the broker).
    MattzoWifiClient::Assert();

    // Wait for connection to MQTT (because we need it to send messages to the broker).
    if (!mqttSubscriberClient.connected()) {
        reconnect();
    }

    if (_config->Ping > 0 && millis() - lastPing >= _config->Ping * 1000) {
        lastPing = millis();

        // Send a ping message.
        sendMessage("roc2bricks/ping", _subscriberName);
    }

    // Allow the MQTT client to process incoming messages and maintain its connection to the server.
    mqttSubscriberClient.loop();
}

// Initialize static members.
int MattzoMQTTSubscriber::ReconnectDelayInMilliseconds = 1000;
uint8_t MattzoMQTTSubscriber::TaskPriority = 2;
int8_t MattzoMQTTSubscriber::CoreID = 0;
uint32_t MattzoMQTTSubscriber::StackDepth = 2048;
uint16_t MattzoMQTTSubscriber::MaxBufferSize = 1024;

bool MattzoMQTTSubscriber::_setupCompleted = false;
unsigned long MattzoMQTTSubscriber::lastPing = millis();
char MattzoMQTTSubscriber::_subscriberName[60] = "Unknown";
MCMQTTConfiguration *MattzoMQTTSubscriber::_config = nullptr;