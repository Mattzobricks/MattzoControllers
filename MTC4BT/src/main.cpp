#include <Arduino.h>

#include "MattzoWifiClient.h"
#include "MattzoBLEMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"
#include "log4MC.h"
#include "loadNetworkConfiguration.h"
#include "loadControllerConfiguration.h"
#include "MTC4BTController.h"

#define NETWORK_CONFIG_FILE "/network_config.json"
#define CONTROLLER_CONFIG_FILE "/controller_config.json"

// Globals
static QueueHandle_t msg_queue;
MCNetworkConfiguration *networkConfig;
MTC4BTController *controller;
MTC4BTConfiguration *controllerConfig;

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    // Read the message.
    char msg[length + 1];
    for (int i = 0; i < length; i++)
    {
        msg[i] = (char)payload[i];
    }
    msg[length] = '\0';

    // Allocate memory to hold the message.
    char *messagePtr = (char *)malloc(strlen(msg) + 1);

    // Copy the message to the allocated memory block.
    strcpy(messagePtr, msg);

    // Store pointer to message in queue (don't block if the queue is full).
    if (xQueueSendToBack(msg_queue, (void *)&messagePtr, (TickType_t)0) == pdTRUE)
    {
        // Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Queued incoming MQTT message [" + String(topic) + "]: " + String(msg));
    }
    else
    {
        // Free the allocated memory block as we couldn't queue the message anyway.
        free(messagePtr);
        log4MC::warn("Loop: Incoming MQTT message queue full");
    }
}

void handleMQTTMessages(void *parm)
{
    for (;;)
    {
        char *message;

        // See if there's a message in the queue (do not block).
        while (xQueueReceive(msg_queue, (void *)&message, (TickType_t)0) == pdTRUE)
        {
            // Output message to serial for debug.
            // Serial.print("[" + String(xPortGetCoreID()) + "] Ctrl: Received MQTT message; " + message);

            // Parse message and translate to a BLE command for loco hub(s).
            MattzoBLEMQTTHandler::Handle(message, controller);

            // Erase message from memory by freeing it.
            free(message);
        }

        // Wait a while before trying again (allowing other tasks to do their work)?
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    // Configure Serial.
    Serial.begin(115200);

    // Wait a moment to start (so we don't miss Serial output).
    delay(1000 / portTICK_PERIOD_MS);

    Serial.println();
    Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Starting MattzoTrainController for BLE...");

    // Load the network configuration.
    Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Loading network configuration...");
    networkConfig = loadNetworkConfiguration(NETWORK_CONFIG_FILE);

    // Setup logging (from now on we can use log4MC).
    log4MC::Setup(networkConfig->WiFi->hostname, networkConfig->Logging);

    // Load the controller configuration.
    log4MC::info("Setup: Loading controller configuration...");
    controllerConfig = loadControllerConfiguration(CONTROLLER_CONFIG_FILE);
    controller = new MTC4BTController();
    controller->Setup(controllerConfig);
    log4MC::info("Setup: Controller configuration completed.");

    // Setup and connect to WiFi.
    MattzoWifiClient::Setup(networkConfig->WiFi);

    // Setup a queue with a fixed length that will hold pointers to incoming MQTT messages.
    msg_queue = xQueueCreate(MQTT_INCOMING_QUEUE_LENGTH, sizeof(char *));

    // Start MQTT task loop to handle queued messages.
    xTaskCreatePinnedToCore(handleMQTTMessages, "MQTTHandler", MQTT_TASK_STACK_DEPTH, NULL, MQTT_TASK_PRIORITY, NULL, MQTT_HANDLE_MESSAGE_TASK_COREID);

    // Setup MQTT publisher (with a queue that can hold 1000 messages).
    // MattzoMQTTPublisher::Setup(ROCRAIL_COMMAND_QUEUE, MQTT_OUTGOING_QUEUE_LENGTH);

    // Setup MQTT subscriber (use controller name as part of the subscriber name).
    networkConfig->MQTT->SubscriberName = controllerConfig->ControllerName;
    MattzoMQTTSubscriber::Setup(networkConfig->MQTT, mqttCallback);

    log4MC::info("Setup: MattzoTrainController for BLE running.");
    log4MC::vlogf(LOG_INFO, "Setup: Number of locos to discover hubs for: %u", controllerConfig->Locomotives.size());
}

void loop()
{
    controller->Loop();
}