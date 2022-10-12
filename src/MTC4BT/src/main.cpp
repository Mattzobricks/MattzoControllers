#include <Arduino.h>

#include "MattzoWifiClient.h"
#include "MTC4BTMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"
#include "log4MC.h"
#include "loadNetworkConfiguration.h"
#include "loadControllerConfiguration.h"
#include "MTC4BTController.h"

#define NETWORK_CONFIG_FILE "/network_config.json"
#define CONTROLLER_CONFIG_FILE "/controller_config.json"

// Globals
MCNetworkConfiguration *networkConfig;
MTC4BTController *controller;
MTC4BTConfiguration *controllerConfig;

#ifdef ESP32
#ifdef TICKER
void handleTickerLoop(void *param)
{
    static long minuteTicker = 0;
    for (;;)
    {
        log4MC::vlogf(LOG_INFO, "Minutes uptime: %ld", minuteTicker);
        minuteTicker++;
        // vTaskDelay(6000 / portTICK_PERIOD_MS);
        delay(60000);
    }
}
void setupTicker()
{
    xTaskCreatePinnedToCore(handleTickerLoop, "TickerHandler", 10000, NULL, 2, NULL, 1);
    delay(500);
}
#endif
#endif
void handleMQTTMessageLoop(void *parm)
{
    for (;;)
    {
        char *message;

        // See if there's a message in the queue (do not block).
        while (xQueueReceive(MattzoMQTTSubscriber::IncomingQueue, (void *)&message, (TickType_t)0) == pdTRUE)
        {
            // Output message to serial for debug.
            // Serial.print("[" + String(xPortGetCoreID()) + "] Ctrl: Received MQTT message; " + message);

            // Parse message and translate to an action for devices attached to this controller.
            MTC4BTMQTTHandler::Handle(message, controller);

            // Erase message from memory by freeing it.
            free(message);
        }

        // Wait a while before trying again (allowing other tasks to do their work)?
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    log4MC::Setup(networkConfig->WiFi->hostname.c_str(), networkConfig->Logging);

    // Load the controller configuration.
    log4MC::info("Setup: Loading controller configuration...");
    controllerConfig = loadControllerConfiguration(CONTROLLER_CONFIG_FILE);
    controller = new MTC4BTController();
    controller->Setup(controllerConfig);
    log4MC::info("Setup: Controller configuration completed.");

    // Setup and connect to WiFi.
    MattzoWifiClient::Setup(networkConfig->WiFi);

    // Setup MQTT publisher (with a queue that can hold 1000 messages).
    // MattzoMQTTPublisher::Setup(ROCRAIL_COMMAND_QUEUE, MQTT_OUTGOING_QUEUE_LENGTH);

    // Setup MQTT subscriber (use controller name as part of the subscriber name).
    networkConfig->MQTT->SubscriberName = controllerConfig->ControllerName;
    MattzoMQTTSubscriber::Setup(networkConfig->MQTT, handleMQTTMessageLoop);

    log4MC::info("Setup: MattzoTrainController for BLE running.");
    log4MC::vlogf(LOG_INFO, "Setup: Number of locos to discover hubs for: %u", controllerConfig->Locomotives.size());
#ifdef ESP32
#ifdef TICKER
    setupTicker();
    log4MC::info("Ticker started");
#endif
#endif
}

void loop()
{
    controller->Loop();
}