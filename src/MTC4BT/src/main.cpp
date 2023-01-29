#include <Arduino.h>

#include "MTC4BTController.h"
#include "MTC4BTMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"
#include "MattzoWifiClient.h"
#include "loadControllerConfiguration.h"
#include "loadNetworkConfiguration.h"
#include "log4MC.h"

#define NETWORK_CONFIG_FILE "/network_config.json"
#define CONTROLLER_CONFIG_FILE "/controller_config.json"

// Globals
MCNetworkConfiguration *networkConfig;
MTC4BTController *controller;
MTC4BTConfiguration *controllerConfig;

#ifdef ESP32
// 1 minute, 30, 15 or 10 seconds
#ifdef TICKER
#if TICKER == 1 or TICKER == 2 or TICKER == 4 or TICKER == 6
#define SETUPTICKER
void handleTickerLoop(void *param)
{
    long minuteTicker = 0;
    unsigned long timeTaken = 0;
    for (;;) {
        timeTaken = millis();
        log4MC::vlogf(LOG_INFO, "Minutes uptime: %d.%02d", (minuteTicker / TICKER), (minuteTicker % TICKER) * (60 / TICKER));
        log4MC::vlogf(LOG_INFO, "  Memory Heap free: %8u max alloc: %8u min free: %8u", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
        minuteTicker++;
        timeTaken = abs((long)(timeTaken - millis()));
        delay(60000 / TICKER - timeTaken);
    }
}
void setupTicker()
{
    xTaskCreatePinnedToCore(handleTickerLoop, "TickerHandler", 2048, NULL, 2, NULL, 1);
    delay(500);
}
#else
#error "Invalid ticker value, valid values are 1,2,4 or 6"
#endif
#endif
#endif

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    // Allocate memory to hold the message.
    char *message = (char *)malloc(length + 1);
    for (int i = 0; i < length; i++) {
        message[i] = (char)payload[i];
    }
    message[length] = '\0';
    MTC4BTMQTTHandler::Handle(message, controller);
    free(message);
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
    MattzoMQTTSubscriber::Setup(networkConfig->MQTT, mqttCallback);

    log4MC::info("Setup: MattzoTrainController for BLE running.");
    log4MC::vlogf(LOG_INFO, "Setup: Number of locos to discover hubs for: %u", controllerConfig->Locomotives.size());
#ifdef ESP32
#ifdef SETUPTICKER
    setupTicker();
    log4MC::info("Ticker started");
#endif
#endif
}

void loop()
{
    MattzoMQTTSubscriber::loop();
    controller->Loop();
}