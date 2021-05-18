#include <Arduino.h>

#include "MattzoWifiClient.h"
#include "MattzoBLEMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"
#include "BLEHubScanner.h"
#include "log4MC.h"
#include "loadNetworkConfiguration.h"
#include "loadControllerConfiguration.h"
#include "MTC4BTController.h"
#include "MCStatusLED.h"

#define NETWORK_CONFIG_FILE "/network_config.json"
#define CONTROLLER_CONFIG_FILE "/controller_config.json"

#define LIGHTS_ON true
#define LIGHTS_OFF false
#define LIGHTS_BLINK_DELAY_ON_CONNECT_MS 250

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 5;

// Duration between BLE discovery and connect attempts in seconds.
const uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 3;

// Sets the watchdog timeout (0D &lt; timeout in 0.1 secs, 1 byte &gt;)
// The purpose of the watchdog is to stop driving in case of an application failure.
// Watchdog starts when the first DRIVE command is issued during a connection.
// Watchdog is stopped when all channels are either set to zero drive, or are braking.
// The value is saved to the persistent store.
// The recommended watchdog frequency is 0.2-0.5 seconds, but a smaller and many larger settings are also available.
// Writing a zero disables the watchdog.
// By default watchdog is set to 5, which means a 0.5 second timeout.
const int8_t WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS = 5;

// Globals
MCStatusLED *statusLed = nullptr;
static QueueHandle_t msg_queue;
NimBLEScan *scanner;
BLEHubScanner *hubScanner;
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

void ledLoop(void *parm)
{
    for (;;)
    {
        if (statusLed)
        {
            statusLed->UpdateByStatus();
        }
        
        for (MCLed *led : controller->Leds)
        {
            led->Update();
        }
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
    controller = new MTC4BTController(controllerConfig);

    // Setup status LED, if configured.
    DeviceConfiguration *statusLedConfig = controller->GetStatusLed();
    if (statusLedConfig)
    {
        log4MC::vlogf(LOG_INFO, "Setup: Found status led attached to ESP pin %s. Initializing...", statusLedConfig->GetAddress().c_str());

        // Initiate status LED.
        statusLed = new MCStatusLED(statusLedConfig->GetAddressAsEspPinNumber(), statusLedConfig->IsInverted());
    }

    // Start led task loop.
    xTaskCreatePinnedToCore(ledLoop, "LedLoop", 1024, NULL, 1, NULL, 1);

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

    // Initialize BLE client.
    log4MC::info("Setup: Initializing BLE...");
    NimBLEDevice::init("");

    // Configure BLE scanner.
    scanner = NimBLEDevice::getScan();
    scanner->setInterval(45);
    scanner->setWindow(15);
    scanner->setActiveScan(true);
    hubScanner = new BLEHubScanner();

    log4MC::info("Setup: MattzoTrainController for BLE running.");
    log4MC::vlogf(LOG_INFO, "Setup: Number of locos to discover Hubs for: %u", controllerConfig->Locomotives.size());
}

void loop()
{
    std::vector<BLEHub *> undiscoveredHubs;

    for (int l = 0; l < controllerConfig->Locomotives.size(); l++)
    {
        BLELocomotive *loco = controllerConfig->Locomotives.at(l);

        if (!loco->IsEnabled() || loco->AllHubsConnected())
        {
            // Loco is not in use or all hubs are already connected. Skip to the next loco.
            continue;
        }

        uint hubCount = loco->GetHubCount();

        for (int h = 0; h < hubCount; h++)
        {
            BLEHub *hub = loco->GetHub(h);

            if (!hub->IsEnabled())
            {
                // Skip to the next Hub.
                continue;
            }

            if (!hub->IsConnected())
            {
                if (!hub->IsDiscovered())
                {
                    // Hub not discovered yet, add to list of hubs to discover.
                    undiscoveredHubs.push_back(hub);
                }

                if (hub->IsDiscovered())
                {
                    // Hub discovered, try to connect now.
                    if (!hub->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS))
                    {
                        // Connect attempt failed. Will retry in next loop.
                        log4MC::warn("Loop: Connect failed. Will retry...");
                    }
                    else
                    {
                        log4MC::vlogf(LOG_INFO, "Loop: Connected to all hubs of loco '%s'.", loco->GetLocoName().c_str());

                        if (loco->AllHubsConnected())
                        {
                            // TODO: Make loco blink its lights, instead of the individual hub below.
                        }

                        // Blink lights three times when connected.
                        hub->SetLights(LIGHTS_ON);
                        delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                        hub->SetLights(LIGHTS_OFF);
                        delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                        hub->SetLights(LIGHTS_ON);
                        delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                        hub->SetLights(LIGHTS_OFF);
                        delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                        hub->SetLights(LIGHTS_ON);
                        // delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                        // hub->SetLights(LIGHTS_OFF);
                    }
                }
            }
        }
    }

    if (undiscoveredHubs.size() > 0)
    {
        // Start discovery for undiscovered hubs.
        hubScanner->StartDiscovery(scanner, undiscoveredHubs, BLE_SCAN_DURATION_IN_SECONDS);

        // Delay next discovery/connect attempts for a while, allowing the background tasks of already connected Hubs to send their periodic drive commands.
        delay(BLE_CONNECT_DELAY_IN_SECONDS * 1000 / portTICK_PERIOD_MS);
    }
}