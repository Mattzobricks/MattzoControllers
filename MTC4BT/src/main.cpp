#include <Arduino.h>

#include "MTC4BT_loco_config.h"
#include "MC_mqtt_config.h"
#include "MattzoController_Library.h"
#include "MattzoBLEMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"

#define MATTZO_CONTROLLER_TYPE "MTC4BT"

#define LIGHTS_ON true
#define LIGHTS_OFF false

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
static QueueHandle_t msg_queue;
NimBLEScan *scanner;

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
        Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Incoming MQTT message queue full");
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

            // Parse message and translate to a BLE command for SBrick(s).
            MattzoBLEMQTTHandler::Handle(message, numHubs, hubs);

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
    Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Starting MattzoTrainController for BLE...");

    // Setup Mattzo controller.
    setupMattzoController(MATTZO_CONTROLLER_TYPE);

    // Setup a queue with a fixed length that will hold pointers to incoming MQTT messages.
    msg_queue = xQueueCreate(MQTT_INCOMING_QUEUE_LENGTH, sizeof(char *));

    // Start task loop to handle queued MQTT messages.
    xTaskCreatePinnedToCore(handleMQTTMessages, "MQTTHandler", MQTT_TASK_STACK_DEPTH, NULL, MQTT_TASK_PRIORITY, NULL, MQTT_HANDLE_MESSAGE_TASK_COREID);

    // Setup MQTT publisher (with a queue that can hold 1000 messages).
    // MattzoMQTTPublisher::Setup(ROCRAIL_COMMAND_QUEUE, MQTT_OUTGOING_QUEUE_LENGTH);

    // Setup MQTT subscriber.
    MattzoMQTTSubscriber::Setup(ROCRAIL_COMMAND_TOPIC, mqttCallback);

    // Load the hub configuration.
    configureHubs();

    // Initialize BLE client.
    Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Initializing BLE...");
    NimBLEDevice::init("");

    // Configure BLE scanner.
    scanner = NimBLEDevice::getScan();
    scanner->setInterval(45);
    scanner->setWindow(15);
    scanner->setActiveScan(true);

    Serial.print("[" + String(xPortGetCoreID()) + "] Setup: Number of Hubs to discover: ");
    Serial.println(numHubs);
}

void loop()
{
    for (int i = 0; i < numHubs; i++)
    {
        BLEHub *hub = hubs[i];

        if (!hub->IsEnabled())
        {
            // Skip to the next Hub.
            continue;
        }

        if (!hub->IsConnected())
        {
            if (!hub->IsDiscovered())
            {
                // Hub not discovered yet, first discover it.
                hub->StartDiscovery(scanner, BLE_SCAN_DURATION_IN_SECONDS);
            }

            if (hub->IsDiscovered())
            {
                // Hub discovered, try to connect now.
                if (!hub->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS))
                {
                    // Connect attempt failed. Will retry in next loop.
                    Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Connect failed");
                }
                else
                {
                    // Blink lights three times when connected.
                    hub->SetLights(LIGHTS_ON);
                    delay(250 / portTICK_PERIOD_MS);
                    hub->SetLights(LIGHTS_OFF);
                    delay(250 / portTICK_PERIOD_MS);
                    hub->SetLights(LIGHTS_ON);
                    delay(250 / portTICK_PERIOD_MS);
                    hub->SetLights(LIGHTS_OFF);
                    delay(250 / portTICK_PERIOD_MS);
                    hub->SetLights(LIGHTS_ON);
                    delay(250 / portTICK_PERIOD_MS);
                    hub->SetLights(LIGHTS_OFF);
                }
            }
        }
    }

    // Delay next scan/connect attempt for a while, allowing the background drive tasks of already connected SBricks to send their periodic commands.
    delay(BLE_CONNECT_DELAY_IN_SECONDS * 1000 / portTICK_PERIOD_MS);
}