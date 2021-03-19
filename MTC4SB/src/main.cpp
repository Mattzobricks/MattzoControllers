#include <Arduino.h>
#include <NimBLEDevice.h>

// MattzoControllerType
#define MATTZO_CONTROLLER_TYPE "MTC4SB"

// MattzoController network configuration (the following file needs to be moved into the Arduino library folder):
//#include <MattzoController_Network_Configuration.h>

#include "MattzoController_Library.h"
#include "MattzoWifiClient.h"
#include "MattzoMQTTPublisher.h"
#include "MattzoMQTTSubscriber.h"
#include "MattzoSBrickMQTTHandler.h"
#include "SBrickConst.h"
#include "SBrickHubClient.h"

#define SBRICK_ENABLED true
#define SBRICK_DISABLED false

#define AUTO_LIGHTS_ENABLED true
#define AUTO_LIGHTS_DISABLED false

// Globals
static QueueHandle_t msg_queue;
NimBLEScan *scanner;
SBrickHubClient *mySBricks[] = {
    new SBrickHubClient("YC7939", "00:07:80:d0:47:43", AUTO_LIGHTS_ENABLED),
    new SBrickHubClient("HE10233", "00:07:80:d0:3a:f2", AUTO_LIGHTS_ENABLED, 10, 20, SBRICK_DISABLED),
    new SBrickHubClient("BC60052", "88:6b:0f:23:78:10", AUTO_LIGHTS_ENABLED, 10, 20, SBRICK_DISABLED)};

#define numSBricks (sizeof(mySBricks) / sizeof(SBrickHubClient *))

#define ROCRAIL_COMMAND_TOPIC "rocrail/service/command"

/// <summary>
/// Boolean value indicating whether the MQTT Publisher and Subscriber are enabled.
/// </summary>
bool ENABLE_MQTT = true;

/// <summary>
/// MQTT task priority.
/// </summary>
const uint8_t MQTT_TASK_PRIORITY = 1;

/// <summary>
/// MQTT handle message task core ID.
/// </summary>
const uint8_t MQTT_HANDLE_MESSAGE_TASK_COREID = 1;

/// <summary>
/// The size of the MQTT task stack specified as the number of bytes.
/// </summary>
const uint32_t MQTT_TASK_STACK_DEPTH = 2048;

/// <summary>
/// Number of message received the MQTT queue can hold before we start dropping them.
/// Please note the more messages allowed to be queued, the more (heap) memory we consume.
/// </summary>
const int MQTT_INCOMING_QUEUE_LENGTH = 100;

/// <summary>
/// Number of message to send the MQTT queue can hold before we start dropping them.
/// Please note the more messages allowed to be queued, the more (heap) memory we consume.
/// </summary>
const int MQTT_OUTGOING_QUEUE_LENGTH = 1000;

/// <summary>
/// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
/// </summary>
uint32_t BLE_SCAN_DURATION_IN_SECONDS = 1;

/// <summary>
/// Duration between BLE discovery and connect attempts in seconds.
/// </summary>
uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 2;

/// <summary>
/// Sets the watchdog timeout (0D &lt; timeout in 0.1 secs, 1 byte &gt;)
/// The purpose of the watchdog is to stop driving in case of an application failure.
/// Watchdog starts when the first DRIVE command is issued during a connection.
/// Watchdog is stopped when all channels are either set to zero drive, or are braking.
/// The value is saved to the persistent store.
/// The recommended watchdog frequency is 0.2-0.5 seconds, but a smaller and many larger settings are also available.
/// Writing a zero disables the watchdog.
/// By default watchdog is set to 5, which means a 0.5 second timeout.
/// </summary>
const int8_t WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS = 5;

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
      MattzoSBrickMQTTHandler::Handle(message, numSBricks, mySBricks);

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
  Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Starting MattzoTrainController for SBrick...");

  // Setup Mattzo controller.
  setupMattzoController(MATTZO_CONTROLLER_TYPE);

  if (ENABLE_MQTT)
  {
    // Setup a queue with a fixed length that will hold pointers to incoming MQTT messages.
    msg_queue = xQueueCreate(MQTT_INCOMING_QUEUE_LENGTH, sizeof(char *));

    // Start task loop to handle queued MQTT messages.
    xTaskCreatePinnedToCore(handleMQTTMessages, "MQTTHandler", MQTT_TASK_STACK_DEPTH, NULL, MQTT_TASK_PRIORITY, NULL, MQTT_HANDLE_MESSAGE_TASK_COREID);

    // Setup MQTT publisher (with a queue that can hold 1000 messages).
    // MattzoMQTTPublisher::Setup(ROCRAIL_COMMAND_QUEUE, MQTT_OUTGOING_QUEUE_LENGTH);

    // Setup MQTT subscriber.
    MattzoMQTTSubscriber::Setup(ROCRAIL_COMMAND_TOPIC, mqttCallback);
  }

  Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Initializing BLE...");

  // Initialize BLE client.
  NimBLEDevice::init("");

  // Configure BLE scanner.
  scanner = NimBLEDevice::getScan();
  scanner->setInterval(1349);
  scanner->setWindow(449);
  scanner->setActiveScan(true);

  Serial.print("[" + String(xPortGetCoreID()) + "] Setup: Number of SBrick(s) to discover: ");
  Serial.println(sizeof(mySBricks) / sizeof(mySBricks[0]));
}

void loop()
{
  for (int i = 0; i < numSBricks; i++)
  {
    SBrickHubClient *sbrick = mySBricks[i];
    if (!sbrick->IsEnabled())
    {
      // Skip to the next SBrick Hub.
      continue;
    }

    if (!sbrick->IsConnected())
    {
      if (!sbrick->IsDiscovered())
      {
        // SBrick not discovered yet, first discover it.
        sbrick->StartDiscovery(scanner, BLE_SCAN_DURATION_IN_SECONDS);
      }

      if (sbrick->IsDiscovered())
      {
        // SBrick discovered, try to connect now.
        if (!sbrick->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS))
        {
          // Connect attempt failed. Will retry in next loop.
          Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Connect failed");
        }
      }
    }
  }

  if (ENABLE_MQTT && false)
  {
    // Construct message.
    String message = String("Hello world @ ");
    message.concat(millis());

    // Print message we are about to queue.
    Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Queueing outgoing message (" + message + ").");

    // Try to add message to queue (fails if queue is full).
    if (!MattzoMQTTPublisher::QueueMessage(message.c_str()))
    {
      Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Outgoing MQTT message queue full");
    }
  }

  // Print available heap space.
  // Serial.print("[" + String(xPortGetCoreID()) + "] Loop: Available heap: ");
  // Serial.println(xPortGetFreeHeapSize());

  // Delay next scan/connect attempt for a while, allowing the background drive tasks of already connected SBricks to send their periodic commands.
  delay(BLE_CONNECT_DELAY_IN_SECONDS * 1000 / portTICK_PERIOD_MS);
}