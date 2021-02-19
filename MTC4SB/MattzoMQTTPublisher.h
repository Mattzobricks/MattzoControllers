// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

static QueueHandle_t msg_queue;

WiFiClient wifiPublisherClient;
PubSubClient mqttPublisherClient(wifiPublisherClient);

/// <summary>
/// Class used to publish messages to an MQTT broker.
/// </summary>
class MattzoMQTTPublisher {

public:
  // Public static members

  /// <summary>
  /// Reconnect delay in milliseconds. This configures the delay between reconnect attempts.
  /// </summary>
  static int ReconnectDelayInMilliseconds;

  /// <summary>
  /// Ping delay in milliseconds. This configures the delay between ping messages (0 = don't send ping messages).
  /// </summary>
  static int PingDelayInMilliseconds;

  /// <summary>
  /// Send message delay in milliseconds. This configures the delay between send message attempts.
  /// </summary>
  static int SendMessageDelayInMilliseconds;

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

  // Methods

  /// <summary>
  /// Setup the MQTT Publisher.
  /// </summary>
  /// <param name="messageQueueLength">Message queue length. This configures the number of messages the queue can hold, before the publisher starts dropping them.</param>
  static void Setup(const int messageQueueLength = 10) {
#if !defined(ESP32)
#error "Error: this sketch is designed for ESP32 only."
#endif

    if (setupCompleted) {
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
    msg_queue = xQueueCreate(messageQueueLength, sizeof(char*));

    // Start task loop.
    xTaskCreatePinnedToCore(taskLoop, "MQTTPublisher", StackDepth, NULL, TaskPriority, NULL, CoreID);

    // Setup completed.
    setupCompleted = true;
  }

  /// <summary>
  /// Puts the given message on the queue.
  /// </summary>
  /// <param name="message">Message to add to the queue.</param>
  /// <returns>Boolean value indicating whether the given message was added to the queue. Returns `false` when the queue was full.</returns>
  static bool QueueMessage(const char* message) {
    if (!setupCompleted) {
      Serial.println("Setup not completed. Execute .Setup() first!");
      throw "Setup not completed. Execute .Setup() first!";
    }

    // Allocate memory to hold the message.
    char* messagePtr = (char*)malloc(strlen(message) + 1);

    // Copy the message to the allocated memory block.
    strcpy(messagePtr, message);

    // Store pointer to message in queue (don't block if the queue is full).
    return xQueueSendToBack(msg_queue, (void*)&messagePtr, (TickType_t)0) == pdTRUE;
  }

private:

  static bool setupCompleted;
  static String publisherName;

  // Time of the last sent ping.
  static unsigned long lastPing;

  /// <summary>
  /// Sends all messages that are currently queued to the MQTT broker.
  /// </summary>
  static void sendMessages() {
    char* message;

    // See if there's a message in the queue (do not block).
    while (xQueueReceive(msg_queue, (void*)&message, (TickType_t)0) == pdTRUE) {
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
  static void sendMessage(const char* topic, const char* message) {
    Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: [" + topic + "] " + message);
    mqttPublisherClient.publish(topic, message);
  }
  
  /// <summary>
  /// Reconnects the MQTT client to the broker (blocking).
  /// </summary>
  static void reconnect() {
    while (!mqttPublisherClient.connected()) {
      Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Publisher attempting to connect...");

      String lastWillMessage;
      if (TRIGGER_EBREAK_UPON_DISCONNECT) {
        lastWillMessage = "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
      }
      else {
        lastWillMessage = "<info msg=\"mc_disconnected\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
      }
      char lastWillMessage_char[lastWillMessage.length() + 1];
      lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

      if (mqttPublisherClient.connect(publisherName.c_str(), "rocrail/service/command", 0, false, lastWillMessage_char)) {
        Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Publisher connected");
      }
      else {
        Serial.print("[" + String(xPortGetCoreID()) + "] MQTT: Publisher connect failed, rc=");
        Serial.print(mqttPublisherClient.state());
        Serial.println(". Try again in a few seconds...");
        
        // Wait w litte while before retrying.
        vTaskDelay(ReconnectDelayInMilliseconds / portTICK_PERIOD_MS);
      }
    }
  }

  /// <summary>
  /// The main (endless) task loop.
  /// </summary>
  /// <param name="parm"></param>
  static void taskLoop(void* parm) {
    if (!setupCompleted) {
      throw "Setup not completed. Execute .Setup() first.";
    }

    // Loop forever
    while (1) {
      // Wait for connection to Wifi (because we need it to contact the broker).
      MattzoWifiClient::Assert();

      // Wait for connection to MQTT (because we need it to send messages to the broker).
      if (!mqttPublisherClient.connected())
      {
        reconnect();
      }

      if (PingDelayInMilliseconds > 0 && millis() - lastPing >= PingDelayInMilliseconds) {
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
};

// Initialize static members.
int MattzoMQTTPublisher::ReconnectDelayInMilliseconds = 5000;
int MattzoMQTTPublisher::PingDelayInMilliseconds = 1000;
int MattzoMQTTPublisher::SendMessageDelayInMilliseconds = 500;
uint8_t MattzoMQTTPublisher::TaskPriority = 1;
int8_t MattzoMQTTPublisher::CoreID = 0;
uint32_t MattzoMQTTPublisher::StackDepth = 2048;
bool MattzoMQTTPublisher::setupCompleted = false;
unsigned long MattzoMQTTPublisher::lastPing = millis();
String MattzoMQTTPublisher::publisherName = String("Unknown");
