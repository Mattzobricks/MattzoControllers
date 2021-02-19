// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

WiFiClient wifiSubscriberClient;
PubSubClient mqttSubscriberClient(wifiSubscriberClient);

/// <summary>
/// Class used to publish messages to an MQTT broker.
/// </summary>
class MattzoMQTTSubscriber {

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
  static int HandleMessageDelayInMilliseconds;

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
  /// Setup the MQTT Subscriber.
  /// </summary>
  /// <param name="topic">Topic to subscribe to.</param>
  /// <param name="callback">Callback method to call when a message arrives.</param>
  static void Setup(char* topic, void (*callback)(char *, uint8_t *, unsigned int)) {
#if !defined(ESP32)
#error "Error: this sketch is designed for ESP32 only."
#endif
    
    if (_setupCompleted) {
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
    _subscriberName = String(mattzoControllerName_char);
    _subscriberName.concat("Subscriber");

    // Start task loop.
    xTaskCreatePinnedToCore(taskLoop, "MQTTSubscriber", StackDepth, NULL, TaskPriority, NULL, CoreID);

    // Setup completed.
    _setupCompleted = true;
  }

private:

  static String _subscriberName;
  static char* _topic;
  static bool _setupCompleted;

  // Time of the last sent ping.
  static unsigned long lastPing;

  /// <summary>
  /// Sends the given message to the MQTT broker.
  /// </summary>
  /// <param name="parm">Message to send.</param>
  static void sendMessage(char* topic, const char* message) {
    Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: [" + topic + "] " + message);
    mqttSubscriberClient.publish(topic, message);
  }

  /// <summary>
  /// Reconnects the MQTT client to the broker (blocking).
  /// </summary>
  static void reconnect() {
    while (!mqttSubscriberClient.connected()) {
      Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber attempting to connect...");

      String lastWillMessage;
      if (TRIGGER_EBREAK_UPON_DISCONNECT) {
        lastWillMessage = "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
      }
      else {
        lastWillMessage = "<info msg=\"mc_disconnected\" source=\"lastwill\" mc=\"" + mattzoControllerName + "\"/>";
      }
      char lastWillMessage_char[lastWillMessage.length() + 1];
      lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

      if (mqttSubscriberClient.connect(_subscriberName.c_str(), "rocrail/service/command", 0, false, lastWillMessage_char)) {
        Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber connected");
        mqttSubscriberClient.subscribe(_topic);
        Serial.println("[" + String(xPortGetCoreID()) + "] MQTT: Subscriber subscribed to topic '" + _topic + "'");
      }
      else {
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
  static void taskLoop(void* parm) {
    if (!_setupCompleted) {
      throw "Setup not completed. Execute .Setup() first.";
    }

    // Loop forever
    while (1) {
      // Wait for connection to Wifi (because we need it to contact the broker).
      MattzoWifiClient::Assert();

      // Wait for connection to MQTT (because we need it to send messages to the broker).
      if (!mqttSubscriberClient.connected())
      {
        reconnect();
      }

      if (PingDelayInMilliseconds > 0 && millis() - lastPing >= PingDelayInMilliseconds) {
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
};

// Initialize static members.
int MattzoMQTTSubscriber::ReconnectDelayInMilliseconds = 1000;
int MattzoMQTTSubscriber::PingDelayInMilliseconds = 1000;
int MattzoMQTTSubscriber::HandleMessageDelayInMilliseconds = 100;
uint8_t MattzoMQTTSubscriber::TaskPriority = 1;
int8_t MattzoMQTTSubscriber::CoreID = 0;
uint32_t MattzoMQTTSubscriber::StackDepth = 2048;
bool MattzoMQTTSubscriber::_setupCompleted = false;
unsigned long MattzoMQTTSubscriber::lastPing = millis();
String MattzoMQTTSubscriber::_subscriberName = "Unknown";
char* MattzoMQTTSubscriber::_topic = "Unknown";
