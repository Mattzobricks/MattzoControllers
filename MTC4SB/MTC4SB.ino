#include <DNSServer.h>


/** MattzoTrainController for SBrick 
 *
 *  Created:  Februari 12 2021
 *  Author:   R. Brink
 * 
*/

// MattzoControllerType
#define MATTZO_CONTROLLER_TYPE "MTC4SB"

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT false

//#include <NimBLEDevice.h>

// MattzoController network configuration (the following file needs to be moved into the Arduino library folder):
#include <MattzoController_Network_Configuration.h>

#include "MattzoController_Library.h"
#include "MattzoWifiClient.h"
#include "MattzoMQTTPublisher.h"
#include "MattzoMQTTSubscriber.h"
#include "SBrickConst.h"
#include "SBrick.h"

#define SBRICK_ADDRESS "00:07:80:d0:47:43"

// Globals
SBrick sbrick;

void connectSBricks() {
  if (!sbrick.isConnected()) {
    if (sbrick.isConnecting()) {
      sbrick.connectHub();

      if (sbrick.isConnected()) {
        Serial.print("Connected to SBrick '");
        Serial.print(sbrick.getHubName().c_str());
        Serial.print("' (");
        Serial.print(sbrick.getHubAddress().toString().c_str());
        Serial.println(")");
      }
      else {
        Serial.println("Connection attempt to SBrick refused.");
        //myHubs[i].initHub();
      }
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Received MQTT message [" + String(topic) + "]: " + String(msg));
}

void setup() {
  // Configure Serial.
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output).
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Starting MattzoTrainController for SBrick...");

  // Setup Mattzo controller.
  setupMattzoController();

  // Setup MQTT publisher.
  MattzoMQTTPublisher::Setup(12);

  // Setup MQTT subscriber.
  MattzoMQTTSubscriber::Setup("rocrail/service/command", mqttCallback);

  /** Initialize SBrick, device address specified as we are advertising */
  //sbrick.init(SBRICK_ADDRESS);
}

void loop() {
  // Construct message.
  String message = String("Hello world @ ");
  message.concat(millis());

  // Print message we are about to queue.
  Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Queing message (" + message + ").");

  // Try to add message to queue, fails if queue is full.
  if (!MattzoMQTTPublisher::QueueMessage(message.c_str())) {
    Serial.println("[" + String(xPortGetCoreID()) + "] Loop: Queue full");
  }

  // Wait before trying again.
  vTaskDelay(100 / portTICK_PERIOD_MS);

  //connectSBricks();
}
