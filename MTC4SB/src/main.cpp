#include <Arduino.h>

#include "NimBLEDevice.h"

// MattzoControllerType
#define MATTZO_CONTROLLER_TYPE "MTC4SB"

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT false

//#include <NimBLEDevice.h>

// MattzoController network configuration (the following file needs to be moved into the Arduino library folder):
#include <MattzoController_Network_Configuration.h>

#include "MattzoController_Library.h"
#include "MattzoWifiClient.h"
//#include "MattzoMQTTPublisher.h"
//#include "MattzoMQTTSubscriber.h"
#include "SBrickConst.h"
#include "MattzoSBrickHub.h"

// Globals
NimBLEScan* scanner;
SBrickHubClient* mySBricks[3] = {
    new SBrickHubClient("YC66405", "00:07:80:d0:47:43"),
    new SBrickHubClient("HE10233", "00:07:80:d0:3a:f2"),
    new SBrickHubClient("BC60052", "88:6b:0f:23:78:10")
};

void setup()
{
   // Configure Serial.
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output).
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Starting MattzoTrainController for SBrick...");

  // Setup Mattzo controller.
  setupMattzoController();

  // Initialize BLE client.
  NimBLEDevice::init("");

  // Configure a BLE scanner.
  scanner = NimBLEDevice::getScan();
  scanner->setInterval(1349);
  scanner->setWindow(449);
  scanner->setActiveScan(true);

  Serial.print("Number of SBrick(s) to discover: ");
  Serial.println(sizeof(mySBricks)/sizeof(mySBricks[0]));
}

void loop()
{
  for (int i = 0; i < sizeof(mySBricks)/sizeof(mySBricks[0]); i++) {
    SBrickHubClient* sbrick = mySBricks[i];

    Serial.print(sbrick->getDeviceName().c_str());
    Serial.print(": discovered=");
    Serial.print(sbrick->IsDiscovered());
    Serial.print(", connected=");
    Serial.println(sbrick->IsConnected());
    
    if (!sbrick->IsConnected()) {
      if (!sbrick->IsDiscovered()) {
          sbrick->StartDiscovery(scanner);
      }
  
      if(sbrick->IsDiscovered()) {
        sbrick->Connect();
      }
    }
  }

  // Print available heap space.
  Serial.print("Available heap: ");
  Serial.println(xPortGetFreeHeapSize());
  
  // Wait before trying again.
  vTaskDelay(3000 / portTICK_PERIOD_MS);
}