// Built-in EEPROM library
// Install via the built-in Library Manager of the Arduino IDE
#include <EEPROM.h>

#include "MattzoController_Library.h"

// EEPROM ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
String eepromIDString = "MattzoController";

// mattzoControllerId. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
unsigned int mattzoControllerId;

// Name of this controller, e.g. MattzoSignalController12345 or MattzoSwitchController34567
String mattzoControllerName;

// Name of this controller als char[]
char mattzoControllerName_char[50];

// ****************
// EEPROM functions
// ****************

int getMattzoControllerId() {
  int i;
  int controllerNoHiByte;
  int controllerNoLowByte;

  // set-up EEPROM for read/write operations
  EEPROM.begin(512);

  // Check if the first part of the memory is filled with the MattzoController ID string.
  // This is the case if the controller has booted before with a MattzoController firmware.
  bool idStringCheck = true;
  for (i = 0; i < eepromIDString.length(); i++) {
    char charEeprom = EEPROM.read(i);
    char charIDString = eepromIDString.charAt(i);
    if (charEeprom != charIDString) {
      idStringCheck = false;
      break;
    }
  }

  int paramsStartingPosition = eepromIDString.length();
  if (idStringCheck) {
    // load controller number from preferences
    controllerNoHiByte = EEPROM.read(paramsStartingPosition);
    controllerNoLowByte = EEPROM.read(paramsStartingPosition + 1);
    mattzoControllerId = controllerNoHiByte * 256 + controllerNoLowByte;
    Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Loaded MattzoControllerId from EEPROM: " + String(mattzoControllerId));
  }
  else {
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    delay(5000);

    // store EEPROM ID String in EEPROM
    Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Initializing controller preferences on first start-up...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controller number between 1 and MAX_CONTROLLER_ID and store in EEPROM
    mattzoControllerId = random(1, MAX_CONTROLLER_ID);
    controllerNoHiByte = mattzoControllerId / 256;
    controllerNoLowByte = mattzoControllerId % 256;
    EEPROM.write(paramsStartingPosition, controllerNoHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerNoLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("[" + String(xPortGetCoreID()) + "] Ctrl: Assigned random controller no " + String(mattzoControllerId) + " and stored to EEPROM");
  }

  return mattzoControllerId;
}

void createMattzoControllerName(String mattzoControllerType, int mattzoControllerId) {
  // set mattzoControllerName
  mattzoControllerName = mattzoControllerType + String(mattzoControllerId);
  mattzoControllerName.toCharArray(mattzoControllerName_char, mattzoControllerName.length() + 1);
}

// ***********************
// setup and loop function
// ***********************

void setupMattzoController(String controllerType) {
  randomSeed(ESP.getCycleCount());
  mattzoControllerId = getMattzoControllerId();
  createMattzoControllerName(controllerType, mattzoControllerId);
}