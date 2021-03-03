#pragma once

int getMattzoControllerId();
void createMattzoControllerName(String mattzoControllerType, int mattzoControllerId);
void setupMattzoController(String controllerType);

// Name of this controller, e.g. MattzoSignalController12345 or MattzoSwitchController34567
extern String mattzoControllerName;

// Name of this controller als char[]
extern char mattzoControllerName_char[50];

// If the mattzoControllerId can not be retrieved from EEPROM, a new mattzoControllerId between a and MAX_CONTROLLER_ID is generated and stored in EEPROM.
#define MAX_CONTROLLER_ID 65000