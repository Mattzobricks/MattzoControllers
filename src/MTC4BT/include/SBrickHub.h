#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define SBRICK_REMOTECONTROL_SERVICE_UUID "4dc591b0-857c-41de-b5f1-15abda665b0c"
#define SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID "02b8cbcc-0e25-4bda-8790-a15f53e6010f"

class SBrickHub : public BLEHub
{
  public:
	SBrickHub(BLEHubConfiguration *config);
	bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
	void DriveTaskLoop();
	int16_t MapPwrPercToRaw(int pwrPerc);
	/**
	 * @brief Callback function for notifications of a specific characteristic
	 * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
	 * @param [in] pData The pointer to the received data
	 * @param [in] length The length of the data array
	 * @param [in] isNotify
	 */
	void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
};