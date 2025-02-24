#include <Arduino.h>

#include "DriverTaskDelay.h"
#include "SBrickHub.h"
#include "log4MC.h"

static BLEUUID remoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

const int8_t CMD_BRAKE = 0;
const int8_t CMD_DRIVE = 1;
const int8_t CMD_SET_WATCHDOG_TIMEOUT = 13;
const int8_t CMD_GET_WATCHDOG_TIMEOUT = 14;
const int8_t CMD_BRAKE_WITH_PM = 19;
const int8_t CMD_GET_CHANNEL_STATUS = 34;
const int16_t SBRICK_MAX_CHANNEL_SPEED = 254;
const int16_t SBRICK_MIN_CHANNEL_SPEED = -254;

SBrickHub::SBrickHub(BLEHubConfiguration *config)
	: BLEHub(config)
{
}

bool SBrickHub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
	if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID)) {
		log4MC::error("BLE : Unable to attach to remote control service.");
		return false;
	}

	if (!_remoteControlCharacteristic->canWrite()) {
		log4MC::error("BLE : Remote control characteristic doesn't allow writing.");
		return false;
	}

	uint8_t byteWrite[2] = {CMD_SET_WATCHDOG_TIMEOUT, watchdogTimeOutInTensOfSeconds};
	if (!_remoteControlCharacteristic->writeValue(byteWrite, sizeof(byteWrite), false)) {
		log4MC::error("BLE : Writing remote control characteristic CMD_SET_WATCHDOG_TIMEOUT failed.");
		return false;
	}

	log4MC::vlogf(LOG_INFO, "BLE : Watchdog timeout successfully set to s/10: %u", _remoteControlCharacteristic->readValue<uint8_t>());

	uint8_t byteRead[1] = {CMD_GET_WATCHDOG_TIMEOUT};
	if (!_remoteControlCharacteristic->writeValue(byteRead, sizeof(byteRead), false)) {
		log4MC::error("BLE : Writing remote control characteristic CMD_GET_WATCHDOG_TIMEOUT failed.");
		return false;
	}

	return true;
}

void SBrickHub::DriveTaskLoop()
{
	bool channelAForward = false;
	bool channelBForward = false;
	bool channelCForward = false;
	bool channelDForward = false;

	uint8_t channelAPwr = 0;
	uint8_t channelBPwr = 0;
	uint8_t channelCPwr = 0;
	uint8_t channelDPwr = 0;

	for (;;) {
		for (BLEHubChannelController *controller : _channelControllers) {
			// Update current channel pwr, if needed.
			controller->UpdateCurrentPwrPerc();

			switch (controller->GetHubChannel()) {
			case BLEHubChannel::A:
				channelAForward = controller->IsDrivingForward();
				channelAPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::B:
				channelBForward = controller->IsDrivingForward();
				channelBPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::C:
				channelCForward = controller->IsDrivingForward();
				channelCPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::D:
				channelDForward = controller->IsDrivingForward();
				channelDPwr = getRawChannelPwrForController(controller);
				break;
			case OnboardLED: // not on the sbrick
				break;
			}
		}

		// Construct one drive command for all channels.
		uint8_t byteCmd[13] = {
			CMD_DRIVE,
			BLEHubChannel::A,
			channelAForward,
			channelAPwr,
			BLEHubChannel::B,
			channelBForward,
			channelBPwr,
			BLEHubChannel::C,
			channelCForward,
			channelCPwr,
			BLEHubChannel::D,
			channelDForward,
			channelDPwr};

		// Send drive command.
		if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false)) {
			log4MC::vlogf(LOG_ERR, "SBK : Drive failed. Unabled to write to SBrick characteristic.");
		}

		// Wait 50 milliseconds.
		vTaskDelay(DRIVERTASKDELAY / portTICK_PERIOD_MS);
	}
}

int16_t SBrickHub::MapPwrPercToRaw(int pwrPerc)
{
	// Map absolute speed (no matter the direction) to raw channel speed.
	return map(abs(pwrPerc), 0, 100, 0, SBRICK_MAX_CHANNEL_SPEED);
}

void SBrickHub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
	// TODO: Ignored for now...
}