#include <Arduino.h>

#include "BLEDeviceCallbacks.h"
#include "BLEHubScanner.h"
#include "log4MC.h"

BLEHubScanner::BLEHubScanner()
{
	_advertisedDeviceCallback = nullptr;
	_isDiscovering = false;

	NimBLEDevice::init("");

	// Configure BLE scanner.
	_scanner = NimBLEDevice::getScan();
	// _scanner->setActiveScan(true);   // Set active scanning, this will get more data from the advertiser.
	// _scanner->setInterval(97);       // How often the scan occurs / switches channels; in milliseconds.
	// _scanner->setWindow(37);         // How long to scan during the interval; in milliseconds.
	// _scanner->setMaxResults(20);     // The number of results to limit storage to (0 = do not store the scan results, use callback only).
	_scanner->setDuplicateFilter(true); // Set that the BLE controller should only report results from devices it has not already seen.
}

void BLEHubScanner::StartDiscovery(std::vector<BLEHub *> &hubs, const uint32_t scanDurationInSeconds)
{
	if (_isDiscovering) {
		return;
	}

	// Start discovery.
	_isDiscovering = true;

	log4MC::vlogf(LOG_INFO, "BLE : Scanning for %u hub(s)...", hubs.size());

	// TODO: We could start using the whitelist, but then we'd have to manage it too.
	for (BLEHub *hub : hubs) {
		BLEDevice::whiteListAdd(hub->GetAddress());
	}

	// Set the callback we want to use to be informed when we have detected a new device.
	if (_advertisedDeviceCallback == nullptr) {
		_advertisedDeviceCallback = new BLEDeviceCallbacks(hubs);
		_scanner->setScanCallbacks(_advertisedDeviceCallback, false);
	}
	if (BLEDevice::getWhiteListCount() != 0) {
		_scanner->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL);
	} else {
		_scanner->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL);
	}
	_scanner->start(scanDurationInSeconds, false);

	log4MC::vlogf(LOG_INFO, "BLE : Scanning for %u hub(s) aborted.", hubs.size());

	// Discovery stopped.
	_isDiscovering = false;
}