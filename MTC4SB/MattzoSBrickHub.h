#include "BLEDevice.h"
#include "SBrickConst.h"

/*
*   - setWatchdogTimeout()
*   - subscribeToNotifications(callback)
*       pMyRemoteCharacteristic->registerForNotify(notifyCallback)
*       void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify)
* SBrickHub.Drive(command, command, command, command)
* SBrickHub.DriveChannel(channel, command)
* SBrickHub.EmergencyBreak()
*/

static BLEUUID sbrickRemoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID sbrickRemoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

/// <summary>
/// Class used to connect to a SBrick Hub.
/// </summary>
class SBrickHubClient {

public:
  // Public members

  SBrickHubClient(std::string deviceName, std::string deviceAddress) {
    _deviceName = deviceName;
    _address = new BLEAddress(deviceAddress);
  }
  
  class SBrickClientCallback : public BLEClientCallbacks {
    SBrickHubClient* _sbrick;

  public:
    SBrickClientCallback(SBrickHubClient* sbrick) : BLEClientCallbacks()
    {
      _sbrick = sbrick;
    }

    void onConnect(BLEClient* pclient) {
      Serial.println("onConnect");
      _sbrick->_isConnected = true;
    }

    void onDisconnect(BLEClient* pclient) {
      Serial.println("onDisconnect");
      _sbrick->_isDiscovered = false;
      _sbrick->_isConnected = false;
      pclient->disconnect();
    }
  };

  class SBrickAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    SBrickHubClient *_sbrick;
  
  public:
    SBrickAdvertisedDeviceCallbacks(SBrickHubClient *sbrick) : BLEAdvertisedDeviceCallbacks()
    {
      _sbrick = sbrick;
    }
      
     /**
       * Called for each advertising BLE server.
       */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // We have found a device, let us now see if it has the address we are looking for.
      if (advertisedDevice.getAddress().equals(*_sbrick->_address)) {
        // Stop active scan.
        BLEDevice::getScan()->stop();

        //Serial.print("Discovered our device: ");
        //Serial.println(advertisedDevice.toString().c_str());
        _sbrick->_isDiscovered = true;
      }
    }
  };

  /// <summary>
  /// Start SBrick Hub discovery. 
  /// </summary>
  void StartDiscovery(BLEScan* scanner, const uint32_t scanDurationInSeconds = 3) {
    if (_isDiscovering) {
      return;
    }

    // Start discovery.
    _isDiscovering = true;
    _isDiscovered = false;

    Serial.println("Scan started");
    // Set the callback we want to use to be informed when we have detected a new device.
    if(_advertisedDeviceCallback == nullptr) {
      _advertisedDeviceCallback = new SBrickAdvertisedDeviceCallbacks(this);
    }
    scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
    scanner->start(scanDurationInSeconds, false);
    Serial.println("Scan stopped");

    _isDiscovering = false;
  }

  /// <summary>
  /// Connect to the SBrick Hub.
  /// </summary>
  bool Connect() {
    return connectToServer();
  }

  bool IsDiscovered() {
    return _isDiscovered;
  }

  bool IsConnected() {
    return _isConnected;
  }

private:

  bool connectToServer() {
    Serial.print("Connecting to ");
    Serial.println(_address->toString().c_str());

    if(_sbrick == nullptr) {
      _sbrick = BLEDevice::createClient();
      //Serial.println(" - Created client");
    }

    if(_clientCallback == nullptr) {
      _clientCallback = new SBrickClientCallback(this);
    }
    _sbrick->setClientCallbacks(_clientCallback);

    // Connect to the remote BLE Server.
    if(!_sbrick->connect(*_address)) {
      Serial.println("- Failed to connect to server");
      return false;
    }
    //Serial.println(" - Connected to server");

    // Try obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    if (!attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID)) {
      // Failed to find the remote control service or characteristic.
      _sbrick->disconnect();
      return false;
    }

    // Read the value of the characteristic.
    //if(_remoteControlCharacteristic->canRead()) {
    //  std::string value = _remoteControlCharacteristic->readValue();
    //  //Serial.print("The characteristic value was: ");
    //  //Serial.println(value.c_str());
    //}

    //if(_remoteControlCharacteristic->canNotify()) {
    //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
    //}
    
    return true;
  }

  static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
  }

  // Private members.
  bool attachCharacteristic(BLEUUID serviceUUID, BLEUUID characteristicUUID) {
    // Obtain a reference to the service remote control service in the BLE server.
    _remoteControlService = _sbrick->getService(serviceUUID);
    if (_remoteControlService == nullptr) {
      return false;
    }

    // Obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    _remoteControlCharacteristic = _remoteControlService->getCharacteristic(characteristicUUID);

    return _remoteControlCharacteristic != nullptr;
  }

  std::string _deviceName;
  BLEAddress* _address = nullptr;
  BLEClient* _sbrick = nullptr;
  BLEAdvertisedDeviceCallbacks* _advertisedDeviceCallback = nullptr;
  SBrickClientCallback* _clientCallback = nullptr;
  boolean _isDiscovering = false;
  boolean _isDiscovered = false;
  boolean _isConnected = false;
  BLERemoteService* _remoteControlService = nullptr;
  BLERemoteCharacteristic* _remoteControlCharacteristic;
  BLERemoteCharacteristic* _genericAccessCharacteristic;
  BLERemoteCharacteristic* _deviceInformationCharacteristic;
};
