#include "NimBLEDevice.h"
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
    _address = new NimBLEAddress(deviceAddress);
  }
  
  class SBrickClientCallback : public NimBLEClientCallbacks {
    SBrickHubClient* _psbrick;

  public:
    SBrickClientCallback(SBrickHubClient* sbrick) : NimBLEClientCallbacks()
    {
      _psbrick = sbrick;
    }

    void onConnect(NimBLEClient* pclient) {
      if(pclient->getPeerAddress().equals(*_psbrick->_address)) {
        // Serial.print("onConnect: ");
        // Serial.println(pclient->getPeerAddress().toString().c_str());
        _psbrick->_isConnected = true;
      }
    }

    void onDisconnect(NimBLEClient* pclient) {
      if(pclient->getPeerAddress().equals(*_psbrick->_address)) {
        // Serial.print("onDisconnect: ");
        // Serial.println(_psbrick->_address->toString().c_str());
        _psbrick->_isDiscovered = false;
        _psbrick->_isConnected = false;
        pclient->disconnect();
      }
    }
  };

  class SBrickAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
    SBrickHubClient* _psbrick;
  
  public:
    SBrickAdvertisedDeviceCallbacks(SBrickHubClient* sbrick) : NimBLEAdvertisedDeviceCallbacks()
    {
      _psbrick = sbrick;
    }
      
     /**
       * Called for each advertising BLE server.
       */
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
      // We have found a device, let us now see if it has the address we are looking for.
      if (advertisedDevice->getAddress().equals(*_psbrick->_address)) {
        // Stop active scan.
        NimBLEDevice::getScan()->stop();

        //Serial.print("Discovered our device: ");
        //Serial.println(advertisedDevice.toString().c_str());
        _psbrick->_isDiscovered = true;
      }
    }
  };

  /// <summary>
  /// Start SBrick Hub discovery. 
  /// </summary>
  void StartDiscovery(NimBLEScan* scanner, const uint32_t scanDurationInSeconds = 3) {
    if (_isDiscovering) {
      return;
    }

    // Start discovery.
    _isDiscovering = true;
    _isDiscovered = false;

    // Serial.println("Scan started");
    // Set the callback we want to use to be informed when we have detected a new device.
    if(_advertisedDeviceCallback == nullptr) {
      _advertisedDeviceCallback = new SBrickAdvertisedDeviceCallbacks(this);
    }
    scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
    scanner->start(scanDurationInSeconds, false);
    // Serial.println("Scan stopped");

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

  std::string getDeviceName() {
    return _deviceName;
  }

  /// <summary>
  /// Sets the watchdog timeout (0D &lt; timeout in 0.1 secs, 1 byte &gt;)
  /// The purpose of the watchdog is to stop driving in case of an application failure.
  /// Watchdog starts when the first DRIVE command is issued during a connection.
  /// Watchdog is stopped when all channels are either set to zero drive, or are braking.
  /// The value is saved to the persistent store.
  /// The recommended watchdog frequency is 0.2-0.5 seconds, but a smaller and many larger settings are also available.
  /// Writing a zero disables the watchdog.
  /// By default watchdog is set to 5, which means a 0.5 second timeout.
  /// </summary>
  //void SetWatchdogTimeout(float seconds)
  //{
  //  if (attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID) &&
  //    _remoteControlCharacteristic->canWrite()) {
  //
  //  }
  //
  //  // Safety check before writing anything (not needed though).
  //  if (_characteristic.CharacteristicProperties.HasFlag(GattCharacteristicProperties.Write))
  //  {
  //    var setWatchDogCommand = new List<byte>{ (byte)Command.SetWatchDogTimeOut, (byte)(seconds * 10) };
  //
  //    try
  //    {
  //      var status = await _characteristic.WriteValueAsync(setWatchDogCommand.ToArray().AsBuffer());
  //
  //      if (status == GattCommunicationStatus.Success)
  //      {
  //        // ...
  //      }
  //
  //      if (status == GattCommunicationStatus.Unreachable)
  //      {
  //        // ...
  //      }
  //    }
  //    catch (Exception ex)
  //    {
  //      throw;
  //      //// TODO: Signal error
  //    }
  //  }
  //}

//  void Drive() {
//
//  }
//
//  void EmergencyBreak() {
//    // Reset all channels.
//    Drive(0, 0, 0, 0);
//  }
private:

  bool connectToServer() {
    // Serial.print("Connecting to ");
    // Serial.println(_address->toString().c_str());

    if(_sbrick == nullptr) {
      _sbrick = NimBLEDevice::createClient(*_address);
      //Serial.println(" - Created client");
    }

    if(_clientCallback == nullptr) {
      _clientCallback = new SBrickClientCallback(this);
    }
    _sbrick->setClientCallbacks(_clientCallback);

    // Connect to the remote BLE Server.
    if(!_sbrick->connect(false)) {
      // Serial.println("- Failed to connect to server");
      return false;
    }
    //Serial.println(" - Connected to server");

    // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
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

  static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Serial.print("Notify callback for characteristic ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    // Serial.print(" of data length ");
    // Serial.println(length);
    // Serial.print("data: ");
    // Serial.println((char*)pData);
  }

  // Private members.
  bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID) {
    if (_remoteControlCharacteristic != nullptr) {
      return true;
    }

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
  NimBLEAddress* _address = nullptr;
  NimBLEClient* _sbrick = nullptr;
  NimBLEAdvertisedDeviceCallbacks* _advertisedDeviceCallback = nullptr;
  SBrickClientCallback* _clientCallback = nullptr;
  bool _isDiscovering = false;
  bool _isDiscovered = false;
  bool _isConnected = false;
  NimBLERemoteService* _remoteControlService = nullptr;
  NimBLERemoteCharacteristic* _remoteControlCharacteristic = nullptr;
  NimBLERemoteCharacteristic* _genericAccessCharacteristic = nullptr;
  NimBLERemoteCharacteristic* _deviceInformationCharacteristic = nullptr;
};
