/*
 * SBrick.cpp - Arduino base class for controlling Powered UP and Boost controllers
 * 
 * (c) Copyright 2020 - Cornelius Munz
 * Released under MIT License
 * 
*/

//#if defined(ESP32)

#include "SBrick.h"

/** 
 * Callback if a scan has ended with the results of found devices 
 * only needed to enforce the non blocking scan start
 */
void scanEndedCallback(NimBLEScanResults results)
{
    log_d("Number of devices: %d", results.getCount());
    for (int i = 0; i < results.getCount(); i++)
    {
        log_d("device[%d]: %s", i, results.getDevice(i).toString().c_str());
    }
}

/**
 * Derived class which could be added as an instance to the BLEClient for callback handling
 * The current hub is given as a parameter in the constructor to be able to set the 
 * status flags on a disconnect event accordingly
 */
class SBrickClientCallback : public BLEClientCallbacks
{
  SBrick* _sbrick;

public:
    SBrickClientCallback(SBrick *sbrick) : BLEClientCallbacks()
    {
        _sbrick = sbrick;
    }

    void onConnect(BLEClient *bleClient)
    {
    }

    void onDisconnect(BLEClient *bleClient)
    {
        _sbrick->_isConnecting = false;
        _sbrick->_isConnected = false;
        log_d("disconnected client");
    }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class SBrickAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    SBrick *_sbrick;

public:
    SBrickAdvertisedDeviceCallbacks(SBrick *sbrick) : NimBLEAdvertisedDeviceCallbacks()
    {
        _sbrick = sbrick;
    }

    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
        //Found a device, check if the service is contained and optional if address fits requested address
        //Serial.print("advertised device: ");
        //Serial.println(advertisedDevice->toString().c_str());

        if (_sbrick->_requestedDeviceAddress == nullptr || (_sbrick->_requestedDeviceAddress && advertisedDevice->getAddress().equals(*_sbrick->_requestedDeviceAddress)))
        {
            advertisedDevice->getScan()->stop();
            _sbrick->_pServerAddress = new BLEAddress(advertisedDevice->getAddress());
            _sbrick->_hubName = advertisedDevice->getName();

            if (advertisedDevice->haveManufacturerData())
            {
                uint8_t *manufacturerData = (uint8_t *)advertisedDevice->getManufacturerData().data();
                uint8_t manufacturerDataLength = advertisedDevice->getManufacturerData().length();

                for(int i = 0; i < manufacturerDataLength; i++){
                  //Serial.println(manufacturerData[i]);
                }
                
                if (manufacturerDataLength >= 3)
                {
                    //check device type ID
                    /*
                    switch (manufacturerData[3])
                    {
                    case BLEManufacturerData::SBRICK_AND_SBRICK_PLUS_ID:
                        _sbrick->_product = SBrickProduct::SBRICK_AND_SBRICK_PLUS;
                        break;
                    case BLEManufacturerData::SBRICK_LIGHT_ID:
                        _sbrick->_product = SBrickProduct::SBRICK_LIGHT;
                        break;
                    default:
                        _sbrick->_product = SBrickProduct::UNKNOWN;
                        break;
                    }
                    */
                }
            }

            /** Ready to connect now */ 
            _sbrick->_isConnecting = true;
        }
    }
};

/**
 * @brief Write value to the remote characteristic
 * @param [in] command byte array which contains the ble command
 * @param [in] size length of the command byte array
 */
//void SBrick::WriteValue(byte command[], int size)
//{
//    byte commandWithCommonHeader[size + 2] = {(byte)(size + 2), 0x00};
//    memcpy(commandWithCommonHeader + 2, command, size);
//    _pRemoteCharacteristic->writeValue(commandWithCommonHeader, sizeof(commandWithCommonHeader), false);
//}

/**
 * @brief Register a device on a defined port. This will store the device
 * in the connectedDevices array. This method will be called if a port connection
 * event is triggered by the hub
 * 
 * @param [in] port number where the device is connected
 * @param [in] device type of the connected device
 */
//void SBrick::registerPortDevice(byte portNumber, byte deviceType)
//{
//    log_d("port: %x, device type: %x", portNumber, deviceType);
//    Device newDevice = {portNumber, deviceType, nullptr};
//    connectedDevices[numberOfConnectedDevices] = newDevice;
//    numberOfConnectedDevices++;
//}

/**
 * @brief Remove a device from the connectedDevices array. This method
 * will be called if a port disconnection event is triggered by the hub
 * 
 * @param [in] port number where the device is connected
 */
//void SBrick::deregisterPortDevice(byte portNumber)
//{
//    log_d("port: %x", portNumber);
//
//    bool hasReachedRemovedIndex = false;
//    for (int i = 0; i < numberOfConnectedDevices; i++)
//    {
//        if (hasReachedRemovedIndex)
//        {
//            connectedDevices[i - 1] = connectedDevices[i];
//        }
//        if (!hasReachedRemovedIndex && connectedDevices[i].PortNumber == portNumber)
//        {
//            hasReachedRemovedIndex = true;
//        }
//    }
//    if (numberOfConnectedDevices > 0)
//    {
//        numberOfConnectedDevices--;
//    }
//}

/**
 * @brief Activate device for receiving updates. E.g. activate a color/distance sensor to
 * write updates on the characteristic if a value has changed. An optional callback could be
 * regeistered here. This function will be called if the update event will occur. 
 * 
 * @param [in] port number where the device is connected
 * @param [in] callback function which will be called on an update event
 */
//void SBrick::activatePortDevice(byte portNumber, PortValueChangeCallback portValueChangeCallback)
//{
//    byte deviceType = getDeviceTypeForPortNumber(portNumber);
//    activatePortDevice(portNumber, deviceType, portValueChangeCallback);
//}

/**
 * @brief Activate device for receiving updates. E.g. activate a color/distance sensor to
 * write updates on the characteristic if a value has changed. An optional callback could be
 * regeistered here. This function will be called if the update event will occur. The Update mode 
 * is currently fixed based on the device type
 * 
 * @param [in] port number where the device is connected
 * @param [in] deviceType of the connected port
 * @param [in] callback function which will be called on an update event
 */
//void SBrick::activatePortDevice(byte portNumber, byte deviceType, PortValueChangeCallback portValueChangeCallback)
//{
//    byte mode = getModeForDeviceType(deviceType);
//    log_d("port: %x, device type: %x, callback: %x, mode: %x", portNumber, deviceType, portValueChangeCallback, mode);
//    int deviceIndex = getDeviceIndexForPortNumber(portNumber);
//    if (deviceIndex < 0)
//    {
//        return;
//    }
//    connectedDevices[deviceIndex].Callback = portValueChangeCallback;
//    byte activatePortDeviceMessage[8] = {0x41, portNumber, mode, 0x01, 0x00, 0x00, 0x00, 0x01};
//    WriteValue(activatePortDeviceMessage, 8);
//}

/**
 * @brief Deactivate device for receiving updates. 
 * 
 * @param [in] port number where the device is connected
 */
//void SBrick::deactivatePortDevice(byte portNumber)
//{
//    byte deviceType = getDeviceTypeForPortNumber(portNumber);
//    deactivatePortDevice(portNumber, deviceType);
//}

/**
 * @brief Deactivate device for receiving updates. 
 * 
 * @param [in] port number where the device is connected
 * @param [in] device type 
 */
//void SBrick::deactivatePortDevice(byte portNumber, byte deviceType)
//{
//    byte mode = getModeForDeviceType(deviceType);
//    byte deactivatePortDeviceMessage[8] = {0x41, portNumber, mode, 0x01, 0x00, 0x00, 0x00, 0x00};
//    WriteValue(deactivatePortDeviceMessage, 8);
//}

/**
 * @brief Parse the incoming characteristic notification for a Device Info Message
 * @param [in] pData The pointer to the received data
 */
void SBrick::parseDeviceInfo(uint8_t *pData)
{
    //if (_hubPropertyChangeCallback != nullptr)
    //{
    //    _hubPropertyChangeCallback(this, (HubPropertyReference)pData[3], pData);
    //    return;
    //}

    //if (pData[3] == (byte)HubPropertyReference::ADVERTISING_NAME)
    //{
    //    log_d("advertising name: %s", parseHubAdvertisingName(pData).c_str());
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::BUTTON)
    //{
    //    log_d("hub button state: %x", parseHubButton(pData));
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::FW_VERSION)
    //{
    //    Version version = parseVersion(pData);
    //    log_d("version: %d-%d-%d (%d)", version.Major, version.Minor, version.Bugfix, version.Build);
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::HW_VERSION)
    //{
    //    Version version = parseVersion(pData);
    //    log_d("version: %d-%d-%d (%d)", version.Major, version.Minor, version.Bugfix, version.Build);
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::RSSI)
    //{
    //    log_d("rssi: ", parseRssi(pData));
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::BATTERY_VOLTAGE)
    //{
    //    log_d("battery level: %d %", parseBatteryLevel(pData));
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::BATTERY_TYPE)
    //{
    //    log_d("battery type: %d", parseBatteryType(pData));
    //    return;
    //}
    //else if (pData[3] == (byte)HubPropertyReference::SYSTEM_TYPE_ID)
    //{
    //    log_d("system type id: %x", parseSystemTypeId(pData));
    //    return;
    //}
}

/**
 * @brief Parse the incoming characteristic notification for a Port Message
 * @param [in] pData The pointer to the received data
 */
//void SBrick::parsePortMessage(uint8_t *pData)
//{
//    byte port = pData[3];
//    bool isConnected = (pData[4] == 1 || pData[4] == 2) ? true : false;
//    if (isConnected)
//    {
//        log_d("port %x is connected with device %x", port, pData[5]);
//        registerPortDevice(port, pData[5]);
//    }
//    else
//    {
//        log_d("port %x is disconnected", port);
//        deregisterPortDevice(port);
//    }
//}

/**
 * @brief Parse advertising name of hub
 * @param [in] pData The pointer to the received data
 * @return advertising name
 */
std::string SBrick::parseHubAdvertisingName(uint8_t *pData)
{
    int charArrayLength = min(pData[0] - 5, 14);
    char name[charArrayLength + 1];
    for (int i = 0; i < charArrayLength; i++)
    {
        name[i] = pData[5 + i];
    }
    name[charArrayLength + 1] = 0;
    log_d("advertising name: %s", name);
    return std::string(name);
}

/**
 * @brief Parse sw or hw version
 * @param [in] pData The pointer to the received data
 * @return version structure (Major-Minor-Bugfix, Build)
 */
Version SBrick::parseVersion(uint8_t *pData)
{
    Version version;

    /*
    version.Build = LegoinoCommon::ReadUInt16LE(pData, 5);
    version.Major = LegoinoCommon::ReadUInt8(pData, 8) >> 4;
    version.Minor = LegoinoCommon::ReadUInt8(pData, 8) & 0xf;
    version.Bugfix = LegoinoCommon::ReadUInt8(pData, 7);
    */
    
    return version;
}

/**
 * @brief Parse RSSI value
 * @param [in] pData The pointer to the received data
 * @return RSSI value in dB
 */
int SBrick::parseRssi(uint8_t *pData)
{
    //int rssi = LegoinoCommon::ReadInt8(pData, 5);
    //log_d("rssi: %d", rssi);
    //return rssi;

    return -1;
}

/**
 * @brief Parse battery level of hub
 * @param [in] pData The pointer to the received data
 * @return battery level in [%]
 */
//uint8_t SBrick::parseBatteryLevel(uint8_t *pData)
//{
//    uint8_t batteryLevel = LegoinoCommon::ReadUInt8(pData, 5);
//    log_d("battery level: %d", batteryLevel);
//    return batteryLevel;
//}

/**
 * @brief Parse battery type of hub
 * @param [in] pData The pointer to the received data
 * @return battery type (0=normal, 1=recharchable)
 */
//byte SBrick::parseBatteryType(uint8_t *pData)
//{
//    byte batteryType = pData[5];
//    log_d("battery type: %x", batteryType);
//    return batteryType;
//}

/**
 * @brief Parse system type id of hub
 * @param [in] pData The pointer to the received data
 * @return system type id
 */
//uint8_t SBrick::parseSystemTypeId(uint8_t *pData)
//{
//    uint8_t systemTypeId = LegoinoCommon::ReadUInt8(pData, 5);
//    return systemTypeId;
//}

/**
 * @brief Get the update mode dependent on the device type
 * @param [in] pData The pointer to the received data
 * @return Update mode
 */
//byte SBrick::getModeForDeviceType(byte deviceType)
//{
//    switch (deviceType)
//    {
//    case (byte)DeviceType::SIMPLE_MEDIUM_LINEAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::TRAIN_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::MEDIUM_LINEAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::MOVE_HUB_MEDIUM_LINEAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::COLOR_DISTANCE_SENSOR:
//        return 0x08;
//    case (byte)DeviceType::MOVE_HUB_TILT_SENSOR:
//        return 0x00;
//    case (byte)DeviceType::TECHNIC_MEDIUM_ANGULAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::TECHNIC_LARGE_ANGULAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::TECHNIC_LARGE_LINEAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::TECHNIC_XLARGE_LINEAR_MOTOR:
//        return (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM;
//    case (byte)DeviceType::MARIO_HUB_GESTURE_SENSOR:
//        return 0x01;
//    default:
//        return 0x00;
//    }
//}

/**
 * @brief Parse the incoming characteristic notification for a Port Action Message
 * @param [in] pData The pointer to the received data
 */
//void SBrick::parsePortAction(uint8_t *pData)
//{
//    log_d("parsePortAction");
//}

/**
 * @brief Callback function for notifications of a specific characteristic
 * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
 * @param [in] pData The pointer to the received data
 * @param [in] length The length of the data array
 * @param [in] isNotify 
 */
//void SBrick::notifyCallback(
//    NimBLERemoteCharacteristic *pBLERemoteCharacteristic,
//    uint8_t *pData,
//    size_t length,
//    bool isNotify)
//{
//    log_d("notify callback for characteristic %s", pBLERemoteCharacteristic->getUUID().toString().c_str());
//
//    switch (pData[2])
//    {
//    case (byte)MessageType::HUB_PROPERTIES:
//    {
//        parseDeviceInfo(pData);
//        break;
//    }
//    case (byte)MessageType::HUB_ATTACHED_IO:
//    {
//        parsePortMessage(pData);
//        break;
//    }
//    case (byte)MessageType::PORT_VALUE_SINGLE:
//    {
//        parseSensorMessage(pData);
//        break;
//    }
//    case (byte)MessageType::PORT_OUTPUT_COMMAND_FEEDBACK:
//    {
//        parsePortAction(pData);
//        break;
//    }
//    }
//}

/**
 * @brief Constructor
 */
SBrick::SBrick(){};

/**
 * @brief Init function set the UUIDs and scan for the Hub
 */
void SBrick::init()
{
    _isConnected = false;
    _isConnecting = false;
    _bleUuid = BLEUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
    _charachteristicUuid = BLEUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);
    //_hubType = HubType::UNKNOWNHUB;

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();

    pBLEScan->setAdvertisedDeviceCallbacks(new SBrickAdvertisedDeviceCallbacks(this));

    pBLEScan->setActiveScan(true);

    // Start method with callback function to enforce the non blocking scan.
    // If no callback function is used, the scan starts in a blocking manner.
    pBLEScan->start(_scanDuration, scanEndedCallback);
}

/**
 * @brief Init function set the UUIDs and scan for the Hub
 * @param [in] deviceAddress to which the arduino should connect represented by a hex string of the format: 00:00:00:00:00:00
 */
void SBrick::init(std::string deviceAddress)
{
    _requestedDeviceAddress = new BLEAddress(deviceAddress);
    init();
}

/**
 * @brief Init function set the BLE scan duration (default value 5s)
 * @param [in] BLE scan durtation in unit seconds
 */
void SBrick::init(uint32_t scanDuration)
{
    _scanDuration = scanDuration;
    init();
}

/**
 * @brief Init function set the BLE scan duration (default value 5s)
 * @param [in] deviceAddress to which the arduino should connect represented by a hex string of the format: 00:00:00:00:00:00
 * @param [in] BLE scan durtation in unit seconds
 */
void SBrick::init(std::string deviceAddress, uint32_t scanDuration)
{
    _requestedDeviceAddress = new BLEAddress(deviceAddress);
    _scanDuration = scanDuration;
    init();
}

/**
 * @brief Get the address of the HUB (server address)
 * @return HUB Address
 */
NimBLEAddress SBrick::getHubAddress()
{
    NimBLEAddress pAddress = *_pServerAddress;
    return pAddress;
}

/**
 * @brief Get the array index of a specific connected device on a defined port in the connectedDevices array
 * @param [in] port number
 * @return array index of the connected device
 */
//int SBrick::getDeviceIndexForPortNumber(byte portNumber)
//{
//    log_d("Number of connected devices: %d", numberOfConnectedDevices);
//    for (int idx = 0; idx < numberOfConnectedDevices; idx++)
//    {
//        log_v("device %d, port number: %x, device type: %x, callback address: %x", idx, connectedDevices[idx].PortNumber, connectedDevices[idx].DeviceType, connectedDevices[idx].Callback);
//        if (connectedDevices[idx].PortNumber == portNumber)
//        {
//            log_d("device on port %x has index %d", portNumber, idx);
//            return idx;
//        }
//    }
//    log_w("no device found for port number %x", portNumber);
//    return -1;
//}

/**
 * @brief Get the device type of a specific connected device on a defined port in the connectedDevices array
 * @param [in] port number
 * @return device type of the connected device
 */
//byte SBrick::getDeviceTypeForPortNumber(byte portNumber)
//{
//    log_d("Number of connected devices: %d", numberOfConnectedDevices);
//    for (int idx = 0; idx < numberOfConnectedDevices; idx++)
//    {
//        log_v("device %d, port number: %x, device type: %x, callback address: %x", idx, connectedDevices[idx].PortNumber, connectedDevices[idx].DeviceType, connectedDevices[idx].Callback);
//        if (connectedDevices[idx].PortNumber == portNumber)
//        {
//            log_d("device on port %x has type %x", portNumber, connectedDevices[idx].DeviceType);
//            return connectedDevices[idx].DeviceType;
//        }
//    }
//    log_w("no device found for port number %x", portNumber);
//    return (byte)DeviceType::UNKNOWNDEVICE;
//}

/**
 * @brief Get the port where a specific device is connected
 * @param [in] device type
 * @return port number if device type is found or 255
 */
//byte SBrick::getPortForDeviceType(byte deviceType)
//{
//    log_d("Number of connected devices: %d", numberOfConnectedDevices);
//    for (int idx = 0; idx < numberOfConnectedDevices; idx++)
//    {
//        log_v("device %d, port number: %x, device type: %x, callback address: %x", idx, connectedDevices[idx].PortNumber, connectedDevices[idx].DeviceType, connectedDevices[idx].Callback);
//        if (connectedDevices[idx].DeviceType == deviceType)
//        {
//            log_d("port %x has device of type %x", connectedDevices[idx].PortNumber, deviceType);
//            return connectedDevices[idx].PortNumber;
//        }
//    }
//    log_w("no port found with device type %x", deviceType);
//    return 255;
//}

/**
 * @brief Set the color of the HUB LED with predefined colors
 * @param [in] color one of the available hub colors
 */
//void SBrick::setLedColor(Color color)
//{
//    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
//    byte setColorMode[8] = {0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
//    WriteValue(setColorMode, 8);
//    byte setColor[6] = {0x81, port, 0x11, 0x51, 0x00, color};
//    WriteValue(setColor, 6);
//}

/**
 * @brief Set the color of the HUB LED with RGB values 
 * @param [in] red 0..255 
 * @param [in] green 0..255 
 * @param [in] blue 0..255 
 */
//void SBrick::setLedRGBColor(char red, char green, char blue)
//{
//    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
//    byte setRGBMode[8] = {0x41, port, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
//    WriteValue(setRGBMode, 8);
//    byte setRGBColor[8] = {0x81, port, 0x11, 0x51, 0x01, red, green, blue};
//    WriteValue(setRGBColor, 8);
//}

/**
 * @brief Set the color of the HUB LED with HSV values 
 * @param [in] hue 0..360 
 * @param [in] saturation 0..1 
 * @param [in] value 0..1
 */
//void SBrick::setLedHSVColor(int hue, double saturation, double value)
//{
//    hue = hue % 360; // map hue to 0..360
//    double huePart = hue / 60.0;
//    double fract = huePart - floor(huePart);
//
//    double p = value * (1. - saturation);
//    double q = value * (1. - saturation * fract);
//    double t = value * (1. - saturation * (1. - fract));
//
//    if (huePart >= 0.0 && huePart < 1.0)
//    {
//        setLedRGBColor((char)(value * 255), (char)(t * 255), (char)(p * 255));
//    }
//    else if (huePart >= 1.0 && huePart < 2.0)
//    {
//        setLedRGBColor((char)(q * 255), (char)(value * 255), (char)(p * 255));
//    }
//    else if (huePart >= 2.0 && huePart < 3.0)
//    {
//        setLedRGBColor((char)(p * 255), (char)(value * 255), (char)(t * 255));
//    }
//    else if (huePart >= 3.0 && huePart < 4.0)
//    {
//        setLedRGBColor((char)(p * 255), (char)(q * 255), (char)(value * 255));
//    }
//    else if (huePart >= 4.0 && huePart < 5.0)
//    {
//        setLedRGBColor((char)(t * 255), (char)(p * 255), (char)(value * 255));
//    }
//    else if (huePart >= 5.0 && huePart < 6.0)
//    {
//        setLedRGBColor((char)(value * 255), (char)(p * 255), (char)(q * 255));
//    }
//    else
//    {
//        setLedRGBColor(0, 0, 0);
//    }
//}

/**
 * @brief Set name of the HUB
 * @param [in] name character array which contains the name (max 14 characters are supported)
 */
void SBrick::setHubName(char name[])
{
    int nameLength = strlen(name);
    if (nameLength > 14)
    {
        return;
    }
    _hubName = std::string(name, nameLength);

    char offset = 3;
    int arraySize = offset + nameLength;
    byte setNameCommand[arraySize] = {0x01, 0x01, 0x01};

    memcpy(setNameCommand + offset, name, nameLength);
    //WriteValue(setNameCommand, arraySize);
}

/**
 * @brief Activate the update/notification of hub specific property changes (battery level, rssi, ...)
 * @param [in] hubProperty for which updates should be activated
 * @param [in] optional callback function which will be called if a value has changed
 */
//void SBrick::activateHubPropertyUpdate(HubPropertyReference hubProperty, HubPropertyChangeCallback hubPropertyChangeCallback)
//{
//    if (hubPropertyChangeCallback != nullptr)
//    {
//        _hubPropertyChangeCallback = hubPropertyChangeCallback;
//    }
//
//    // Activate reports
//    byte notifyPropertyCommand[3] = {0x01, (byte)hubProperty, (byte)HubPropertyOperation::ENABLE_UPDATES_DOWNSTREAM};
//    WriteValue(notifyPropertyCommand, 3);
//}

/**
 * @brief Request a hub specific property value (battery level, rssi, ...)
 * @param [in] hubProperty for which the value should be requested
 * @param [in] optional callback function which will be called if a value has changed
 */
//void SBrick::requestHubPropertyUpdate(HubPropertyReference hubProperty, HubPropertyChangeCallback hubPropertyChangeCallback)
//{
//    if (hubPropertyChangeCallback != nullptr)
//    {
//        _hubPropertyChangeCallback = hubPropertyChangeCallback;
//    }
//
//    // Activate reports
//    byte notifyPropertyCommand[3] = {0x01, (byte)hubProperty, (byte)HubPropertyOperation::REQUEST_UPDATE_DOWNSTREAM};
//    WriteValue(notifyPropertyCommand, 3);
//}

/**
 * @brief Deactivate the update/notification of hub specific property changes (battery level, rssi, ...)
 * @param [in] hubProperty for which updates should be activated
 */
//void SBrick::deactivateHubPropertyUpdate(HubPropertyReference hubProperty)
//{
//
//    // Activate reports
//    byte notifyPropertyCommand[3] = {0x01, (byte)hubProperty, (byte)HubPropertyOperation::DISABLE_UPDATES_DOWNSTREAM};
//    WriteValue(notifyPropertyCommand, 3);
//}

/**
 * @brief Connect to the HUB, get a reference to the characteristic and register for notifications
 */
bool SBrick::connectHub()
{
    BLEAddress pAddress = *_pServerAddress;
    NimBLEClient *pClient = nullptr;

    log_d("number of ble clients: %d", NimBLEDevice::getClientListSize());

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize())
    {
        /** Special case when we already know this device, we send false as the 
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(pAddress);
        if (pClient)
        {
            if (!pClient->connect(pAddress, false))
            {
                log_e("reconnect failed");
                return false;
            }
            log_d("reconnect client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else
        {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
            log_w("max clients reached - no more connections available: %d", NimBLEDevice::getClientListSize());
            return false;
        }

        pClient = NimBLEDevice::createClient();
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(pAddress))
        {
            log_e("failed to connect");
            return false;
        }
    }

    log_d("connected to: %s, RSSI: %d", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());
    BLERemoteService *pRemoteService = pClient->getService(_bleUuid);
    if (pRemoteService == nullptr)
    {
        log_e("failed to get ble client");
        return false;
    }

    _pRemoteCharacteristic = pRemoteService->getCharacteristic(_charachteristicUuid);
    if (_pRemoteCharacteristic == nullptr)
    {
        log_e("failed to get ble service");
        return false;
    }

    // register notifications (callback function) for the characteristic
    if (_pRemoteCharacteristic->canNotify())
    {
        //_pRemoteCharacteristic->subscribe(true, std::bind(&SBrick::notifyCallback, this, _1, _2, _3, _4), true);
    }

    // add callback instance to get notified if a disconnect event appears
    pClient->setClientCallbacks(new SBrickClientCallback(this));

    // Set states
    _isConnected = true;
    _isConnecting = false;
    return true;
}

/**
 * @brief Retrieve the connection state. The BLE client (ESP32) has found a service with the desired UUID (HUB)
 * If this state is available, you can try to connect to the Hub
 */
bool SBrick::isConnecting()
{
    return _isConnecting;
}

/**
 * @brief Retrieve the connection state. The BLE client (ESP32) is connected to the server (HUB)
 */
bool SBrick::isConnected()
{
    return _isConnected;
}

/**
 * @brief Retrieve the hub type
 * @return hub type 
 */
//HubType SBrick::getHubType()
//{
//    return _hubType;
//}

/**
 * @brief Retrieve the hub name
 * @return hub name 
 */
std::string SBrick::getHubName()
{
    return _hubName;
}

/**
 * @brief Set the motor speed on a defined port. 
 * @param [in] port Port of the Hub on which the speed of the motor will set (A, B)
 * @param [in] speed Speed of the Motor -100..0..100 negative values will reverse the rotation
 */
//void SBrick::setBasicMotorSpeed(byte port, int speed = 0)
//{
//    byte setMotorCommand[8] = {0x81, port, 0x11, 0x51, 0x00, LegoinoCommon::MapSpeed(speed)}; //train, batmobil
//    WriteValue(setMotorCommand, 6);
//}

/**
 * @brief Stop the motor on a defined port.
 * @param [in] port Port of the Hub on which the motor will be stopped (A, B)
 */
//void SBrick::stopBasicMotor(byte port)
//{
//    setBasicMotorSpeed(port, 0);
//}

/**
 * @brief Set the acceleration profile 
 * @param [in] port Port of the Hub on which the speed of the motor will set (A, B, AB)
 * @param [in] time Time value in ms of the acceleration from 0-100% speed/Power
 */
//void SBrick::setAccelerationProfile(byte port, int16_t time)
//{
//    byte *timeBytes = LegoinoCommon::Int16ToByteArray(time);
//    byte setMotorCommand[7] = {0x81, port, 0x10, 0x05, timeBytes[0], timeBytes[1], 0x01};
//    WriteValue(setMotorCommand, 7);
//}

/**
 * @brief Set the deceleration profile 
 * @param [in] port Port of the Hub on which the speed of the motor will set (A, B, AB)
 * @param [in] time Time value in ms of the deceleration from 100-0% speed/Power
 */
//void SBrick::setDecelerationProfile(byte port, int16_t time)
//{
//    byte *timeBytes = LegoinoCommon::Int16ToByteArray(time);
//    byte setMotorCommand[7] = {0x81, port, 0x10, 0x06, timeBytes[0], timeBytes[1], 0x02};
//    WriteValue(setMotorCommand, 7);
//}

//#endif // ESP32
