/*
 * SBrick.h - Arduino base class for controlling SBrick controllers
 * 
 * (c) Copyright 2021 - Raymond Brink
 * Released under MIT License
 * 
*/

//#if defined(ESP32)

#ifndef SBrick_h
#define SBrick_h

#include "Arduino.h"
#include "NimBLEDevice.h"
#include "SBrickConst.h"

using namespace std::placeholders;

//typedef void (*HubPropertyChangeCallback)(void *hub, HubPropertyReference hubProperty, uint8_t *pData);
//typedef void (*PortValueChangeCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

/*
struct Device
{
  byte PortNumber;
  byte DeviceType;
  PortValueChangeCallback Callback;
};
*/

class SBrick
{

public:
  // constructor
  SBrick();

  // initializer methods
  void init();
  void init(uint32_t scanDuration);
  void init(std::string deviceAddress);
  void init(std::string deviceAddress, uint32_t scanDuration);

  // hub related methods
  bool connectHub();
  bool isConnected();
  bool isConnecting();
  NimBLEAddress getHubAddress();
  //HubType getHubType();
  std::string getHubName();
  void setHubName(char name[]);
  BLEAddress* _requestedDeviceAddress = nullptr;
  BLEAddress* _pServerAddress;
  std::string _hubName;
  SBrickProduct _product;

  boolean _isConnecting;
  boolean _isConnected;
  
  //void activateHubPropertyUpdate(HubPropertyReference hubProperty, HubPropertyChangeCallback hubPropertyChangeCallback = nullptr);
  //void deactivateHubPropertyUpdate(HubPropertyReference hubProperty);
  //void requestHubPropertyUpdate(HubPropertyReference hubProperty, HubPropertyChangeCallback hubPropertyChangeCallback = nullptr);

  // port and device related methods
  //int getDeviceIndexForPortNumber(byte portNumber);
  //byte getDeviceTypeForPortNumber(byte portNumber);
  //byte getPortForDeviceType(byte deviceType);
  //byte getModeForDeviceType(byte deviceType);
  //void registerPortDevice(byte portNumber, byte deviceType);
  //void deregisterPortDevice(byte portNumber);
  //void activatePortDevice(byte portNumber, byte deviceType, PortValueChangeCallback portValueChangeCallback = nullptr);
  //void activatePortDevice(byte portNumber, PortValueChangeCallback portValueChangeCallback = nullptr);
  //void deactivatePortDevice(byte portNumber, byte deviceType);
  //void deactivatePortDevice(byte portNumber);

  // write (set) operations on port devices
  //void WriteValue(byte command[], int size);

  //void setLedColor(Color color);
  //void setLedRGBColor(char red, char green, char blue);
  //void setLedHSVColor(int hue, double saturation, double value);

  //void stopBasicMotor(byte port);
  //void setBasicMotorSpeed(byte port, int speed);

  //void setAccelerationProfile(byte port, int16_t time);
  //void setDecelerationProfile(byte port, int16_t time);

private:
  // parse methods to read in the message content of the charachteristic value
  void parseDeviceInfo(uint8_t* pData);
  //void parsePortMessage(uint8_t* pData);
  //void parsePortAction(uint8_t* pData);
  //uint8_t parseSystemTypeId(uint8_t* pData);
  //byte parseBatteryType(uint8_t* pData);
  //uint8_t parseBatteryLevel(uint8_t* pData);
  int parseRssi(uint8_t* pData);
  Version parseVersion(uint8_t* pData);
  std::string parseHubAdvertisingName(uint8_t* pData);

  // BLE specific stuff
  //void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
  BLEUUID _bleUuid;
  BLEUUID _charachteristicUuid;
  BLERemoteCharacteristic* _pRemoteCharacteristic;

  // Notification callbacks
  //HubPropertyChangeCallback _hubPropertyChangeCallback = nullptr;

  // List of connected devices
  //Device connectedDevices[13];
  int numberOfConnectedDevices = 0;

  //BLE settings
  uint32_t _scanDuration = 10;
};

#endif // SBrick_h

//#endif // ESP32
