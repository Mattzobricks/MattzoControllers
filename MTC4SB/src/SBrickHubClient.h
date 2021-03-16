#pragma once

#include <Arduino.h>
#include "NimBLEDevice.h"

#include "SBrickHubChannel.h"

class SBrickHubClient
{
public:
  // Public static members

  /// <summary>
  /// The priority at which the task should run.
  /// Systems that include MPU support can optionally create tasks in a privileged (system) mode by setting bit portPRIVILEGE_BIT of the priority parameter.
  /// For example, to create a privileged task at priority 2 the uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
  /// </summary>
  static uint8_t TaskPriority;

  /// <summary>
  /// If the value is tskNO_AFFINITY, the created task is not pinned to any CPU, and the scheduler can run it on any core available.
  /// Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
  /// Specifying values larger than (portNUM_PROCESSORS - 1) will cause the function to fail.
  /// </summary>
  static int8_t CoreID;

  /// <summary>
  /// The size of the task stack specified as the number of bytes.
  /// </summary>
  static uint32_t StackDepth;

  /// <summary>
  /// The number of seconds to wait for an SBrick to connect.
  /// </summary>
  static uint8_t ConnectDelayInSeconds;

  SBrickHubClient(std::string deviceName, std::string deviceAddress, bool autoLightsOnEnabled = false, uint8_t speedStep = 10, uint8_t brakeStep = 20, bool enabled = true);
  bool IsEnabled();
  void StartDiscovery(NimBLEScan *scanner, const uint32_t scanDurationInSeconds = 3);
  bool Connect(const uint32_t watchdogTimeOutInTensOfSeconds);
  bool IsDiscovered();
  bool IsConnected();
  void Drive(const int16_t a, const int16_t b, const int16_t c, const int16_t d);
  void DriveChannel(const SBrickHubChannel::SBrickChannel channel, const int16_t speed);
  void EmergencyBreak(const bool enabled);
  std::string getDeviceName();
  bool getAutoLightsEnabled();

private:
  bool connectToServer(const uint16_t watchdogTimeOutInMs);
  bool setWatchdogTimeout(const uint16_t watchdogTimeOutInMs);
  // static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
  bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID);

  BaseType_t startDriveTask();
  static void driveTaskImpl(void*);
  void driveTaskLoop();

  std::string _deviceName;
  NimBLEAddress* _address;
  bool _autoLightsEnabled;
  uint8_t _speedStep;
  uint8_t _brakeStep;
  bool _isEnabled;

  TaskHandle_t _driveTaskHandle;
  NimBLEClient* _sbrick;
  NimBLEAdvertisedDeviceCallbacks* _advertisedDeviceCallback;
  NimBLEClientCallbacks* _clientCallback;
  bool _ebreak;
  bool _isDiscovering;
  bool _isDiscovered;
  bool _isConnected;
  uint16_t _watchdogTimeOutInTensOfSeconds;
  NimBLERemoteService* _remoteControlService;
  NimBLERemoteCharacteristic* _remoteControlCharacteristic;
  NimBLERemoteCharacteristic* _genericAccessCharacteristic;
  NimBLERemoteCharacteristic* _deviceInformationCharacteristic;

  SBrickHubChannel *_channels[4] = {
    new SBrickHubChannel(SBrickHubChannel::SBrickChannel::A),
    new SBrickHubChannel(SBrickHubChannel::SBrickChannel::B),
    new SBrickHubChannel(SBrickHubChannel::SBrickChannel::C),
    new SBrickHubChannel(SBrickHubChannel::SBrickChannel::D)
  };

  // SBrickClientCallback and SBrickAdvertisedDeviceCallbacks can access private members of SBrickHubClient.
  friend class SBrickClientCallback;
  friend class SBrickAdvertisedDeviceCallbacks;
};