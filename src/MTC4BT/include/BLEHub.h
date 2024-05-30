#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "BLEHubChannel.h"
#include "BLEHubChannelController.h"
#include "BLEHubConfiguration.h"
#include "MCLocoAction.h"

// The priority at which the task should run.
// Systems that include MPU support can optionally create tasks in a privileged (system) mode by setting bit portPRIVILEGE_BIT of the priority parameter.
// For example, to create a privileged task at priority 2 the uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
#define BLE_TaskPriority 1

// If the value is tskNO_AFFINITY, the created task is not pinned to any CPU, and the scheduler can run it on any core available.
// Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
// Specifying values larger than (portNUM_PROCESSORS - 1) will cause the function to fail.
#define BLE_CoreID CONFIG_BT_NIMBLE_PINNED_TO_CORE

// The size of the task stack specified as the number of bytes.
#define BLE_StackDepth 2048

// The number of seconds to wait for a Hub to connect.
#define ConnectDelayInSeconds 5

enum struct MessageType {
    HUB_PROPERTIES = 0x01,
    HUB_ACTIONS = 0x02,
    HUB_ALERTS = 0x03,
    HUB_ATTACHED_IO = 0x04,
    GENERIC_ERROR_MESSAGES = 0x05,
    HW_NETWORK_COMMANDS = 0x08,
    FW_UPDATE_GO_INTO_BOOT_MODE = 0x10,
    FW_UPDATE_LOCK_MEMORY = 0x11,
    FW_UPDATE_LOCK_STATUS_REQUEST = 0x12,
    FW_LOCK_STATUS = 0x13,
    PORT_INFORMATION_REQUEST = 0x21,
    PORT_MODE_INFORMATION_REQUEST = 0x22,
    PORT_INPUT_FORMAT_SETUP_SINGLE = 0x41,
    PORT_INPUT_FORMAT_SETUP_COMBINEDMODE = 0x42,
    PORT_INFORMATION = 0x43,
    PORT_MODE_INFORMATION = 0x44,
    PORT_VALUE_SINGLE = 0x45,
    PORT_VALUE_COMBINEDMODE = 0x46,
    PORT_INPUT_FORMAT_SINGLE = 0x47,
    PORT_INPUT_FORMAT_COMBINEDMODE = 0x48,
    VIRTUAL_PORT_SETUP = 0x61,
    PORT_OUTPUT_COMMAND = 0x81,
    PORT_OUTPUT_COMMAND_FEEDBACK = 0x82,
};

// Abstract Bluetooth Low Energy (BLE) hub base class.
class BLEHub
{
  public:
    BLEHub(BLEHubConfiguration *config);

    // Returns a boolean value indicating whether we have discovered the BLE hub.
    bool IsDiscovered();

    // Returns a boolean value indicating whether we are connected to the BLE hub.
    bool IsConnected();

    // Sets
    void SetConnectCallback(std::function<void(bool)> callback);

    // Returns the hub's raw address.
    std::string GetRawAddress();

    // Returns the hub's address.
    NimBLEAddress GetAddress();

    // Sets the given target power perc for all motor channels.
    void Drive(const int16_t minPwrPerc, const int16_t pwrPerc);

    // Gets the current power perc for any motor channel.
    int16_t GetCurrentDrivePwrPerc();

    // Executes the given action.
    void Execute(MCLocoAction *action);

    // Makes all channels with lights attached blink for the given duration.
    void BlinkLights(int durationInMs);

    // Set the initial color of the Hub's onboard LED.
    void SetHubLedColor(HubLedColor color);

    // If true, immediately sets the current speed for all channels to zero.
    // If false, releases the manual brake.
    void SetManualBrake(const bool enabled);

    // If true, immediately sets the current speed for all channels to zero.
    // If false, releases the emergency brake.
    void SetEmergencyBrake(const bool enabled);

    // Method used to connect to the BLE hub.
    bool Connect(const uint8_t watchdogTimeOutInTensOfSeconds);

    // Abstract method used to set the watchdog timeout.
    virtual bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds) = 0;

    // Abstract method used to periodically send drive commands to the BLE hub.
    virtual void DriveTaskLoop() = 0;

    // Abstract method used to map a speed percentile (-100% - 100%) to a raw speed value.
    virtual int16_t MapPwrPercToRaw(int pwrPerc) = 0;

    // Abstract callback method used to handle hub notifications.
    virtual void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) = 0;

  private:
    void initChannelControllers();
    void setTargetPwrPercByAttachedDevice(DeviceType device, int16_t minPwrPerc, int16_t pwrPerc);
    HubLedColor getRawLedColorForController(BLEHubChannelController *controller);
    uint8_t getRawChannelPwrForController(BLEHubChannelController *controller);
    BLEHubChannelController *findControllerByChannel(BLEHubChannel channel);

    bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID);
    bool startDriveTask();
    static void driveTaskImpl(void *);
    void connected();
    void disconnected();

    std::function<void(bool)> _onConnectionChangedCallback;

    BLEHubConfiguration *_config;
    std::vector<BLEHubChannelController *> _channelControllers;

    TaskHandle_t _driveTaskHandle;
    NimBLEAdvertisedDevice *_advertisedDevice;
    NimBLEAdvertisedDeviceCallbacks *_advertisedDeviceCallback;
    NimBLEClient *_hub;
    NimBLEClientCallbacks *_clientCallback;
    bool _mbrake;
    bool _ebrake;
    bool _blinkLights;
    ulong _blinkUntil;
    bool _isDiscovered;
    bool _isConnected;
    uint16_t _watchdogTimeOutInTensOfSeconds;
    NimBLERemoteService *_remoteControlService;
    NimBLERemoteCharacteristic *_remoteControlCharacteristic;
    // NimBLERemoteCharacteristic *_genericAccessCharacteristic;
    // NimBLERemoteCharacteristic *_deviceInformationCharacteristic;

    // The following classes can access private members of BLEHub.
    friend class PUHub;
    friend class SBrickHub;
    friend class BLEClientCallback;
    friend class BLEDeviceCallbacks;
};