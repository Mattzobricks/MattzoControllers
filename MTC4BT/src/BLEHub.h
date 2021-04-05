#pragma once

#include <Arduino.h>
#include "NimBLEDevice.h"

#include "ChannelConfiguration.h"
#include "ChannelController.h"

#define AUTO_LIGHTS_ENABLED true
#define AUTO_LIGHTS_DISABLED false

// The priority at which the task should run.
// Systems that include MPU support can optionally create tasks in a privileged (system) mode by setting bit portPRIVILEGE_BIT of the priority parameter.
// For example, to create a privileged task at priority 2 the uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
#define BLE_TaskPriority 1

// If the value is tskNO_AFFINITY, the created task is not pinned to any CPU, and the scheduler can run it on any core available.
// Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
// Specifying values larger than (portNUM_PROCESSORS - 1) will cause the function to fail.
#define BLE_CoreID 1

// The size of the task stack specified as the number of bytes.
#define BLE_StackDepth 2048

// The number of seconds to wait for a Hub to connect.
#define ConnectDelayInSeconds 5

class BLEHub
{
public:
    BLEHub(std::string deviceName, std::string deviceAddress, std::vector<ChannelConfiguration> channels[], int16_t lightPerc = 100, bool autoLightsEnabled = false, bool enabled = true);

    // Returns a boolean value indicating whether this BLE hub is enabled (in use).
    bool IsEnabled();

    // Starts device discovery (if not already discovering).
    void StartDiscovery(NimBLEScan *scanner, const uint32_t scanDurationInSeconds = 3);

    // Returns a boolean value indicating whether we have discovered the BLE hub.
    bool IsDiscovered();

    // Returns a boolean value indicating whether we are connected to the BLE hub.
    bool IsConnected();

    BaseType_t StartDriveTask();

    // Sets the given target speed percentages for the respective channels (by their index).
    // The number of speed percentages specified should match the actual number of BLE hub channels.
    // void Drive(const int16_t channelSpeedPercs[]);

    // Sets the given target speed percentage for all motor channels.
    void Drive(const int16_t speedPerc);

    // Turns all light channels on/off.
    void SetLights(bool on);

    // Turns specific light channel on/off.
    void SetLights(HubChannel channel, bool on);

    // If true, immediately sets the current speed for all channels to zero.
    // If false, releases the emergency break.
    void EmergencyBreak(const bool enabled);

    // Returns the device name.
    std::string GetDeviceName();

    // Returns a boolean value indicating whether the lights should automatically turn on when the train starts driving.
    bool GetAutoLightsEnabled();

    // Abstract method used to connect to the BLE hub.
    virtual bool Connect(const uint8_t watchdogTimeOutInTensOfSeconds) = 0;

    // Abstract method used to periodically send drive commands to the BLE hub.
    virtual void DriveTaskLoop() = 0;

    // Abstract method used to map a speed percentile (-100% - 100%) to a raw speed value.
    virtual int16_t MapSpeedPercToRaw(int speedPerc) = 0;

private:
    void initChannelControllers(std::vector<ChannelConfiguration> channels[]);
    void setTargetSpeedPercByAttachedDevice(AttachedDevice device, int16_t speedPerc);
    void setTargetSpeedPercForChannelByAttachedDevice(HubChannel channel, AttachedDevice device, int16_t speedPerc);
    ChannelController *findControllerByChannel(HubChannel channel);
    bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID);

    static void driveTaskImpl(void *);

    std::string _deviceName;
    NimBLEAddress *_address;
    int16_t _lightPerc;
    bool _autoLightsEnabled;
    bool _isEnabled;

    std::vector<ChannelController *> _channelControllers;

    TaskHandle_t _driveTaskHandle;
    NimBLEAdvertisedDevice *_advertisedDevice;
    NimBLEAdvertisedDeviceCallbacks *_advertisedDeviceCallback;
    NimBLEClient *_hub;
    NimBLEClientCallbacks *_clientCallback;
    bool _ebreak;
    bool _isDiscovering;
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
    friend class AdvertisedBLEDeviceCallbacks;
};