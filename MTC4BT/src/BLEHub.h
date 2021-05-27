#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "BLEHubChannel.h"
#include "BLEHubChannelController.h"
#include "BLEHubConfiguration.h"
#include "Fn.h"

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

// Abstract Bluetooth Low Energy (BLE) hub base class.
class BLEHub
{
public:
    BLEHub(BLEHubConfiguration *config, int16_t speedStep, int16_t brakeStep);

    // Returns a boolean value indicating whether this BLE hub is enabled (in use).
    bool IsEnabled();

    // Returns a boolean value indicating whether we have discovered the BLE hub.
    bool IsDiscovered();

    // Returns a boolean value indicating whether we are connected to the BLE hub.
    bool IsConnected();

    // Returns the hub's address.
    std::string GetAddress();

    // Sets the given target speed for the respective channels (by their index).
    // The number of speeds specified should match the actual number of BLE hub channels.
    // void Drive(const int16_t channelSpeedPercs[]);

    // Sets the given target speed for all motor channels.
    void Drive(const int16_t minSpeed, const int16_t speed);

    // Handles the given function.
    void HandleFn(Fn* fn, bool on);

    // Makes all channels with lights attached blink for the given duration.
    void BlinkLights(int durationInMs);

    // If true, immediately sets the current speed for all channels to zero.
    // If false, releases the emergency brake.
    void EmergencyBrake(const bool enabled);

    // Returns a boolean value indicating whether the lights should automatically turn on when the train starts driving.
    bool GetAutoLightsEnabled();

    // Method used to connect to the BLE hub.
    bool Connect(const uint8_t watchdogTimeOutInTensOfSeconds);

    // Abstract method used to set the watchdog timeout.
    virtual bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds) = 0;

    // Abstract method used to periodically send drive commands to the BLE hub.
    virtual void DriveTaskLoop() = 0;

    // Abstract method used to map a speed percentile (-100% - 100%) to a raw speed value.
    virtual int16_t MapSpeedPercToRaw(int speedPerc) = 0;

private:
    void initChannelControllers();
    void setTargetSpeedPercByAttachedDevice(DeviceType device, int16_t minSpeedPerc, int16_t speedPerc);
    void setTargetSpeedPercForChannelByAttachedDevice(BLEHubChannel channel, DeviceType device, int16_t minSpeedPerc, int16_t speedPerc);
    uint8_t getRawChannelSpeedForController(BLEHubChannelController *controller);
    BLEHubChannelController *findControllerByChannel(BLEHubChannel channel);
    bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID);
    BaseType_t startDriveTask();
    static void driveTaskImpl(void *);

    BLEHubConfiguration *_config;
    int16_t _speedStep;
    int16_t _brakeStep;
    std::vector<BLEHubChannelController *> _channelControllers;

    TaskHandle_t _driveTaskHandle;
    NimBLEAdvertisedDevice *_advertisedDevice;
    NimBLEAdvertisedDeviceCallbacks *_advertisedDeviceCallback;
    NimBLEClient *_hub;
    NimBLEClientCallbacks *_clientCallback;
    bool _ebrake;
    bool _blinkLights;
    ulong _blinkUntil;
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
    friend class BLEDeviceCallbacks;
};