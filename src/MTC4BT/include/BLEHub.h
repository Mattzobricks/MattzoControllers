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

enum BLEHubParam {
  ledColour = 0,
  locomotive = 1,
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
    
    /**
     * @brief setHubParameter, the paramname should do the cast, because it knows
     * @param [in] paramname
     * @param [in] value
     */
    virtual void setHubParameter(BLEHubParam paramname, void * value) = 0;


  protected:
    void initChannelControllers();
    void dumpPData(uint8_t *pData, size_t length);
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
    HubLedColor hubColour;
    // The following classes can access private members of BLEHub.
    friend class BLEClientCallback;
    friend class BLEDeviceCallbacks;
};