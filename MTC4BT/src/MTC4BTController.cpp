#include "MTC4BTController.h"
#include "MCStatusLed.h"
#include "MCLed.h"
#include "enums.h"
#include "log4MC.h"

#define LIGHTS_ON true
#define LIGHTS_OFF false
#define LIGHTS_BLINK_DELAY_ON_CONNECT_MS 250

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 5;

// Duration between BLE discovery and connect attempts in seconds.
const uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 3;

// Sets the watchdog timeout (0D &lt; timeout in 0.1 secs, 1 byte &gt;)
// The purpose of the watchdog is to stop driving in case of an application failure.
// Watchdog starts when the first DRIVE command is issued during a connection.
// Watchdog is stopped when all channels are either set to zero drive, or are braking.
// The value is saved to the persistent store.
// The recommended watchdog frequency is 0.2-0.5 seconds, but a smaller and many larger settings are also available.
// Writing a zero disables the watchdog.
// By default watchdog is set to 5, which means a 0.5 second timeout.
const int8_t WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS = 3;

MTC4BTController::MTC4BTController() : MCController()
{
}

void MTC4BTController::Setup(MTC4BTConfiguration *config)
{
    // Keep controller configuration.
    _config = config;

    // Setup controller base configuration.
    MCController::Setup(_config);

    // Setup controller configuration.
    initLocomotives(config->Locomotives);

    // Initialize BLE client.
    log4MC::info("Setup: Initializing BLE...");
    NimBLEDevice::init("");

    // Configure BLE scanner.
    _scanner = NimBLEDevice::getScan();
    _scanner->setInterval(45);
    _scanner->setWindow(15);
    _scanner->setActiveScan(true);
    _hubScanner = new BLEHubScanner();

    // Start BLE device discovery task loop (will detect and connect to configured BLE devices).
    xTaskCreatePinnedToCore(this->discoveryLoop, "DiscoveryLoop", 3072, this, 1, NULL, 1);
}

void MTC4BTController::Loop()
{
    // Run the loop from the base MCController class (handles WiFi/MQTT connection monitoring and leds).
    MCController::Loop();
}

void MTC4BTController::HandleEmergencyBrake(const bool enabled)
{
    // Handle e-brake on all locomotives.
    for (BLELocomotive *loco : Locomotives)
    {
        loco->EmergencyBrake(enabled);
    }
}

BLELocomotive *MTC4BTController::GetLocomotive(uint address)
{
    for (BLELocomotive *loco : Locomotives)
    {
        if (loco->GetLocoAddress() == address)
        {
            return loco;
        }
    }

    return nullptr;
}

void MTC4BTController::HandleFn(int locoAddress, MCFunction f, const bool on)
{
    BLELocomotive *loco = GetLocomotive(locoAddress);
    if (!loco)
    {
        // Not a loco under our control. Ignore message.
        log4MC::vlogf(LOG_DEBUG, "Ctrl: Loco with address '%u' is not under our control. Fn command ignored.", locoAddress);
        return;
    }

    // Get applicable functions from loco.
    for (Fn *fn : loco->GetFn(f))
    {
        // Determine type of hardware.
        switch (fn->GetDeviceConfiguration()->GetHardwareType())
        {
        case HardwareType::EspPin:
        {
            // Handle function locally on the controller.
            MCController::HandleFn(fn, on);
            break;
        }
        case HardwareType::BleHub:
        {
            // Let loco handle the function.
            loco->HandleFn(fn, on);
            break;
        }
        }
    }
}

void MTC4BTController::discoveryLoop(void *parm)
{
    MTC4BTController *controller = (MTC4BTController *)parm;

    for (;;)
    {
        std::vector<BLEHub *> undiscoveredHubs;

        for (BLELocomotive *loco : controller->Locomotives)
        {
            if (!loco->IsEnabled() || loco->AllHubsConnected())
            {
                // Loco is not in use or all hubs are already connected. Skip to the next loco.
                continue;
            }

            for (BLEHub *hub : loco->Hubs)
            {
                if (!hub->IsEnabled())
                {
                    // Skip to the next Hub.
                    continue;
                }

                if (!hub->IsConnected())
                {
                    if (!hub->IsDiscovered())
                    {
                        // Hub not discovered yet, add to list of hubs to discover.
                        undiscoveredHubs.push_back(hub);
                    }

                    if (hub->IsDiscovered())
                    {
                        // Hub discovered, try to connect now.
                        if (!hub->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS))
                        {
                            // Connect attempt failed. Will retry in next loop.
                            log4MC::warn("Loop: Connect failed. Will retry...");
                        }
                        else
                        {
                            log4MC::vlogf(LOG_INFO, "Loop: Connected to all hubs of loco '%s'.", loco->GetLocoName().c_str());

                            if (loco->AllHubsConnected())
                            {
                                if (controller->GetEmergencyBrake())
                                {
                                    // Pass current e-brake status from controller to loco.
                                    loco->EmergencyBrake(true);
                                }
                                else
                                {
                                    // Blink lights three times when connected.
                                    hub->SetLights(LIGHTS_ON);
                                    delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                                    hub->SetLights(LIGHTS_OFF);
                                    delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                                    hub->SetLights(LIGHTS_ON);
                                    delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                                    hub->SetLights(LIGHTS_OFF);
                                    delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                                    hub->SetLights(LIGHTS_ON);
                                    delay(LIGHTS_BLINK_DELAY_ON_CONNECT_MS / portTICK_PERIOD_MS);
                                    hub->SetLights(LIGHTS_OFF);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (undiscoveredHubs.size() > 0)
        {
            // Start discovery for undiscovered hubs.
            controller->_hubScanner->StartDiscovery(controller->_scanner, undiscoveredHubs, BLE_SCAN_DURATION_IN_SECONDS);

            // Delay next discovery/connect attempts for a while, allowing the background tasks of already connected Hubs to send their periodic drive commands.
            delay(BLE_CONNECT_DELAY_IN_SECONDS * 1000 / portTICK_PERIOD_MS);
        }
    }
}

void MTC4BTController::initLocomotives(std::vector<BLELocomotiveConfiguration *> locoConfigs)
{
    for (BLELocomotiveConfiguration *locoConfig : locoConfigs)
    {
        // Keep an instance of the configured loco and pass it a reference to the controller.
        Locomotives.push_back(new BLELocomotive(locoConfig, this));
    }
}