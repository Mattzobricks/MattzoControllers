#include "MTC4BTController.h"
#include "MCLed.h"
#include "MCStatusLed.h"
#include "enums.h"
#include "log4MC.h"

#include "MTC4BTMQTTHandler.h"
#include "PURemote.h"

// The priority at which the task should run.
// Systems that include MPU support can optionally create tasks in a privileged (system) mode by setting bit portPRIVILEGE_BIT of the priority parameter.
// For example, to create a privileged task at priority 2 the uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
#define Discovery_TaskPriority 2

// If the value is tskNO_AFFINITY, the created task is not pinned to any CPU, and the scheduler can run it on any core available.
// Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
// Specifying values larger than (portNUM_PROCESSORS - 1) will cause the function to fail.
#define Discovery_CoreID CONFIG_BT_NIMBLE_PINNED_TO_CORE

// The size of the task stack specified as the number of bytes.
#define Discovery_StackDepth 3072

// Blink duration in milliseconds. If all hubs if a loco are connected, its lights will blink for this duration.
const uint32_t BLINK_AT_CONNECT_DURATION_IN_MS = 3000;

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 2;

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

MTC4BTController::MTC4BTController() : MController()
{
}

void MTC4BTController::Setup(MTC4BTConfiguration *config)
{
    // Keep controller configuration.
    _config = config;

    // Setup basic MC controller configuration.
    MController::Setup(_config);

    // Setup MTC4BT specific controller configuration.
    initLocomotives(config->LocoConfigs);
    initRemotes(config->RemoteConfigs);
}

void MTC4BTController::SetupScanner()
{
    // Initialize BLE hub scanner.
    log4MC::info("Setup: Initializing BLE...");
    _hubScanner = new BLEHubScanner();

    // Start BLE device discovery task loop (will detect and connect to configured BLE devices).
    xTaskCreatePinnedToCore(this->discoveryLoop, "DiscoveryLoop", Discovery_StackDepth, this, Discovery_TaskPriority, NULL, Discovery_CoreID);
}

void MTC4BTController::Loop()
{
    // Run the loop from the base MCController class (handles WiFi/MQTT connection monitoring and leds).
    MController::Loop();

    // Handle e-brake on all locomotives.
    for (BLELocomotive *loco : Locomotives) {
        loco->SetEmergencyBrake(GetEmergencyBrake());
    }
}

bool MTC4BTController::HasLocomotive(uint address)
{
    return getLocomotive(address);
}

void MTC4BTController::HandleSys(const bool ebrakeEnabled)
{
    // Update global e-brake status.
    SetEmergencyBrake(ebrakeEnabled);
}

void MTC4BTController::HandleLc(int locoAddress, int speed, int minSpeed, int maxSpeed, char *mode, bool dirForward)
{
    for (BLELocomotive *loco : Locomotives) {
        // BLELocomotive *loco = getLocomotive(locoAddress);
        if (loco->GetLocoAddress() == locoAddress) {
            // Loco is under the control of this controller. Process command!

            // Calculate target speed percentage (as percentage if mode is "percent", or else as a percentage of max speed).
            int targetSpeedPerc = strcmp(mode, "percent") == 0 ? speed : (speed * maxSpeed) / 100;

            // Calculate direction multiplier (1 or -1)
            int8_t dirMultiplier = dirForward ? 1 : -1;

            // Log message and execute drive command.
            log4MC::vlogf(LOG_DEBUG, "Ctrl: Received lc command for loco address '%u', speed %i.", locoAddress, targetSpeedPerc * dirMultiplier);
            loco->Drive(minSpeed, targetSpeedPerc * dirMultiplier);
        }
    }
}

std::vector<lc *> MTC4BTController::findRemoteByAddr(int addr)
{
    std::vector<lc *> remotes;
    remotes.reserve(6);
    for (BLERemote *remote : Remotes) {
        for (BLEHub *hub : remote->Hubs) {
            if (hub->GetHubType() == BLEHubType::PUController) {
                PURemote *remoteHub = (PURemote *)hub;
                //  find the index where minRange is valid
                lc *locoLc;
                if (remoteHub->getMode() == listMode) {
                    locoLc = remoteHub->getPort(addr);
                    if (locoLc)
                        remotes.push_back(locoLc);
                } else {
                    // TODO: free mode, check all items
                }
            }
        }
    }
    return remotes;
}

void MTC4BTController::handleLCList()
{
    /* for all controller 'locomotives' for all hubs, find the PURemotes (HubType = BLEHubType::PUController)
     */
    for (BLERemote *remote : Remotes) {
        for (BLEHub *hub : remote->Hubs) {
            if (hub->GetHubType() == BLEHubType::PUController) {

                PURemote *remoteHub = (PURemote *)hub;
                if (remoteHub->getMode() == listMode) {
                    lc *remoteLc = remoteHub->getPort(); // get current selected locomotive
                    for (int i = 0; i < locs.size(); i++) {
                        // find the id of the locos, and set them in the itemlist!
                        std::vector<freeListItem *> itemList = remoteHub->getItemList();
                        for (int j = 0; j < itemList.size(); j++) {
                            if (itemList[j]->addr < 0 && strcmp(itemList[j]->id,locs[i]->id) == 0) {
                                itemList[j]->addr = locs[i]->addr;
                            }
                            if (itemList[j]->addr > 0 && itemList[j]->addr == locs[i]->addr) {
                                if (itemList[j]->id) {
                                    free(itemList[j]->id);
                                    itemList[j]->id = (char *)malloc(strlen(locs[i]->id) + 1);
                                    strcpy(itemList[j]->id, locs[i]->id);
                                }
                            }
                        }

                        if (remoteLc->addr == locs[i]->addr) {
                            remoteLc->setIdandAddr(locs[i]->id, locs[i]->addr);
                            // get the loc info
                            log4MC::vlogf(LOG_DEBUG, "A id addr %s %d", locs[i]->id, locs[i]->addr);
                            MTC4BTMQTTHandler::pubGetLcInfo(locs[i]->id);
                        }
                    }
                } else {
                    // TODO: FreeMode
                    /*
                    lc *lcportA = remoteHub->getPortA();
                    lc *lcportB = remoteHub->getPortB();
                    // set lcportA and LCportB
                    for (int i = 0; i < locs.size(); i++) {
                        // find the id of the locos.
                        if (lcportA->addr == locs[i]->addr) {
                            lcportA->setIdandAddr(locs[i]->id, locs[i]->addr);
                            // get the loc info
                            log4MC::vlogf(LOG_DEBUG, "A id addr %s %d", locs[i]->id, locs[i]->addr);
                            MTC4BTMQTTHandler::pubGetLcInfo(locs[i]->id);
                        }
                        if (lcportB->addr == locs[i]->addr) {
                            lcportB->setIdandAddr(locs[i]->id, locs[i]->addr);
                            // get the loc info
                            log4MC::vlogf(LOG_DEBUG, "B id addr %s %d", locs[i]->id, locs[i]->addr);
                            MTC4BTMQTTHandler::pubGetLcInfo(locs[i]->id);
                        }
                    }
                    */
                    remoteHub->SetHubLedColor(GREEN);
                }
            }
        }
    }
}
void MTC4BTController::HandleTrigger(int locoAddress, MCTriggerSource source, std::string eventType, std::string eventId, std::string value)
{
    bool locoFound = false;
    for (BLELocomotive *loco : Locomotives) {
        if (loco->GetLocoAddress() == locoAddress) {
            loco->TriggerEvent(source, eventType, eventId, value);
            locoFound = true;
        }
    }
    if (!locoFound) {
        // Not a loco under our control. Ignore trigger.
        log4MC::vlogf(LOG_DEBUG, "Ctrl: Loco with address '%u' is not under our control. Trigger ignored.", locoAddress);
    }
}

void MTC4BTController::discoveryLoop(void *parm)
{
    MTC4BTController *controller = (MTC4BTController *)parm;

    for (;;) {
        std::vector<BLEHub *> undiscoveredHubs;

        // Locomotives
        for (BLELocomotive *loco : controller->Locomotives) {
            // All loco hubs are already connected. Skip to the next loco.
            if (loco->AllHubsConnected()) {
                continue;
            }

            for (BLEHub *hub : loco->Hubs) {
                if (hub->IsConnected()) {
                    // Hub is already connected. Skip to the next Hub.
                    continue;
                }

                if (hub->IsDiscovered()) {
                    // Hub discovered, try to connect now.
                    if (hub->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS)) {
                        if (loco->AllHubsConnected()) {
                            log4MC::vlogf(LOG_INFO, "Loop: Connected to all hubs of loco '%s'.", loco->GetLocoName().c_str());

                            // Blink lights for a while when connected.
                            loco->BlinkLights(BLINK_AT_CONNECT_DURATION_IN_MS);
                        }
                    } else {
                        // Connect attempt failed. Will retry in next loop.
                        log4MC::warn("Loop: Connect failed. Will retry...");
                    }
                } else {
                    // Hub not discovered yet, add to list of hubs to discover.
                    undiscoveredHubs.push_back(hub);
                }

                delay(50 / portTICK_PERIOD_MS);
            }
        }

        // Remotes
        for (BLERemote *remote : controller->Remotes) {
            // All loco hubs are already connected. Skip to the next loco.
            if (remote->AllHubsConnected()) {
                continue;
            }

            for (BLEHub *hub : remote->Hubs) {
                if (hub->IsConnected()) {
                    // Hub is already connected. Skip to the next Hub.
                    continue;
                }

                if (hub->IsDiscovered()) {
                    // Hub discovered, try to connect now.
                    if (hub->Connect(WATCHDOG_TIMEOUT_IN_TENS_OF_SECONDS)) {
                        if (remote->AllHubsConnected()) {
                            log4MC::vlogf(LOG_INFO, "Loop: Connected to all remote hubs.");

                            // Blink lights for a while when connected.
                            remote->BlinkLights(BLINK_AT_CONNECT_DURATION_IN_MS);
                        }
                    } else {
                        // Connect attempt failed. Will retry in next loop.
                        log4MC::warn("Loop: Connect failed. Will retry...");
                    }
                } else {
                    // Hub not discovered yet, add to list of hubs to discover.
                    undiscoveredHubs.push_back(hub);
                }

                delay(50 / portTICK_PERIOD_MS);
            }
        }

        if (undiscoveredHubs.size() > 0 || controller->Locomotives.size() == 0) {
            // Start discovery for undiscovered hubs.
            controller->_hubScanner->StartDiscovery(undiscoveredHubs, BLE_SCAN_DURATION_IN_SECONDS);
        }

        // Delay next discovery/connect attempts for a while, allowing the background tasks of already connected Hubs to send their periodic drive commands.
        delay(BLE_CONNECT_DELAY_IN_SECONDS * 1000 / portTICK_PERIOD_MS);
    }
}

void MTC4BTController::initLocomotives(std::vector<BLELocomotiveConfiguration *> locoConfigs)
{
    for (BLELocomotiveConfiguration *locoConfig : locoConfigs) {
        // Keep an instance of the configured loco and pass it a reference to this controller.
        Locomotives.push_back(new BLELocomotive(locoConfig, this));
    }
}

void MTC4BTController::initRemotes(std::vector<BLERemoteConfiguration *> remoteConfigs)
{
    for (BLERemoteConfiguration *remoteConfig : remoteConfigs) {
        // Keep an instance of the configured loco and pass it a reference to this controller.
        Remotes.push_back(new BLERemote(remoteConfig, this));
    }
}

BLELocomotive *MTC4BTController::getLocomotive(uint address)
{
    for (BLELocomotive *loco : Locomotives) {
        if (loco->GetLocoAddress() == address) {
            return loco;
        }
    }
    return nullptr;
}