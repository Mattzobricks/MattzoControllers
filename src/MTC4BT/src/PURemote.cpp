#include <Arduino.h>

#include "DriverTaskDelay.h"
#include "MTC4BTMQTTHandler.h"
#include "MattzoMQTTSubscriber.h"
#include "PURemote.h"
#include "log4MC.h"
#include <PubSubClient.h>

#include "remoteList/modetypes.h"

#define MAX_PUHUB_CHANNEL_COUNT 2

extern PubSubClient mqttSubscriberClient;

PURemote::PURemote(BLEHubConfiguration *config)
    : PUHub(config)
{
    log4MC::debug("Init remote");

    index = -1;

    // build loco list, because we need to update values (speed)
    // all other devices are just the id's
    // for the locos we need the short list to make it addressable by id and addr
    if (_config->mode == listMode) {
        currentLC = new lc(NULL, 0, false, 0, 0, 0);
    } else if (config->mode == freeMode) {
        // create a vector of lc's on all the buttons
        lcs = _config->buttons->getAllLocoItems();
    }
    // just for testing, will be filled with the config later on!
}

lc *PURemote::getPort(int address)
{
    if (currentLC->addr == address && address != 0) {
        return currentLC;
    }
    return nullptr;
}
lc *PURemote::getPort()
{
    return currentLC;
}

void PURemote::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    switch (pData[2]) {
    // case (byte)MessageType::HUB_PROPERTIES:
    // {
    //     parseDeviceInfo(pData);
    //     break;
    // }
    case (byte)PUMessageType::HUB_ATTACHED_IO:
        parsePortMessage(pData);
        break;

    case (byte)PUMessageType::HW_NETWORK_COMMANDS:
        parseHWNetworkCommandMessage(pData, length);
        break;

    case (byte)PUMessageType::PORT_VALUE_SINGLE:
        parsePortValueSingleMessage(pData, length);
        break;

    case (byte)PUMessageType::PORT_OUTPUT_COMMAND_FEEDBACK:
        parsePortAction(pData, length);
        break;
#ifdef DEBUGNOTIFYPUREMOTE
    default:
        dumpPData(pData, length);
#endif
    }
}
/**
 * @brief Parse the incoming characteristic notification for a HW Network Commands feedback
 * @param [in] pData The pointer to the received data
 * @param [in] length The length of to the received data
 */
void PURemote::parseHWNetworkCommandMessage(uint8_t *pData, size_t length)
{
    uint8_t value;
    switch (pData[3]) {
    case 0x02:            // H/W NetWork Command Type = 0x02 Connection Request [Upstream]
        value = pData[4]; // is the green button pressed or released? 1 or 0
        if (value == 1) {
            buttonHandleAction(PUbutton::Green);
        }
        break;

    default:
        break;
    }
}

bool PURemote::lookupLcById(char *id, int *locIndex)
{
    for (int i = 0; i < locs.size(); i++) {
        if (strcmp(locs[i]->id, id) == 0) {
            log4MC::vlogf(LOG_DEBUG, "Found loc by id %s '%d'", locs[i]->id, locs[i]->addr);
            *locIndex = i;
            return true;
        }
    }
    return false;
}

bool PURemote::lookupLcByAddr(int addr, int *locIndex)
{
    for (int i = 0; i < locs.size(); i++) {
        if (locs[i]->addr == addr) {
            log4MC::vlogf(LOG_DEBUG, "Found loc by addr %s '%d'", locs[i]->id, locs[i]->addr);
            *locIndex = i;
            return true;
        }
    }
    return false;
}

/// @brief set all locomotives id and addresses and call rocrail for more info about the loco's
/// @return false when the loco list is empty otherwise true

bool PURemote::setLCs()
{
    //  Make sure we have a loco list
    if (locs.size() == 0) {
        MTC4BTMQTTHandler::pubGetShortLcList();
        return false;
    }
    //  find address or id from the loco list

    for (auto item : lcs) {
        // we have a loco, lets find more info about them
        if (item->addr == -1) {
            // lookup by id
            if (lookupLcById(item->id, &index)) {
                item->addr = locs[index]->addr;
            } else {
                log4MC::vlogf(LOG_ERR, "Configuration error, locomotive id \"%s\" not found in the current plan, using \"-1\" as substitute, this error keeps ocuring.");
                item->addr = -1;
            }
        } else if (item->id == NULL) {
            // lookup by addr
            if (lookupLcByAddr(item->addr, &index)) {
                item->setId(locs[index]->id);
            } else {
                log4MC::vlogf(LOG_ERR, "Configuration error, locomotive address %d not found in the current plan, using \"DummyError\" as substitute");
                item->setId("DummyError");
            }
        } else if (item->id == NULL && item->addr == -1) {
            // This should not happen!
            log4MC::error("Somehow this locomotive is misconfigured, find the error in the json file.");
        }
        if (!item->initiated)
            MTC4BTMQTTHandler::pubGetLcInfo(item->id);
    }
    return true;
}

std::vector<lc *> PURemote::getLCs()
{
    return lcs;
}

bool PURemote::setColourAndLC(freeListItem *item)
{
    int index;
    //  find address or id from the loco list
    if (locs.size() == 0) {
        // locolist is empty, request a new list and return false (indicating: nothing happened)
        MTC4BTMQTTHandler::pubGetShortLcList();
        return false;
    }
    log4MC::vlogf(LOG_DEBUG, "%s: addr: %d, id(as pointer): %d", __func__, item->addr, item->id);
    // look up the loco in the locs list.
    if (item->addr == -1) {
        // lookup by id
        if (lookupLcById(item->id, &index)) {
            item->addr = locs[index]->addr;
        } else {
            log4MC::vlogf(LOG_ERR, "Configuration error, locomotive id \"%s\" not found in the current plan, using \"-1\" as substitute, this error keeps ocuring.");
            item->addr = -1;
        }
    } else if (item->id == NULL) {
        // lookup by addr
        if (lookupLcByAddr(item->addr, &index)) {
            item->setId(locs[index]->id);
        } else {
            log4MC::vlogf(LOG_ERR, "Configuration error, locomotive address %d not found in the current plan, using \"DummyError\" as substitute");
            item->setId("DummyError");
        }
    } else if (item->id == NULL && item->addr == -1) {
        // This should not happen!
        log4MC::error("Somehow this locomotive is misconfigured, find the error in the json file.");
    }
    SetHubLedColor(item->ledColour);
    currentLC->setIdandAddr(item->id, item->addr, false);
    MTC4BTMQTTHandler::pubGetLcInfo(item->id);
    return true;
}

void PURemote::incLocSpeed(lc *currentLC, int increment)
{
    if (currentLC->initiated) {
        int currentMotorSpeed = currentLC->motorDir() ? currentLC->V : -currentLC->V;
        int targetMotorSpeed = currentMotorSpeed + increment;
        if (targetMotorSpeed > currentLC->Vmax) {
            targetMotorSpeed = currentLC->Vmax;
        }
        if (targetMotorSpeed < -currentLC->Vmax) {
            targetMotorSpeed = -currentLC->Vmax;
        }
        int targetLocoSpeed = currentLC->placing ? targetMotorSpeed : -targetMotorSpeed;

        // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: currentMotorSpeed %d, vmax %d, increment %d, targetMotorSpeed %d, targetLocoSpeed %d",__func__, currentLC->addr, currentMotorSpeed, currentLC->Vmax, increment, targetMotorSpeed, targetLocoSpeed);

        setLocSpeed(currentLC, targetLocoSpeed);
    }
}

void PURemote::setLocSpeed(lc *currentLC, int V)
{
    if (currentLC->initiated) {
        MTC4BTMQTTHandler::pubLcSpeed(currentLC->id, V);
    }
}
/**
 * @brief Parse the incoming characteristic notification for a Port value change feedback
 * @param [in] pData The pointer to the received data
 * @param [in] length The length of to the received data
 */
void PURemote::parsePortValueSingleMessage(uint8_t *pData, size_t length)
{
    uint8_t port = pData[3];
    uint8_t value = pData[4];

    if ((port == 0) || (port == 1)) {
        // port A or B of the remote
        // dumpPData(pData, length);
        // plus  pressed = 0x01
        // red   pressed = 0x7f
        // minus pressed = 0xff
        PUbutton pressedButton;
        switch (value) {
        case 0x01: // plus
            pressedButton = (port == 0) ? PUbutton::Aplus : PUbutton::Bplus;
            break;
        case 0x7f: // red
            pressedButton = (port == 0) ? PUbutton::Ared : PUbutton::Bred;
            break;
        case 0xff: // minus
            pressedButton = (port == 0) ? PUbutton::Amin : PUbutton::Bmin;
            break;
        default:
            pressedButton = Bnone;
            break;
        }
        if (pressedButton != Bnone) {
            buttonHandleAction(pressedButton);
        }
    }
#ifdef DEBUGNOTIFYPUREMOTE
    else {
        dumpPData(pData, length);
    }
#endif
}
/**
 * @brief Parse the incoming characteristic notification for a Port output feedback
 * for know we see it as the ack
 * @param [in] pData The pointer to the received data
 */
void PURemote::parsePortAction(uint8_t *pData, size_t length)
{
    // empty function
#ifdef DEBUGNOTIFYPUREMOTE
    dumpPData(pData, length);
#endif
}

/**
 * @brief Parse the incoming characteristic notification for a Port Message
 * @param [in] pData The pointer to the received data
 */
void PURemote::parsePortMessage(uint8_t *pData)
{
    byte port = pData[3];
    bool isConnected = (pData[4] == 1 || pData[4] == 2) ? true : false;
    if (isConnected) {
        log4MC::vlogf(LOG_INFO, "port %x is connected with device %x", port, pData[5]);
        if (pData[5] == 0x0017) {
            _hubLedPort = port;
            log4MC::vlogf(LOG_INFO, "PU  : Found integrated RGB LED at port %x", port);
        }
        if (pData[5] == 0x0037) {
            // we have found switches, activate callback on button press
            byte setPortInputFormatSetup[8] = {0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01};
            writeValue(setPortInputFormatSetup, 8);
        }
    }
}

remoteModes PURemote::getMode()
{
    return _config->mode;
}
std::vector<freeListItem *> PURemote::getItemList()
{
    return _config->list.freeListItems;
}

freeListItem *PURemote::getItemByIndex(int index)
{
    return _config->list.freeListItems[index];
}

HubLedColor PURemote::getConfigLedColor()
{
    return _config->remoteColor;
}

void PURemote::buttonHandleAction(PUbutton button)
{
    if (_config->mode == listMode) {
        if (index == -1) {
            // ignore the buttons when index = -1
            // this is the init mode of the remote
            return;
        }
        int oldIndex = index;
        // get current selected device
        RRdevice device = _config->list.freeListItems[index]->RRtype;
        RRaction action = _config->list.buttons->getButton(device, button);

        log4MC::vlogf(LOG_DEBUG, "Got a key press device %d action %d", device, action);
        // make a nice switch statement here, maybe change it when implementing the freeMode ;-)
        switch (action) {
        case navUp:
            index = (index + 1) % _config->list.freeListItems.size();
            if (_config->list.freeListItems[index]->RRtype == RRloco) {
                if (!setColourAndLC(_config->list.freeListItems[index])) {
                    index = oldIndex; // no button is pressed, request to get loco list is send
                }
            } else {
                SetHubLedColor(_config->list.freeListItems[index]->ledColour);
            }
            break;
        case navDown:
            index -= 1;
            if (index < 0)
                index = _config->list.freeListItems.size() - 1;
            if (_config->list.freeListItems[index]->RRtype == RRloco) {
                if (!setColourAndLC(_config->list.freeListItems[index])) {
                    index = oldIndex; // no button is pressed, request to get loco list is send
                }
            } else {
                SetHubLedColor(_config->list.freeListItems[index]->ledColour);
            }
            break;
        case RRebrake:
            MTC4BTMQTTHandler::pubEBrake();
            break;
        case RRgo:
            MTC4BTMQTTHandler::pubGo();
            break;
        case RRinc:
            incLocSpeed(currentLC, 10);
            break;
        case RRdec:
            incLocSpeed(currentLC, -10);
            break;
        case RRstop:
            setLocSpeed(currentLC, 0);
            break;
        case RRflip:
            MTC4BTMQTTHandler::pubFlip(_config->list.freeListItems[index]->RRtype, _config->list.freeListItems[index]->id);
            break;
        case RRon:
        case RRoff:
            MTC4BTMQTTHandler::pubCo(action, _config->list.freeListItems[index]->id);
            break;
        case RRgreen:
        case RRred:
        case RRyellow:
        case RRwhite:
            MTC4BTMQTTHandler::pubSg(action, _config->list.freeListItems[index]->id);
            break;
        case RRleft:
        case RRright:
        case RRstraight:
        case RRturnout:
            MTC4BTMQTTHandler::pubSw(action, _config->list.freeListItems[index]->id);
            break;
        case RRnoop:
        default:
            break;
        }

    } else {
        if (index == -1) {
            // ignore the buttons when index = -1
            // this is the init mode of the remote
            return;
        }
        // get the list of items belonging to the pressed button
        std::vector<freeButtonItem *> freeItems = _config->buttons->getItemsByButton(button);
        log4MC::vlogf(LOG_DEBUG, "%s Number of actions %d button %d.", __func__, freeItems.size(), button);
        for (int i = 0; i < freeItems.size(); i++) {
            // make a nice switch statement here, maybe change it when implementing the freeMode ;-) Nope, freeMode is a bit different
            log4MC::vlogf(LOG_DEBUG, "%s Number of action %d button %d.", __func__, freeItems[i]->action, button);
            switch (freeItems[i]->action) {
            case RRebrake:
                MTC4BTMQTTHandler::pubEBrake();
                break;
            case RRgo:
                MTC4BTMQTTHandler::pubGo();
                break;
            case RRinc:
                incLocSpeed(freeItems[i]->loc, 10);
                break;
            case RRdec:
                incLocSpeed(freeItems[i]->loc, -10);
                break;
            case RRstop:
                setLocSpeed(freeItems[i]->loc, 0);
                break;
            case RRflip:
                MTC4BTMQTTHandler::pubFlip(freeItems[i]->RRtype, freeItems[i]->id);
                break;
            case RRon:
            case RRoff:
                MTC4BTMQTTHandler::pubCo(freeItems[i]->action, freeItems[i]->id);
                break;
            case RRgreen:
            case RRred:
            case RRyellow:
            case RRwhite:
                MTC4BTMQTTHandler::pubSg(freeItems[i]->action, freeItems[i]->id);
                break;
            case RRleft:
            case RRright:
            case RRstraight:
            case RRturnout:
                MTC4BTMQTTHandler::pubSw(freeItems[i]->action, freeItems[i]->id);
                break;
            case RRfn0:
            case RRfn1:
            case RRfn2:
            case RRfn3:
            case RRfn4:
            case RRfn5:
            case RRfn6:
            case RRfn7:
            case RRfn8:
            case RRfn9:
            case RRfn10:
            case RRfn11:
            case RRfn12:
            case RRfn13:
            case RRfn14:
            case RRfn15:
            case RRfn16:
            case RRfn17:
            case RRfn18:
            case RRfn19:
            case RRfn20:
            case RRfn21:
            case RRfn22:
            case RRfn23:
            case RRfn24:
            case RRfn25:
            case RRfn26:
            case RRfn27:
            case RRfn28:
            case RRfn29:
            case RRfn30:
            case RRfn31:
            case RRfn32:
                // a bit dirty but it will work in the case statement ;-)
                if (freeItems[i]->RRtype == RRloco) {
                    if ((freeItems[i]->fnAction == RRfn_on || freeItems[i]->fnAction == RRfn_off || freeItems[i]->fnAction == RRfn_flip)) {
                        // only fn actions for locomotives, all others are ignored
                        MTC4BTMQTTHandler::pubLcFn(freeItems[i]->id, freeItems[i]->action, freeItems[i]->fnAction);
                    } else if (freeItems[i]->fnAction == RRfn_push) {
                        MTC4BTMQTTHandler::pubLcFn(freeItems[i]->id, freeItems[i]->action, RRfn_on);
                        MTC4BTMQTTHandler::pubLcFn(freeItems[i]->id, freeItems[i]->action, RRfn_off);
                    }
                }
                break;
            case navUp:   // not supported in freeMode
            case navDown: // not supported in freeMode
            case RRnoop:  // not supported in freeMode
            default:
                break;
            }
            vTaskDelay(PUFREELISTACTIONDELAY / portTICK_PERIOD_MS); // don't spam mqtt
        }
    }
}