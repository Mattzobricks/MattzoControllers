#include <Arduino.h>

#include "DriverTaskDelay.h"
#include "PURemote.h"
#include "log4MC.h"
#include <PubSubClient.h>

#define MAX_PUHUB_CHANNEL_COUNT 2

extern PubSubClient mqttSubscriberClient;

PURemote::PURemote(BLEHubConfiguration *config)
    : PUHub(config)
{
    // do controller stuff
    currentLCPortA = new lc(nullptr, 0, false, 0, 0, 0);
    currentLCPortB = new lc(nullptr, 0, false, 0, 0, 0);
    // just for testing, will be filled with the config later on!
    index = -1;
    minRange = 3;
    maxRange = 10000;
}

int PURemote::getLowIndex()
{
    return lowIndex;
}

void PURemote::setLowIndex(int index)
{
    lowIndex = index;
}

int PURemote::getMinRange()
{
    return minRange;
}
void PURemote::setHubParameter(BLEHubParam paramname, void *value)
{
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
        parseHWNeworkCommandMessage(pData, length);
        break;

    case (byte)PUMessageType::PORT_VALUE_SINGLE:
        parsePortValueSingleMessage(pData, length);
        break;

    case (byte)PUMessageType::PORT_OUTPUT_COMMAND_FEEDBACK:
        parsePortAction(pData, length);
        break;
#ifdef DEBUGNOTIFYPU
    default:
        dumpPData(pData, length);
#endif
    }
}
/**
 * @brief Parse the incoming characteristic notification for a HW Network Commands feadback
 * @param [in] pData The pointer to the received data
 * @param [in] length The length of to the received data
 */
void PURemote::parseHWNeworkCommandMessage(uint8_t *pData, size_t length)
{
    uint8_t value;
    switch (pData[3]) {
    case 0x02:            // H/W NetWork Command Type = 0x02 Connection Request [Upstream]
        value = pData[4]; // is the green button pressed or released? 1 or 0
        if (value == 1) {
            // log4MC::info("Green button pressed.");
            // setHubLedColor(hubColour);
            mqttSubscriberClient.publish("rocrail/service/client", "<sys cmd=\"go\" informall=\"true\"/>");
            // force refresh locs list
            mqttSubscriberClient.publish("rocrail/service/client", "<model cmd=\"lclist\" val=\"short\"/>");
        }
        break;

    default:
        break;
    }
}
/**
 * @brief Parse the incoming characteristic notification for a Port value change feadback
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
        switch (value) {
        case 0x01: // plus  pressed = 0x01
            if (port == 1) {
                if (locs.size() == 0) {
                    mqttSubscriberClient.publish("rocrail/service/client", "<model cmd=\"lclist\" val=\"short\"/>");
                } else {
                    // find the next that is in the range, when at the last one, start at the first one again.
                    int roundcount = 0;
                    while (roundcount != 2) {
                        index++;
                        if (index >= locs.size()) {
                            index = 0;
                            roundcount++;
                        }
                        if (locs[index]->addr >= minRange && locs[index]->addr <= maxRange) {
                            // found a new index;
                            break;
                        }
                    }
                    log4MC::vlogf(LOG_DEBUG, "round %d, index %d, addr %d", roundcount, index, locs[index]->addr);
                    if (roundcount == 2) {
                        index = -1;
                    } else {
                        SetHubLedColor((HubLedColor)(((index - lowIndex) % 10) + 1));
                        currentLCPortA->setIdandAddr(locs[index]->id, locs[index]->addr);
                        // get current info of the loc from rocrail and start following it!
                        char request[200];
                        snprintf(request, 200, "<model cmd=\"lcprops\" val=\"%s\"/>", locs[index]->id);
                        mqttSubscriberClient.publish("rocrail/service/client", request);
                    }
                }

            } else {
                log4MC::info("Plus button pressed.");
            }
            break;
        case 0x7f: // red  pressed = 0x7f
            if (port == 1) {
                mqttSubscriberClient.publish("rocrail/service/client", "<sys cmd=\"ebreak\" informall=\"true\"/>");
            } else {
                log4MC::info("Red button pressed.");
            }
            break;
        case 0xff: // minus  pressed = 0x01
            if (port == 1) {
                if (locs.size() == 0) {
                    mqttSubscriberClient.publish("rocrail/service/client", "<model cmd=\"lclist\" val=\"short\"/>");
                } else {
                    // find the next that is in the range, when at the last one, start at the first one again.
                    int roundcount = 0;
                    while (roundcount != 2) {
                        index--;
                        if (index < 0) {
                            index = locs.size() - 1;
                            roundcount++;
                        }
                        if (locs[index]->addr >= minRange && locs[index]->addr <= maxRange) {
                            // found a new index;
                            break;
                        }
                    }
                    log4MC::vlogf(LOG_DEBUG, "round %d, index %d, addr %d", roundcount, index, locs[index]->addr);
                    if (roundcount == 2) {
                        index = -1;
                    } else {
                        SetHubLedColor((HubLedColor)(((index - lowIndex) % 10) + 1));
                        currentLCPortA->setIdandAddr(locs[index]->id, locs[index]->addr);
                        // get current info of the loc from rocrail and start following it!
                        char request[200];
                        snprintf(request, 200, "<model cmd=\"lcprops\" val=\"%s\"/>", locs[index]->id);
                        mqttSubscriberClient.publish("rocrail/service/client", request);
                    }
                }
            } else {
                log4MC::info("Minus button pressed.");
            }
            break;

        default:
            break;
        }
    }
#ifdef DEBUGNOTIFYPU
    else {
        dumpPData(pData, length);
    }
#endif
}
/**
 * @brief Parse the incoming characteristic notification for a Port output feadback
 * for know we see it as the ack
 * @param [in] pData The pointer to the received data
 */
void PURemote::parsePortAction(uint8_t *pData, size_t length)
{
    // empty function
    dumpPData(pData, length);
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
