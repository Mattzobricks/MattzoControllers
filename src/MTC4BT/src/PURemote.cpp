#include <Arduino.h>

#include "DriverTaskDelay.h"
#include "PURemote.h"
#include "log4MC.h"

#define MAX_PUHUB_CHANNEL_COUNT 2

PURemote::PURemote(BLEHubConfiguration *config)
    : PUHub(config)
{
    // do controller stuff
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

        // case (byte)MessageType::PORT_VALUE_SINGLE:
        // {
        //     parseSensorMessage(pData);
        //     break;
        // }
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
            // we have found switches
            byte setPortInputFormatSetup[8] = {0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01};
            writeValue(setPortInputFormatSetup, 8);
        }
    }
}
