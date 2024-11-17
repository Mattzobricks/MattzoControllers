#pragma once

#include <Arduino.h>

#include "BLEHub.h"
#include "PUHub.h"
#include "rocrailitems/lclist.h"

class PURemote : public PUHub
{
  public:
    PURemote(BLEHubConfiguration *config);
    void setHubParameter(BLEHubParam paramname, void *value);
    /**
     * @brief Callback function for notifications of a specific characteristic
     * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
     * @param [in] pData The pointer to the received data
     * @param [in] length The length of the data array
     * @param [in] isNotify
     */
    void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

    int getLowIndex();
    void setLowIndex(int index);

    int getMinRange();
    lc *getPortA(int address);
    lc *getPortB(int address);
    bool isRange;

  protected:
    lc *currentLCPortA;
    lc *currentLCPortB;
    int lowIndex; // needed for the ofset for the ledcolour
    int index;
    int minRange;
    int maxRange;
    int portA;
    int portB;
    void parsePortValueSingleMessage(uint8_t *pData, size_t length);
    void parseHWNeworkCommandMessage(uint8_t *pData, size_t length);
    void parsePortAction(uint8_t *pData, size_t length);
    void parsePortMessage(uint8_t *pData);
};