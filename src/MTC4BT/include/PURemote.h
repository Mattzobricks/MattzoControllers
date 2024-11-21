#pragma once

#include <Arduino.h>

#include "BLEHub.h"
#include "PUHub.h"
#include "rocrailitems/lclist.h"

class PURemote : public PUHub
{
  public:
    PURemote(BLEHubConfiguration *config);
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
    lc *getPortA();
    lc *getPortB(int address);
    lc *getPortB();
    bool isRange;

  protected:
    lc *currentLCPortA;
    lc *currentLCPortB;
    int lowIndex; // needed for the offset for the led colour
    int index;
    int minRange;
    int maxRange;
    int portA;
    int portB;
    void parsePortValueSingleMessage(uint8_t *pData, size_t length);
    void parseHWNetworkCommandMessage(uint8_t *pData, size_t length);
    void parsePortAction(uint8_t *pData, size_t length);
    void parsePortMessage(uint8_t *pData);
    void setPortAColourAndLC();
    void incLocSpeed(lc *currentLC, int increment);
    void setLocSpeed(lc *currentLC, int value);
    void decLocSpeed(lc *currentLC, int decrement);
};