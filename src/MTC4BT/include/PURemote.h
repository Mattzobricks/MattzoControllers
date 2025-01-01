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

    void buttonHandleAction(PUbutton button);
    bool setColourAndLC(freeListItem *item);
    bool setLCs();
    std::vector<lc *> getLCs();

    lc *getPort(int address);
    lc *getPort();
    remoteModes getMode();
    std::vector<freeListItem *> getItemList();
    int index;
    freeListItem *getItemByIndex(int index);

  protected:
    lc *currentLC;         // used in list mode
    std::vector<lc *> lcs; // used in free mode

    void parsePortValueSingleMessage(uint8_t *pData, size_t length);
    void parseHWNetworkCommandMessage(uint8_t *pData, size_t length);
    void parsePortAction(uint8_t *pData, size_t length);
    void parsePortMessage(uint8_t *pData);
    void setPortAColourAndLC();
    void incLocSpeed(lc *currentLC, int increment);
    void setLocSpeed(lc *currentLC, int V);

    bool lookupLcByAddr(int addr, int *locIndex);
    bool lookupLcById(char *id, int *locIndex);
};