#pragma once

#include "PUremoteButtons/PUButtonsMap.h"
#include "enums.h"
#include "rocrailitems/RRtypes.h"
#include "rocrailitems/lclist.h"

class freeButtonItem
{
  public:
    freeButtonItem(const char *newId, int newAddr, RRdevice newRRtype, RRaction newAction, RRfnAction newFnAction);
    ~freeButtonItem();

    void setId(const char *newId);
    char *id;
    int addr;
    lc *loc;
    RRdevice RRtype;
    RRaction action;
    RRfnAction fnAction;
};
