#pragma once

#include "enums.h"
#include "rocrailitems/RRtypes.h"
#include "PUremoteButtons/PUButtonsMap.h"

class freeButtonItem
{
  public:
    freeButtonItem(const char *newId, int newAddr, RRdevice newRRtype, RRaction newAction);
    ~freeButtonItem();

    void setId(const char *newId);
    char *id;
    int addr;
    RRdevice RRtype;
    RRaction action;
};

