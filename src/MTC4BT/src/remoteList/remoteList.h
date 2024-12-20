#pragma once

/*
 * This remoteList class is used to hold the items for the remote list
 *
 *  - id
 *  - addr
 *  - type: loco, switch, signal, co
 *  - ledColour : (from enum.h)
 *
 */

#include "PUremoteButtons/PUButtons.h"
#include "enums.h"
#include "rocrailitems/RRtypes.h"

class freeListItem
{
  public:
    freeListItem(const char *newId, uint newAddr, RRdevice newRRtype, HubLedColor newledColour);
    ~freeListItem();

    RRdevice RRtype;
    const char *id;
    uint addr;
    HubLedColor ledColour;
};

typedef struct {
    PUbuttonByType *buttons;
    std::vector<freeListItem *> freeListItems;
} listModeType;