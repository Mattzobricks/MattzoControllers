/*
 * remoteList
 */

#include "remoteList/remoteList.h"

freeListItem::freeListItem(const char * newId, uint newAddr, RRdevice newRRtype, HubLedColor newledColour)
    : id{newId}, addr{newAddr}, RRtype{newRRtype}, ledColour{newledColour}
{
}

freeListItem::~freeListItem()
{
    if (id) {
        free((char *) id);
        id = NULL;
    }
}