/*
 * remoteList
 */

#include "remoteList/remoteList.h"

freeListItem::freeListItem(const char * newId, uint newAddr, RRdevice newRRtype, HubLedColor newledColour)
    : addr{newAddr}, RRtype{newRRtype}, ledColour{newledColour}
{
    id = (char *)malloc(strlen(newId) + 1);
    strcpy(id, newId);
}

freeListItem::~freeListItem()
{
    if (id) {
        free((char *) id);
        id = NULL;
    }
}