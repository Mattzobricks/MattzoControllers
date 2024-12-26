/*
 * remoteList
 */

#include "remoteList/remoteList.h"

freeListItem::freeListItem(const char *newId, int newAddr, RRdevice newRRtype, HubLedColor newledColour)
    : addr{newAddr}, RRtype{newRRtype}, ledColour{newledColour}
{
    if (newId) {
        id = (char *)malloc(strlen(newId) + 1);
        strcpy(id, newId);
    } else {
        id = NULL;
    }
}

freeListItem::~freeListItem()
{
    if (id) {
        free((char *)id);
        id = NULL;
    }
}

void freeListItem::setId(const char *newId)
{
    if (id) {
        free((char *)id);
        id = NULL;
    }
    if (newId) {
        id = (char *)malloc(strlen(newId) + 1);
        strcpy(id, newId);
    } else {
        id = NULL;
    }
}