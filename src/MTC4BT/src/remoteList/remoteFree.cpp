/*
 * freeButtonItem
 */

#include "remoteList/remoteFree.h"
#include <cstdlib>
#include <string.h>

freeButtonItem::freeButtonItem(const char *newId, int newAddr, RRdevice newRRtype, RRaction newAction)
    : addr{newAddr}, RRtype{newRRtype}, action{newAction}
{
    if (newId) {
        id = (char *)malloc(strlen(newId) + 1);
        strcpy(id, newId);
    } else {
        id = NULL;
    }
}

freeButtonItem::~freeButtonItem()
{
    if (id) {
        free((char *)id);
        id = NULL;
    }
}
/*
    void setId(const char *newId);
    char *id;
    int addr;
    RRdevice RRtype;
    RRaction action;
*/
