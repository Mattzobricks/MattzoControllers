/*
 * freeButtonItem
 */

#include "remoteList/remoteFree.h"
#include "log4MC.h"
#include <cstdlib>
#include <string.h>

freeButtonItem::freeButtonItem(const char *newId, int newAddr, RRdevice newRRtype, RRaction newAction, RRfnAction newFnAction)
	: addr{newAddr}, RRtype{newRRtype}, action{newAction}, fnAction{newFnAction}
{
	if (newId) {
		id = (char *)malloc(strlen(newId) + 1);
		strcpy(id, newId);
	} else {
		id = NULL;
	}
	log4MC::vlogf(LOG_DEBUG, "%s action %d device %d.", __func__, newAction, newRRtype);
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
