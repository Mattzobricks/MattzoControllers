/*
 * Two classes:
 *  PUbuttonByType
 *  PUbuttonByAction
 */

#include "PUremoteButtons/PUButtons.h"
#include "log4MC.h"

/*
  It initializes the array by the defaults, which can later be overriden by the "buttons" section in the "list" mode
*/
PUbuttonByType::PUbuttonByType()
{
	buttonByType[RRloco][Aplus] = RRinc;
	buttonByType[RRloco][Ared] = RRstop;
	buttonByType[RRloco][Amin] = RRdec;
	buttonByType[RRloco][Bplus] = navUp;
	buttonByType[RRloco][Bred] = RRebrake;
	buttonByType[RRloco][Bmin] = navDown;
	buttonByType[RRloco][Green] = RRgo;

	buttonByType[RRswitch][Aplus] = RRstraight;
	buttonByType[RRswitch][Ared] = RRflip;
	buttonByType[RRswitch][Amin] = RRturnout;
	buttonByType[RRswitch][Bplus] = navUp;
	buttonByType[RRswitch][Bred] = RRebrake;
	buttonByType[RRswitch][Bmin] = navDown;
	buttonByType[RRswitch][Green] = RRgo;

	buttonByType[RRtripleswitch][Aplus] = RRleft;
	buttonByType[RRtripleswitch][Ared] = RRstraight;
	buttonByType[RRtripleswitch][Amin] = RRright;
	buttonByType[RRtripleswitch][Bplus] = navUp;
	buttonByType[RRtripleswitch][Bred] = RRebrake;
	buttonByType[RRtripleswitch][Bmin] = navDown;
	buttonByType[RRtripleswitch][Green] = RRgo;

	buttonByType[RRsignal][Aplus] = RRgreen;
	buttonByType[RRsignal][Ared] = RRflip;
	buttonByType[RRsignal][Amin] = RRred;
	buttonByType[RRsignal][Bplus] = navUp;
	buttonByType[RRsignal][Bred] = RRebrake;
	buttonByType[RRsignal][Bmin] = navDown;
	buttonByType[RRsignal][Green] = RRgo;

	buttonByType[RRoutput][Aplus] = RRon;
	buttonByType[RRoutput][Ared] = RRflip;
	buttonByType[RRoutput][Amin] = RRoff;
	buttonByType[RRoutput][Bplus] = navUp;
	buttonByType[RRoutput][Bred] = RRebrake;
	buttonByType[RRoutput][Bmin] = navDown;
	buttonByType[RRoutput][Green] = RRgo;
}

void PUbuttonByType::setButton(RRdevice device, PUbutton button, RRaction action)
{
	if (action != navDown && action != navUp) {
		buttonByType[device][button] = action;
	} else {
		log4MC::warn("PuButton: trying to set a reserved button action, ignored.");
	}
}

RRaction PUbuttonByType::getButton(RRdevice device, PUbutton button)
{
	return buttonByType[device][button];
}

PUbuttonList::PUbuttonList()
{
}

PUbuttonList::~PUbuttonList()
{
	for (auto item : buttons) {
		for (auto button : item)
			delete button;
	}
}

void PUbuttonList::addButtonItem(PUbutton button, freeButtonItem *item)
{
	buttons[button].push_back(item);
}

/// @brief iterate over all buttons and find the loco type an push it in a vector, so we can
/// keep track of them
/// @return list of non initiated locomotives
std::vector<lc *> PUbuttonList::getAllLocoItems()
{
	std::vector<lc *> locos;
	lc *loc = NULL;
	int foundIndex;
	for (auto item : buttons) {
		for (auto button : item)
			if (button->RRtype == RRloco) {
				if (findLocoByIdOrAddr(locos, &foundIndex, button->id, button->addr)) {
					// found the loco, put the refecerence in the button
					button->loc = locos[foundIndex];
				} else {
					loc = new lc(button->id, button->addr, false, 0, 0, 0);
					// store the pointer here, that will be easier for the button press
					button->loc = loc;
					// also here, for easier updateing, NOTE it is the same object!
					// updates are done through the locos list, only the speed update for the buttons
					// is done through the action
					locos.push_back(loc);
				}
			}
	}
	return locos;
}

bool PUbuttonList::findLocoByIdOrAddr(std::vector<lc *> locos, int *foundIndex, char *id, int addr)
{
	for (int i = 0; i < locos.size(); i++) {
		if ((id != NULL) &&
			(locos[i]->id != NULL) &&
			strcmp(locos[i]->id, id) == 0) {
			// found by index
			*foundIndex = i;
			return true;
		}
		if ((addr != -1) &&
			(locos[i]->addr != -1) &&
			(addr == locos[i]->addr)) {
			// found by addr
			*foundIndex = i;
			return true;
		}
	}
	return false;
}

std::vector<freeButtonItem *> PUbuttonList::getItemsByButton(PUbutton button)
{
	return buttons[button];
}