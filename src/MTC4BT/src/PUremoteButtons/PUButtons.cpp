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
