#pragma once

#include <map>
#include <vector>

#include "remoteList/remoteFree.h"
#include "rocrailitems/RRtypes.h"
#include "rocrailitems/lclist.h"

class PUbuttonByType
{
  public:
    PUbuttonByType();
    ~PUbuttonByType();

    void setButton(RRdevice device, PUbutton button, RRaction action);
    RRaction getButton(RRdevice device, PUbutton button);

  protected:
    RRaction buttonByType[RRmaxdevice][maxButton];
};

class PUbuttonList
{
  public:
    PUbuttonList();
    ~PUbuttonList();

    void addButtonItem(PUbutton button, freeButtonItem *item);
    std::vector<lc *> getAllLocoItems();

  protected:
    std::vector<freeButtonItem *> buttons[maxButton];
};