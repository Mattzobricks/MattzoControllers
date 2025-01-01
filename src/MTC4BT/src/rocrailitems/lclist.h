/*
MIT License

Copyright (c) 2023 Hilbert Barelds

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once

#include <string.h>
#include <vector>
/*
    id
    V_max
    V_Rmax
    V_Smin
    V_Smax
    V_mode
    V
    dir
*/
typedef struct {
    bool fn;
    bool pushbutton;
} fn_t;

class lc
{
  public:
    lc();
    lc(char *id, int addr, bool vModePercent, int Vmax, int VRmax, int VSmax);
    lc(char *newId, int newAddr);
    lc(char *newId);
    lc(int newAddr);
    ~lc();

    // used for the std::find
    // example: std::vector<lc *>::iterator itr = std::find(locs.begin(), locs.end(), currentLc);
    bool operator==(const lc &rhs) const { return strcmp(this->id, rhs.id) == 0; }

    void setIdandAddr(const char *newId, const int newAddr, bool newInitiated);
    void setId(const char *newId);
    void clear();
    bool isSelected() { return id != nullptr; }
    bool motorDir() { return !dir ^ placing; }

    char *id;
    int addr;
    bool vModePercent; // true, if V_mode is percent
    int V;
    bool initiated;
    bool dir;
    bool placing;
    int Vmax;
    int VRmax;
    int VSmax;
    fn_t fn[32];
};

extern std::vector<lc *> locs;