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
#include "lclist.h"
#include "log4MC.h"

lc::lc(char *newId, int addr, bool vModePercent, int Vmax, int VRmax, int VSmax)
	: addr(addr), vModePercent(vModePercent), Vmax(VSmax), VRmax(VRmax), VSmax(VSmax)
{
	V = 0;
	initiated = false;
	if (newId) {
		id = (char *)malloc(strlen(newId) + 1);
		strcpy(id, newId);
	} else {
		id = NULL;
	}
	initializeFn();
}

lc::lc()
{
	id = nullptr;
	V = 0;
	initiated = false;
	initializeFn();
}

lc::lc(char *newId)
	: lc()
{
	id = (char *)malloc(strlen(newId) + 1);
	strcpy(id, newId);
	initializeFn();
}

lc::lc(char *newId, int newAddr)
	: addr(newAddr)
{
	id = (char *)malloc(strlen(newId) + 1);
	strcpy(id, newId);
	initializeFn();
}

lc::lc(int newAddr)
	: addr(newAddr)
{
	id = nullptr;
	V = 0;
	initiated = false;
	initializeFn();
}

lc::~lc()
{
	if (id) {
		log4MC::vlogf(LOG_DEBUG, "Delete %s", id);
		free(id);
		id = nullptr;
	}
}
void lc::setIdandAddr(const char *newId, const int newAddr, bool newInitiated)
{
	if (id) {
		free(id);
		id = NULL;
	}
	if (newId) {
		id = (char *)malloc(strlen(newId) + 1);
		strcpy(id, newId);
	} else {
		id = NULL;
	}
	addr = newAddr;
	initiated = newInitiated;
}

void lc::setId(const char *newId)
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

void lc::clear()
{
	if (id) {
		free(id);
		id = nullptr;
	}
	initiated = false;
}

void lc::initializeFn()
{
	for (int i = 0; i < NUM_LOCO_FUNCTIONS; i++) {
		fn[i] = false;
	}
}

std::vector<lc *> locs;
