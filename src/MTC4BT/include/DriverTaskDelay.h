#pragma once
/*
The default value is 100 and may be overridded by a -D flag, only change if you know what you
are doing, change on your own risc. DO NOT CHANGE THE DEFAULT VALUE, only use
-DDRIVERTASKDELAY=150
or some other value
*/

#ifndef DRIVERTASKDELAY
#define DRIVERTASKDELAY 100
#endif

#ifndef PUFREELISTACTIONDELAY
#define PUFREELISTACTIONDELAY 250
#endif