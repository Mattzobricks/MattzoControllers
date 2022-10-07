/*
* This file must be included after MLC_type.h and a configuratun file
*/
#ifndef LC_NUM_LEDS
#error "LC_NUM_LEDS not defined, it must be a #define, not a const"
#endif

#ifndef LC_NUM_TRACKS
#error "LC_NUM_TRACKS not defined, it must be a #define, not a const"
#endif

#ifndef LC_NUM_BOOM_BARRIERS
#error "LC_NUM_BOOM_BARRIERS not defined, it must be a #define, not a const"
#endif

#ifndef NUM_SIGNAL_ASPECTS
#error "NUM_SIGNAL_ASPECTS not defined, it must be a #define, not a const"
#endif

#ifndef NUM_SIGNAL_LEDS
#error "NUM_SIGNAL_LEDS not defined, it must be a #define, not a const"
#endif

#ifndef NUM_SIGNAL_SERVOS
#error "NUM_SIGNAL_SERVOS not defined, it must be a #define, not a const"
#endif

#ifndef NUM_BASCULE_BRIDGE_LEAFS
#error "NUM_BASCULE_BRIDGE_LEAFS not defined, it must be a #define, not a const"
#endif

#if LC_NUM_LEDS > MAX_LC_NUM_LEDS
#error "LC_NUM_LEDS may not be larger than: MAX_LC_NUM_LEDS"
#endif

#if LC_NUM_TRACKS > MAX_LC_NUM_TRACKS
#error "LC_NUM_TRACKS may not be larger than: MAX_LC_NUM_TRACKS"
#endif

#if LC_NUM_BOOM_BARRIERS > MAX_LC_NUM_BOOM_BARRIERS
#error "LC_NUM_BOOM_BARRIERS may not be larger than: MAX_LC_NUM_BOOM_BARRIERS"
#endif 

#if LC_NUM_SIGNAL_ASPECTS > MAX_NUM_SIGNAL_ASPECTS
#error "LC_NUM_SIGNAL_ASPECTS may not be larger than: MAX_NUM_SIGNAL_ASPECTS"
#endif

#if LC_NUM_SIGNAL_LEDS > MAX_NUM_SIGNAL_LEDS
#error "LC_NUM_SIGNAL_LEDS may not be larger than: MAX_NUM_SIGNAL_LEDS"
#endif

#if LC_NUM_SIGNAL_SERVOS > MAX_NUM_SIGNAL_SERVOS
#error "LC_NUM_SIGNAL_SERVOS may not be larger than: MAX_NUM_SIGNAL_SERVOS"
#endif

#if LC_NUM_BASCULE_BRIDGE_LEAFS > MAX_NUM_BASCULE_BRIDGE_LEAFS
#error "LC_NUM_BASCULE_BRIDGE_LEAFS may not be larger than: MAX_NUM_BASCULE_BRIDGE_LEAFS"
#endif