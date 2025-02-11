// This file must be included after MLC_type.h and controller_config.h

// Array out of bounds check for signals
#if NUM_SIGNAL_ASPECTS > MAX_NUM_SIGNAL_ASPECTS
#error "NUM_SIGNAL_ASPECTS must  not be greater than MAX_NUM_SIGNAL_ASPECTS"
#endif

#if NUM_SIGNAL_LEDS > MAX_NUM_SIGNAL_LEDS
#error "NUM_SIGNAL_LEDS must not be greater than MAX_NUM_SIGNAL_LEDS"
#endif

#if NUM_SIGNAL_SERVOS > MAX_NUM_SIGNAL_SERVOS
#error "NUM_SIGNAL_SERVOS must not be greater than MAX_NUM_SIGNAL_SERVOS"
#endif

// Array out of bounds check for level crossing
#if LC_NUM_BOOM_BARRIERS > MAX_LC_NUM_BOOM_BARRIERS
#error "LC_NUM_BOOM_BARRIERS must not be greater than MAX_LC_NUM_BOOM_BARRIERS"
#endif

#if LC_NUM_LEDS > MAX_LC_NUM_LEDS
#error "LC_NUM_LEDS must not be greater than MAX_LC_NUM_LEDS"
#endif

#if LC_NUM_SENSORS > MAX_LC_NUM_SENSORS
#error "LC_NUM_SENSORS must not be greater than MAX_LC_NUM_SENSORS"
#endif

#if LC_NUM_TRACKS > MAX_LC_NUM_TRACKS
#error "LC_NUM_TRACKS must not be greater than MAX_LC_NUM_TRACKS"
#endif

// Array out of bounds check for bascule bridge
#if LC_NUM_BASCULE_BRIDGE_LEAFS > MAX_NUM_BASCULE_BRIDGE_LEAFS
#error "LC_NUM_BASCULE_BRIDGE_LEAFS may not be larger than: MAX_NUM_BASCULE_BRIDGE_LEAFS"
#endif
