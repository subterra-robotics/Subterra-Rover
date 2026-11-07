// flags.cpp - storage for the global mission flags declared (extern) in
// config.hpp. A header can only declare them, so they live here exactly once.

#include "config.hpp"

volatile bool g_recording   = false;
volatile bool g_stop_moving = false;
volatile bool g_go_back     = false;
