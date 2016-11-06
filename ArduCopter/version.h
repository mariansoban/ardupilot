#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Copter V3.4.3-phl-sobi-8fm"
#define FIRMWARE_VERSION 3,4,3,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
