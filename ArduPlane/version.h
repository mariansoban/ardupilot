#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V4.1.7-sobi-gyroflow"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,1,7,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 4
#define FW_MINOR 1
#define FW_PATCH 7
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
