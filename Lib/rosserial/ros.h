#pragma once

#include "STM32Hardware.h"
#include "ros/node_handle.h"

namespace ros {
typedef NodeHandle_<STM32Hardware, 25, 25, 512, 512> NodeHandle;
}