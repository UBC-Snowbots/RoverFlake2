// The one and only place that decides which message types get their own topic.
//
// This is the only file in this package that includes the comms library, and it
// only uses it for the MSG_TYPE__* ids so that the topic names and the message
// ids can never drift apart.
#pragma once

#include "comms.h"   // see CMakeLists.txt if this include cannot be found

#include <cstdint>

// Returns the topic name suffix this message type should be published on, or
// nullptr if it should go to the generic "incoming" topic.
//
// TO ADD A TYPED TOPIC: add a case. The node walks every message
// type at startup and creates a publisher for anything that returns a name.
static inline const char* type_to_topic_suffix(uint16_t type)
{
    switch (type) {
    case MSG_TYPE__HEARTBEAT:       return "/can/heartbeats";
    // case MSG_TYPE__SET_LED_PWM:     return "/can/set_led_pwm";
    // case MSG_TYPE__HARDWARE_ERROR:  return "/can/hardware_error";
    default:                        return nullptr;   // -> /can/<loop_name>/incoming
    }
}