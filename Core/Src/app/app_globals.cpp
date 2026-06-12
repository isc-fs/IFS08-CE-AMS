// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"

extern "C" {

osEventFlagsId_t safety_eventsHandle = nullptr;
osEventFlagsId_t bms_eventsHandle    = nullptr;

static const osEventFlagsAttr_t safety_events_attributes = {
    .name      = "safety_events",
    .attr_bits = 0,
    .cb_mem    = nullptr,
    .cb_size   = 0,
};

static const osEventFlagsAttr_t bms_events_attributes = {
    .name      = "bms_events",
    .attr_bits = 0,
    .cb_mem    = nullptr,
    .cb_size   = 0,
};

void ams_app_globals_init(void) {
    safety_eventsHandle = osEventFlagsNew(&safety_events_attributes);
    bms_eventsHandle    = osEventFlagsNew(&bms_events_attributes);
}

}  // extern "C"
