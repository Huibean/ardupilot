#pragma once

#include "AP_Beacon_Backend.h"

class AP_Beacon_Mavlink : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_Mavlink(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy();

    // update
    void update();

private:
    uint32_t last_update_ms = 0;
};
