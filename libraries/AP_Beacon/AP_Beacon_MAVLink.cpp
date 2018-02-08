/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon_MAVLink.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_Mavlink::AP_Beacon_Mavlink(AP_Beacon &frontend) :
    AP_Beacon_Backend(frontend)
{
    // GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AP_Beacon_Mavlink init");
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Mavlink::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Mavlink::update(void)
{
}
