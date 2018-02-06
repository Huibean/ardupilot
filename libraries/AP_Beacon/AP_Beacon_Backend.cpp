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

#include "AP_Beacon_Backend.h"
// debug
#include <stdio.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Beacon_Backend::AP_Beacon_Backend(AP_Beacon &frontend) :
    _frontend(frontend)
{
}

// set vehicle position:
// pos should be in the beacon's local frame in meters
// accuracy_estimate is also in meters
void AP_Beacon_Backend::set_vehicle_position(const Vector3f& pos, float accuracy_estimate)
{
    _frontend.veh_pos_update_ms = AP_HAL::millis();
    _frontend.veh_pos_accuracy = accuracy_estimate;
    _frontend.veh_pos_ned = correct_for_orient_yaw(pos);
}

// set individual beacon distance in meters
void AP_Beacon_Backend::set_beacon_distance(uint8_t beacon_instance, float distance)
{
    // sanity check instance
    if (beacon_instance >= AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance >= _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance+1;
    }

    _frontend.beacon_state[beacon_instance].distance_update_ms = AP_HAL::millis();
    _frontend.beacon_state[beacon_instance].distance = distance;
    _frontend.beacon_state[beacon_instance].healthy = true;
}

// configure beacon's position in meters from origin
// pos should be in the beacon's local frame (meters)
void AP_Beacon_Backend::set_beacon_position(uint8_t beacon_instance, const Vector3f& pos)
{
    // sanity check instance
    if (beacon_instance >= AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance >= _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance+1;
    }

    // set position after correcting yaw
    _frontend.beacon_state[beacon_instance].position = correct_for_orient_yaw(pos);
}

void AP_Beacon_Backend::handle_mavlink_msg(mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_BEACON_CONFIG:
        {
            mavlink_beacon_config_t packet;
            mavlink_msg_beacon_config_decode(msg, &packet);

            // ::printf("beacon_id: %d, x: %d, y: %d, z: %d \n", packet.beacon_id, packet.x, packet.y, packet.z);
            Vector3f beacon_pos(packet.x / 1000.0f, packet.y / 1000.0f, -packet.z / 1000.0f);
            set_beacon_position(packet.beacon_id, beacon_pos);
            break;
        }

    case MAVLINK_MSG_ID_BEACON_DISTANCE:
        {
            mavlink_beacon_distance_t packet;
            mavlink_msg_beacon_distance_decode(msg, &packet);

            // ::printf("beacon_id: %d, distance: %d \n", packet.beacon_id, packet.distance);
            set_beacon_distance(packet.beacon_id, packet.distance);
            break;
        }

    case MAVLINK_MSG_ID_BEACON_VEHICLE_POSITION:
        {
            mavlink_beacon_vehicle_position_t packet;
            mavlink_msg_beacon_vehicle_position_decode(msg, &packet);

            // ::printf("x: %d, y: %d, z: %d, position_error: %d \n", packet.x, packet.y, packet.z, packet.position_error);
            Vector3f veh_pos(Vector3f(packet.x / 1000.0f, packet.y / 1000.0f, -packet.z / 1000.0f));
            set_vehicle_position(veh_pos, packet.position_error);
            break;
        }
    }
}

// rotate vector (meters) to correct for beacon system yaw orientation
Vector3f AP_Beacon_Backend::correct_for_orient_yaw(const Vector3f &vector)
{
    // exit immediately if no correction
    if (_frontend.orient_yaw == 0) {
        return vector;
    }

    // check for change in parameter value and update constants
    if (orient_yaw_deg != _frontend.orient_yaw) {
        _frontend.orient_yaw = wrap_180(_frontend.orient_yaw.get());

        // calculate rotation constants
        orient_yaw_deg = _frontend.orient_yaw;
        orient_cos_yaw = cosf(radians(orient_yaw_deg));
        orient_sin_yaw = sinf(radians(orient_yaw_deg));
    }

    // rotate x,y by -orient_yaw
    Vector3f vec_rotated;
    vec_rotated.x = vector.x*orient_cos_yaw - vector.y*orient_sin_yaw;
    vec_rotated.y = vector.x*orient_sin_yaw + vector.y*orient_cos_yaw;
    vec_rotated.z = vector.z;
    return vec_rotated;
}
