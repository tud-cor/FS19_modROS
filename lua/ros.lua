--[[
Copyright (c) 2021, TU Delft

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

author: Ting-Chia Chiang
author: G.A. vd. Hoorn
--]]

ros = {}

ros.Names = {}
ros.Transformations = {}
ros.Time = {}


-- function for legalizing a name for ROS resources
function ros.Names.sanatize(veh_name)
    -- replace whitespace with underscores and uppercase with lowercase to meet the ROS naming conventions for TF
    return veh_name:gsub(" ", "_"):lower()
end


--- Construct a quaternion from the specified Euler angles.
-- Function creates a table with four elements, representing a quaternion
-- at the same angle as the result of the Euler angle rotations specified by
-- the 'roll', 'pitch' and 'yaw' arguments.
--
-- NOTE: this follows ROS' conventions for order of arguments (RPY), order of
--       rotations and order of quaternion vector elements (ie: qx, qy, qz, qw).
--
-- @param roll The roll angle
-- @param pitch The pitch angle
-- @param yaw The yaw angle
-- @return a ROS compatible quaternion as a Lua table
function ros.Transformations.quaternion_from_euler(roll, pitch, yaw)
    local ci = math.cos(roll / 2)
    local cj = math.cos(pitch / 2)
    local ck = math.cos(yaw / 2)
    local si = math.sin(roll / 2)
    local sj = math.sin(pitch / 2)
    local sk = math.sin(yaw / 2)

    local cc = ci * ck
    local cs = ci * sk
    local sc = si * ck
    local ss = si * sk

    return {
        cj * sc - sj * cs,
        cj * ss + sj * cc,
        cj * cs - sj * sc,
        cj * cc + sj * ss
    }
end


function ros.Time.now()
    -- precision: 48164266.436659 ms with 6 digits after decimal point which is in nanosecond

    local day_time = g_currentMission.environment.dayTime
    local floor = math.floor

    local secs_data = floor(day_time / 1000)
    local nsecs_data = floor((day_time - floor(day_time / 1000) * 1000) * 1e6)

    return {secs = secs_data, nsecs = nsecs_data}
end
