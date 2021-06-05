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

-- sensor_msgs_Imu class
sensor_msgs_Imu = {}
sensor_msgs_Imu.__index = sensor_msgs_Imu

sensor_msgs_Imu.ros_msg_name = "sensor_msgs/Imu"

function sensor_msgs_Imu:init()
    local obj = {}
    setmetatable(obj, sensor_msgs_Imu)

    -- fields as defined by geometry_msgs/sensor_msgs_Imu
    obj.header = {
        frame_id = "",
        stamp = {secs = 0, nsecs = 0}
    }
    obj.orientation = {
        x = 0.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    }
    obj.angular_velocity = {
        x = 0.0,
        y = 0.0,
        z = 0.0
    }
    obj.linear_acceleration = {
        x = 0.0,
        y = 0.0,
        z = 0.0
    }

    return obj
end

function sensor_msgs_Imu:set(frame_id, stamp, qx, qy, qz, qw, avx, avy, avz, lax, lay, laz)
    -- directly overwrite local fields
    self.header = {
        frame_id = frame_id,
        stamp = stamp
    }
    self.orientation = {
        x = qx,
        y = qy,
        z = qz,
        w = qw
    }
    self.angular_velocity = {
        x = avx,
        y = avy,
        z = avz
    }
    self.linear_acceleration = {
        x = lax,
        y = lay,
        z = laz
    }
end

function sensor_msgs_Imu:to_json()
    return json.stringify(self)
end
