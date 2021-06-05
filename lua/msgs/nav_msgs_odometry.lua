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

nav_msgs_Odometry = {}
nav_msgs_Odometry.__index = nav_msgs_Odometry

nav_msgs_Odometry.ros_msg_name = "nav_msgs/Odometry"

function nav_msgs_Odometry:init()
    local obj = {}
    setmetatable(obj, nav_msgs_Odometry)

    -- fields as defined by nav_msgs/Odometry
    obj.header = {
        frame_id = "",
        stamp = {secs = 0, nsecs = 0}
    }
    obj.child_frame_id = ""
    obj.pose = {
        pose = {
            position = {
                x = 0.0,
                y = 0.0,
                z = 0.0
            },
            orientation = {
                x = 0.0,
                y = 0.0,
                z = 0.0,
                w = 0.0
            }
        }
    }
    obj.twist = {
        twist = {
            linear = {
                x = 0.0,
                y = 0.0,
                z = 0.0
            },
            angular = {
                x = 0.0,
                y = 0.0,
                z = 0.0
            }
        }
    }

    return obj
end

function nav_msgs_Odometry:set(frame_id, stamp, child_frame_id, px, py, pz, qx, qy, qz, qw, tlx, tly, tlz, tax, tay, taz)
    -- directly overwrite local fields
    self.header = {
        frame_id = frame_id,
        stamp = stamp
    }
    self.child_frame_id = child_frame_id
    self.pose = {
        pose = {
            position = {
                x = pz,
                y = px,
                z = py
            },
            orientation = {
                x = qx,
                y = qy,
                z = qz,
                w = qw
            }
        }
    }
    self.twist = {
        twist = {
            linear = {
                x = tlx,
                y = tly,
                z = tlz
            },
            angular = {
                x = tax,
                y = tay,
                z = taz
            }
        }
    }
end

function nav_msgs_Odometry:to_json()
    return json.stringify(self)
end
