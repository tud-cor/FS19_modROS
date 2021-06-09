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

-- geometry_msgs_TransformStamped class
geometry_msgs_TransformStamped = {}
geometry_msgs_TransformStamped.ROS_MSG_NAME = "geometry_msgs/TransformStamped"

local geometry_msgs_TransformStamped_mt = Class(geometry_msgs_TransformStamped)

function geometry_msgs_TransformStamped.new()
    local self = {}
    setmetatable(self, geometry_msgs_TransformStamped_mt)

    -- fields as defined by geometry_msgs/TransformStamped
    self.header = {
        frame_id = "",
        stamp = {
            secs = 0,
            nsecs = 0
        }
    }
    self.child_frame_id = ""
    self.transform = {
        translation = {
            x = 0.0,
            y = 0.0,
            z = 0.0
        },
        rotation = {
            x = 0.0,
            y = 0.0,
            z = 0.0,
            w = 0.0
        }
    }

    return self
end

function geometry_msgs_TransformStamped:delete()
end

function geometry_msgs_TransformStamped:set(frame_id, stamp, child_frame_id, tx, ty, tz, qx, qy, qz, qw)
    -- directly overwrite local fields
    self.header = {
        frame_id = frame_id,
        stamp = stamp
    }
    self.child_frame_id = child_frame_id
    self.transform = {
        translation = {
            x = tx,
            y = ty,
            z = tz
        },
        rotation = {
            x = qx,
            y = qy,
            z = qz,
            w = qw
        }
    }
end

function geometry_msgs_TransformStamped:to_json()
    return json.stringify(self)
end
