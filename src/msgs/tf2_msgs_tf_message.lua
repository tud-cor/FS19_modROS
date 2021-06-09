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

-- tf2_msgs_TFMessage class
tf2_msgs_TFMessage = {}
tf2_msgs_TFMessage.__index = tf2_msgs_TFMessage

tf2_msgs_TFMessage.ros_msg_name = "tf2_msgs/TFMessage"

function tf2_msgs_TFMessage:init()
    local obj = {}
    setmetatable(obj, tf2_msgs_TFMessage)

    -- fields as defined by geometry_msgs/TfMessage
    obj.transforms = {}

    return obj
end

function tf2_msgs_TFMessage:set(transforms)
    -- directly overwrite local fields
    self.transforms = transforms
end

function tf2_msgs_TFMessage:add_transform(transform)
    table.insert(self.transforms, transform)
end

function tf2_msgs_TFMessage:to_json()
    return json.stringify(self)
end

-- geometry_msgs_TransformStamped class
geometry_msgs_TransformStamped = {}
geometry_msgs_TransformStamped.__index = geometry_msgs_TransformStamped

function geometry_msgs_TransformStamped:init()
    local obj = {}
    setmetatable(obj, geometry_msgs_TransformStamped)

    -- fields as defined by geometry_msgs/TransformStamped
    obj.header = {
        frame_id = "",
        stamp = {
            secs = 0,
            nsecs = 0
        }
    }
    obj.child_frame_id = ""
    obj.transform = {
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

    return obj
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
