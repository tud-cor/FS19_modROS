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
tf2_msgs_TFMessage.ROS_MSG_NAME = "tf2_msgs/TFMessage"

local tf2_msgs_TFMessage_mt = Class(tf2_msgs_TFMessage)

function tf2_msgs_TFMessage.new()
    local self = {}
    setmetatable(self, tf2_msgs_TFMessage_mt)

    -- fields as defined by tf2_msgs/TFMessage
    self.transforms = {}

    return self
end

function tf2_msgs_TFMessage:delete()
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
