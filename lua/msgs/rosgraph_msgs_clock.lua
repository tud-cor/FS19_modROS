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

-- rosgraph_msgs_Clock class
rosgraph_msgs_Clock = {}
rosgraph_msgs_Clock.__index = rosgraph_msgs_Clock

rosgraph_msgs_Clock.ros_msg_name = "rosgraph_msgs/Clock"

function rosgraph_msgs_Clock:init()
    local obj = {}
    setmetatable(obj, rosgraph_msgs_Clock)

    -- fields as defined by rosgraph_msgs/Clock
    obj.clock = {secs = 0, nsecs = 0}

    return obj
end

function rosgraph_msgs_Clock:set(clock)
    -- directly overwrite local fields
    self.clock = clock
end

function rosgraph_msgs_Clock:to_json()
    return json.stringify(self)
end
