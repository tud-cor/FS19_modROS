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


author: Ting-Chia Chiang, G.A. vd. Hoorn
maintainer: Ting-Chia Chiang, G.A. vd. Hoorn

--]]

Publisher = {}

local Publisher_mt = Class(Publisher)


function Publisher.new(connection, topic_name, message_type)
    local self = {}
    setmetatable(self, Publisher_mt)

    if message_type['ROS_MSG_NAME'] == nil then
        -- TODO: implement proper logging
        print(("message_type must have ROS_MSG_NAME key, can't find it in '%s'"):format(message_type))
        return nil, "ctor error: invalid message_type"
    end

    -- NOTE: there is no locking, so concurrent access to the connection is not
    -- guarded against. Serialisation of access is responsibility of owner of
    -- the Publisher object
    self._conx = connection

    -- "topic_name" could be with or without namespace
    -- with e.g. "<ros_veh_name_with_id>/odom"
    -- wihtout e.g. "clock"
    self._topic_name = topic_name

    -- we need this to be able to tell the Python side what sort of message
    -- we're serialising
    self._message_type_name = message_type.ROS_MSG_NAME

    return self
end

function Publisher:delete()
    -- nothing to do. Connection object is not owned by this Publisher
end

-- register publishers- advertise that we are publishing a topic
function Publisher:advertise()
    if not self._conx:is_connected() then
        return nil, "Can't write to nil fd"
    end

    -- try writing the serialised rosbridge message to the connection
    local data = "{\"op\":\"advertise\",\"topic\": \"" .. self._topic_name .. "\",\"type\":\"" .. self._message_type_name .. "\"}"
    local data_len = string.len(data)
    local ret, err = self._conx:write(data)

    if not ret then
        return nil, "Error advertise message: '" .. err .. "'"
    end
    return ret
end

function Publisher:publish(msg)
    if msg.ROS_MSG_NAME ~= self._message_type_name then
        return nil, ("Can't publish '%s' with publisher of type '%s'"):format(msg.ROS_MSG_NAME, self._message_type_name)
    end
    if not self._conx:is_connected() then
        return nil, "Can't write to nil fd"
    end

    -- try writing the serialised message to the connection
    local ret, err = self._conx:write(self._topic_name .. "\n" .. msg.ROS_MSG_NAME .. "\n" .. msg:to_json())
    if not ret then
        return nil, "Error publishing message: '" .. err .. "'"
    end
    return ret
end
