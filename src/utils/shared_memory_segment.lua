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

author: G.A. vd. Hoorn
--]]

SharedMemorySegment = {}
SharedMemorySegment.__index = SharedMemorySegment

function SharedMemorySegment:init(size)
    local segment = {}
    setmetatable(segment, SharedMemorySegment)

    -- describe the buffer
    segment.len = size
    segment.marker_start = "fs_lua_memorp_buffer_0012"
    segment.marker_end = "fs_lua_memorp_buffer_end_0012"

    -- number of chars we use to encode buffer length
    segment.len_marker_len = 1

    -- index at which the bytes we're interested in actually start.
    -- add an extra 1 because Lua is 1-indexed
    segment.data_start = segment.marker_start:len() + segment.len_marker_len + 1

    -- construct the 'buffer string', give it a special contents:
    --
    --  - first the start sentinel value
    --  - then the length of the actual buffer in bytes as a single character
    --  - then we make some space by adding 'len' spaces
    --  - finally the end sentinel value
    segment.segment_data = segment.marker_start .. string.char(segment.len) .. string.rep(" ", segment.len) .. segment.marker_end

    return segment
end


function SharedMemorySegment:read(len)
    -- if no length specified, return the entire buffer
    len = len or self.len

    -- TODO: is this -1 necessary?
    return self.segment_data:sub(self.data_start, self.data_start + len - 1)
end


function SharedMemorySegment:length()
    return self.len
end

