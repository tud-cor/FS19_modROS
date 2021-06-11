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

WriteOnlyFileConnection = {}

local WriteOnlyFileConnection_mt = Class(WriteOnlyFileConnection)


function WriteOnlyFileConnection.new(uri)
    local self = {}
    setmetatable(self, WriteOnlyFileConnection_mt)

    self._uri = uri
    self._fd = nil

    return self
end

function WriteOnlyFileConnection:delete()
    self:close()
end

function WriteOnlyFileConnection:get_uri()
    return self._uri
end

function WriteOnlyFileConnection:_open()
    -- NOTE: this is not guarded in any way
    -- TODO: allow specifying flags
    self._fd = io.open(self._uri, "w")
    if not self._fd then
        return nil, "Could not open uri: unknown reason (FS Lua does not provide one)"
    end
    return true
end

function WriteOnlyFileConnection:_close()
    -- NOTE: this is not guarded in any way
    self._fd:close()
    self._fd = nil
    return true
end

function WriteOnlyFileConnection:connect()
    -- only connect if we're not already connected
    if self:is_connected() then
        return nil, "Already connected"
    end

    local ret, err = self:_open()
    if not ret then
        return nil, "Could not connect: '" .. err .. "'"
    end
    return ret
end

function WriteOnlyFileConnection:disconnect()
    -- only disconnect if we're actually connected
    if not self:is_connected() then
        return nil, "Not connected"
    end

    local ret, err = self:_close()
    if not ret then
        return nil, "Could not disconnect: '" .. err .. "'"
    end
    return ret
end

function WriteOnlyFileConnection:get_fd()
    return self._fd
end

function WriteOnlyFileConnection:is_connected()
    --TODO: find a better way to figure out whether a file is still open.
    -- io.type(..) does not appear to be supported :(
    return (self._fd ~= nil)
end

function WriteOnlyFileConnection:write(data)
    if not self:is_connected() then
        return nil, "Not connected"
    end
    -- does write() return anything?
    self._fd:write(data)
    return true
end
