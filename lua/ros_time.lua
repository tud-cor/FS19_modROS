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

ros_time = {}
--  function for calculating the simulated time
function ros_time.now()
    -- precision: 48164266.436659 ms with 6 digits after decimal point which is in nanosecond

    local day_time = g_currentMission.environment.dayTime
    local floor = math.floor

    local secs_data = floor(day_time / 1000)
    local nsecs_data = floor((day_time - floor(day_time / 1000) * 1000) * 1e6)

    return {secs = secs_data, nsecs = nsecs_data}
end

return ros_time
