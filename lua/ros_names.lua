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

ros_names  = {}

-- function for legalizing a name for ROS resources
function ros_names.sanatize(veh_name)
    -- replace whitespace with underscores and uppercase with lowercase to meet the ROS naming conventions for TF
    return veh_name:gsub(" ", "_"):lower()
end

return ros_names
