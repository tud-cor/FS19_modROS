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

local directory = g_currentModDirectory
local modName = g_currentModName

g_modROSModDirectory = directory
g_modROSModName = modName

-- ROS 'messages'
source(Utils.getFilename("src/msgs/nav_msgs_odometry.lua", directory))
source(Utils.getFilename("src/msgs/rosgraph_msgs_clock.lua", directory))
source(Utils.getFilename("src/msgs/sensor_msgs_imu.lua", directory))
source(Utils.getFilename("src/msgs/sensor_msgs_laser_scan.lua", directory))
source(Utils.getFilename("src/msgs/tf2_msgs_tf_message.lua", directory))

-- third party utilities
source(Utils.getFilename("src/utils/json.lua", directory))
source(Utils.getFilename("src/utils/shared_memory_segment.lua", directory))
source(Utils.getFilename("src/utils/vehicle_util.lua", directory))

-- our own utilities
source(Utils.getFilename("src/utils/frames.lua", directory))
source(Utils.getFilename("src/utils/ros.lua", directory))

-- main/default configuration
-- TODO: add UI for all of this
source(Utils.getFilename("src/mod_config.lua", directory))

-- main file
source(Utils.getFilename("src/modROS.lua", directory))
