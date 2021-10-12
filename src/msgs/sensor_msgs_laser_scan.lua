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

-- sensor_msgs_LaserScan class
sensor_msgs_LaserScan = {}
sensor_msgs_LaserScan.ROS_MSG_NAME = "sensor_msgs/LaserScan"

local sensor_msgs_LaserScan_mt = Class(sensor_msgs_LaserScan)

function sensor_msgs_LaserScan.new()
    local self = {}
    setmetatable(self, sensor_msgs_LaserScan_mt)

    -- fields as defined by sensor_msgs/LaserScan
    self.header = {
        frame_id = "",
        stamp = {secs = 0, nsecs = 0}
    }
    self.angle_min = 0.0
    self.angle_max = 0.0
    self.angle_increment = 0.0
    self.time_increment = 0.0
    self.scan_time = 0.0
    self.range_min = 0.0
    self.range_max = 0.0
    self.ranges = {}
    -- self.intensities = {}

    return self
end

function sensor_msgs_LaserScan:delete()
end

function sensor_msgs_LaserScan:set(
    frame_id,
    stamp,
    angle_min,
    angle_max,
    angle_increment,
    time_increment,
    scan_time,
    range_min,
    range_max,
    ranges,
    intensities)
    -- directly overwrite local fields
    self.header = {
        frame_id = frame_id,
        stamp = stamp
    }
    self.angle_min = angle_min
    self.angle_max = angle_max
    self.angle_increment = angle_increment
    self.time_increment = time_increment
    self.scan_time = scan_time
    self.range_min = range_min
    self.range_max = range_max
    self.ranges = ranges
    self.intensities = intensities
end

function sensor_msgs_LaserScan:to_json()
    return json.stringify(self)
end
