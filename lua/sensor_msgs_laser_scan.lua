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
sensor_msgs_LaserScan.__index = sensor_msgs_LaserScan

sensor_msgs_LaserScan.ros_msg_name = "sensor_msgs/LaserScan"

function sensor_msgs_LaserScan:init()
    local obj = {}
    setmetatable(obj, sensor_msgs_LaserScan)

    -- fields as defined by sensor_msgs/LaserScan
    obj.header = {
        frame_id = "",
        stamp = {secs = 0, nsecs = 0}
    }
    obj.angle_min = 0.0
    obj.angle_max = 0.0
    obj.angle_increment = 0.0
    obj.time_increment = 0.0
    obj.scan_time = 0.0
    obj.range_min = 0.0
    obj.range_max = 0.0
    obj.ranges = {}
    obj.intensities = {}

    return obj
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
