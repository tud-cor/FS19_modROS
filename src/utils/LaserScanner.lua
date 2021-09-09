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

LaserScanner = {}

local LaserScanner_mt = Class(LaserScanner)

function LaserScanner.new(vehicle, vehicle_table)
    local self = {}
    setmetatable(self, LaserScanner_mt)

    -- store a reference to the table with settings for this scanner instance
    self.vehicle_table = vehicle_table
    -- store a reference to the vehicle this scanner is attached to
    self.vehicle = vehicle

    -- initial raycast distance
    self.INIT_RAY_DISTANCE = 1000
    -- set a raycast mask for a camera node which enables bits 5(unkown), 6(tractors), 7(combines), 8(trailers), 12(dynamic_objects)
    local RC_MASK_UNKNOWN5 = math.pow(2,  5)
    local RC_MASK_TRACTORS = math.pow(2,  6)
    local RC_MASK_COMBINES = math.pow(2,  7)
    local RC_MASK_TRAILERS = math.pow(2,  8)
    local RC_MASK_DYN_OBJS = math.pow(2, 12)

    -- if provided, use custom collision mask. If not, use default.
    if self.collision_mask then
        self.raycastMask = self.collision_mask
        -- print(("Using custom collision mask for laser scanner: 0x%08X"):format(self.raycastMask))
    else
        self.raycastMask = RC_MASK_UNKNOWN5 + RC_MASK_TRACTORS + RC_MASK_COMBINES + RC_MASK_TRAILERS + RC_MASK_DYN_OBJS
        -- print(("Using default collision mask for laser scanner: 0x%08X"):format(self.raycastMask))
    end

    self.cos = math.cos
    self.sin = math.sin
    self.radius = 0.2
    self.LS_FOV = self.vehicle_table.laser_scan.angle_max - self.vehicle_table.laser_scan.angle_min
    -- calculate nr of steps between rays
    self.delta_theta = self.LS_FOV / (self.vehicle_table.laser_scan.num_rays - 1)

    return self
end


function LaserScanner:getLaserData(laser_scan_array, x, y, z, dx_r, dy, dz_r)
    self.raycastDistance = self.INIT_RAY_DISTANCE
    raycastClosest(x, y, z, dx_r, dy, dz_r, "raycastCallback", self.vehicle_table.laser_scan.range_max, self, self.raycastMask)
    -- push back the self.raycastDistance to self.laser_scan_array table
    -- if  self.raycastDistance is updated which mean there is object detected and raycastCallback is called
    -- otherwise, fill the range with self.INIT_RAY_DISTANCE (1000) (no object detected)
    -- if laser_scan.ignore_terrain is set true then ignore the terrain when detected

    if self.vehicle_table.laser_scan.ignore_terrain then
        if self.raycastDistance ~= self.INIT_RAY_DISTANCE and self.raycastTransformId ~= g_currentMission.terrainRootNode and self.raycastTransformId ~= self.vehicle.components[1].node then
            table.insert(laser_scan_array, self.raycastDistance)
        else
            table.insert(laser_scan_array, self.INIT_RAY_DISTANCE)
        end
    else
        if self.raycastDistance ~= self.INIT_RAY_DISTANCE then
            -- table.insert(self.laser_scan_array, self.raycastDistance/10)
            table.insert(laser_scan_array, self.raycastDistance)
        else
            table.insert(laser_scan_array, self.INIT_RAY_DISTANCE)
        end
    end
end


function LaserScanner:raycastCallback(transformId, _, _, _, distance, _, _, _)
    self.raycastDistance = distance
    self.raycastTransformId = transformId
    -- for debugging
    -- self.object = g_currentMission:getNodeObject(self.raycastTransformId)
    -- print(string.format("hitted object is %s",getName(self.raycastTransformId)))
    -- print(string.format("hitted object id is %f",self.raycastTransformId))
    -- print(string.format("self.raycastDistance is %f",self.raycastDistance))
    -- print("v_id ",g_currentMission.controlledVehicle.components[1].node)
end


function LaserScanner:doScan(ros_time, tf_msg, pub_scan)
    local spec = self.vehicle.spec_rosVehicle

    for i = 0, spec.laser_scan_obj.vehicle_table.laser_scan.num_layers-1 do
        spec.laser_scan_array = {}
        -- get the (world) coordinate of each laser scanner's origin: (orig_x, orig_y, orig_z)
        -- "laser_dy" is added between the scanning planes, along +y direction (locally) from the lowest laser scan plane
        -- and all laser scan planes are parallel to each other
        local laser_dy = spec.laser_scan_obj.vehicle_table.laser_scan.inter_layer_distance * i
        local orig_x, orig_y, orig_z = localToWorld(spec.LaserFrameNode, 0, laser_dy, 0)

        for j = 0, (spec.laser_scan_obj.vehicle_table.laser_scan.num_rays - 1) do
            local seg_theta = j * spec.laser_scan_obj.delta_theta
            -- (i_laser_dx, 0 , i_laser_dz) is a local space direction to define the world space raycasting (scanning) direction
            local i_laser_dx = -spec.laser_scan_obj.sin(seg_theta) * spec.laser_scan_obj.radius
            local i_laser_dz = -spec.laser_scan_obj.cos(seg_theta) * spec.laser_scan_obj.radius
            local dx, dy, dz = localDirectionToWorld(spec.LaserFrameNode, i_laser_dx, 0 , i_laser_dz)
            spec.laser_scan_obj:getLaserData(spec.laser_scan_array, orig_x, orig_y, orig_z, dx, dy, dz)
        end

        -- create LaserScan instance
        local scan_msg = sensor_msgs_LaserScan.new()

        -- populate fields (not using LaserScan:set(..) here as this is much
        -- more readable than a long list of anonymous args)
        scan_msg.header.frame_id = spec.base_link_frame .."/laser_frame_" .. i
        scan_msg.header.stamp = ros_time
        -- with zero angle being forward along the x axis, scanning fov +-180 degrees
        scan_msg.angle_min = spec.laser_scan_obj.vehicle_table.laser_scan.angle_min
        scan_msg.angle_max = spec.laser_scan_obj.vehicle_table.laser_scan.angle_max
        scan_msg.angle_increment = spec.laser_scan_obj.LS_FOV / spec.laser_scan_obj.vehicle_table.laser_scan.num_rays
        -- assume sensor gives 50 scans per second
        scan_msg.time_increment = (1.0 / 50) / spec.laser_scan_obj.vehicle_table.laser_scan.num_rays
        --scan_msg.scan_time = 0.0  -- we don't set this field (TODO: should we?)
        scan_msg.range_min = spec.laser_scan_obj.vehicle_table.laser_scan.range_min
        scan_msg.range_max = spec.laser_scan_obj.vehicle_table.laser_scan.range_max
        scan_msg.ranges = spec.laser_scan_array
        --scan_msg.intensities = {}  -- we don't set this field (TODO: should we?)

        -- publish the message
        pub_scan:publish(scan_msg)

        -- convert to quaternion for ROS TF
        -- note the order of the axes here (see earlier comment about FS chirality)
        -- the rotation from base_link to raycastnode is the same as rotation from raycastnode to virtaul laser_frame_i as there is no rotation between base_link to raycastnode
        local q = ros.Transformations.quaternion_from_euler(spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.z, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.x, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.y)

        -- get the translation from base_link to laser_frame_i
        -- laser_dy is the offset from laser_frame_i to laser_frame_i+1
        local base_to_laser_x, base_to_laser_y, base_to_laser_z = localToLocal(spec.LaserFrameNode, self.vehicle.components[1].node, 0, laser_dy, 0)

        -- create single TransformStamped message
        local tf_base_link_laser_frame_i = geometry_msgs_TransformStamped.new()
        tf_base_link_laser_frame_i:set(
            spec.base_link_frame,
            ros_time,
            spec.base_link_frame .."/laser_frame_" .. i,
            -- note the order of the axes here (see earlier comment about FS chirality)
            base_to_laser_z,
            base_to_laser_x,
            base_to_laser_y,
            -- we don't need to swap the order of q, since the calculation of q is based on the ROS chirality
            q[1],
            q[2],
            q[3],
            q[4]
        )
        spec:addTF(tf_msg, tf_base_link_laser_frame_i)
    end
end
