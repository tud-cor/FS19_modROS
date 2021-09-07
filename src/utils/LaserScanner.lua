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
            -- table.insert(self.laser_scan_array, self.raycastDistance/10)
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
