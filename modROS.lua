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


name: modROS.lua
version: 0.0.1
description:
This mod for Farming Simulator 2019 allows autonomous driving of FarmSim vehicles with the ROS navigation stack.
There are three console commands in this mod for now:
The first one is to publish data, the second one is to subscribe data and the last one is to force-center the camera
------------------------------------------------
------------------------------------------------

A: Publishing data
1. sim time publisher - publish the farmsim time
2. odom publisher - publish the odom data from the game
3. laser scan publisher - publish the laser scan data (64 rays)
4. imu publisher - publish the imu data (especially the acc info)
5. TF publisher - publish tf
6. A command for writing all messages to a named pipe: "rosPubMsg true/false"
------------------------------------------------
------------------------------------------------

B. Subscribing data
1. ros_cmd_teleop subscriber - give the vehicle control to ROS'
2. A command for taking over control of a vehicle in the game : "rosControlVehicle true/false"

------------------------------------------------
------------------------------------------------

C. Force-centering the camera
1. A command for force-centering the current camera: "forceCenteredCamera true/false"

------------------------------------------------
------------------------------------------------

author: Ting-Chia Chiang, G.A. vd. Hoorn
maintainer: Ting-Chia Chiang, G.A. vd. Hoorn


--]]


source(Utils.getFilename("lua/json.lua", g_currentModDirectory))
source(Utils.getFilename("lua/shared_memory_segment.lua", g_currentModDirectory))
source(Utils.getFilename("lua/nav_msgs_odometry.lua", g_currentModDirectory))
source(Utils.getFilename("lua/rosgraph_msgs_clock.lua", g_currentModDirectory))
source(Utils.getFilename("lua/sensor_msgs_imu.lua", g_currentModDirectory))
source(Utils.getFilename("lua/sensor_msgs_laser_scan.lua", g_currentModDirectory))
source(Utils.getFilename("lua/tf2_msgs_tf_message.lua", g_currentModDirectory))
source(Utils.getFilename("lua/ros_quaternion.lua", g_currentModDirectory))
source(Utils.getFilename("lua/ros_time.lua", g_currentModDirectory))
source(Utils.getFilename("lua/vehicle_util.lua", g_currentModDirectory))
source(Utils.getFilename("lua/ros_names.lua", g_currentModDirectory))
source(Utils.getFilename("lua/mod_config.lua", g_currentModDirectory))
source(Utils.getFilename("lua/frames.lua", g_currentModDirectory))

local function center_camera_func()
    local camIdx = g_currentMission.controlledVehicle.spec_enterable.camIndex
    local camera = g_currentMission.controlledVehicle.spec_enterable.cameras[camIdx]
    camera.rotX = camera.origRotX
    camera.rotY = camera.origRotY
end

local ModROS = {}
ModROS.modDirectory = g_currentModDirectory

function ModROS:loadMap()
    self.version = g_modManager.nameToMod["modROS"]["version"]
    self.counter = 0
    self.last_read = ""
    self.buf = SharedMemorySegment:init(64)
    -- self.dt = 0
    self.sec = 0
    self.nsec = 0
    self.l_v_x_0 = 0
    self.l_v_y_0 = 0
    self.l_v_z_0 = 0
    -- initial raycast distance
    self.INIT_RAY_DISTANCE = 1000
    -- set a raycast mask for a camera node which enables bits 5(unkown), 6(tractors), 7(combines), 8(trailers), 12(dynamic_objects)
    local RC_MASK_UNKNOWN5 = math.pow(2,  5)
    local RC_MASK_TRACTORS = math.pow(2,  6)
    local RC_MASK_COMBINES = math.pow(2,  7)
    local RC_MASK_TRAILERS = math.pow(2,  8)
    local RC_MASK_DYN_OBJS = math.pow(2, 12)
    self.raycastMask = RC_MASK_UNKNOWN5 + RC_MASK_TRACTORS + RC_MASK_COMBINES + RC_MASK_TRAILERS + RC_MASK_DYN_OBJS

    print("modROS (" .. self.version .. ") loaded")
end

function ModROS:update(dt)
    -- self.dt = self.dt + dt

    -- create TFMessage object
    self.tf_msg = tf2_msgs_TFMessage:init()

    if self.doPubMsg then
        -- avoid writing to the pipe if it isn't actually open
        -- avoid publishing data if one is not inside a vehicle
        if self.file_pipe and g_currentMission.controlledVehicle ~= nil then
            self:publish_sim_time_func()
            self:publish_veh_func()
            self:publish_laser_scan_func()
            self:publish_imu_func()
            self:publish_tf()
        end
    end

    if self.doRosControl then
        self:subscribe_ROScontrol_manned_func(dt)
    end
    if self.doCenterCamera then
        if g_currentMission.controlledVehicle == nil then
            print("You have left your vehicle! Stop force-centering camera")
            self.doCenterCamera = false
        else
            center_camera_func()
        end
    end
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.1 sim_time publisher (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
function ModROS:publish_sim_time_func()
    local msg = rosgraph_msgs_Clock:init()
    msg.clock = ros_time.now()
    self.file_pipe:write(rosgraph_msgs_Clock.ros_msg_name .. "\n" .. msg:to_json())
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.2. odom publisher (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
-- a function to publish get the position and orientaion of unmanned or manned vehicle(s) get and write to the named pipe (symbolic link)
function ModROS:publish_veh_func()
    -- vehicle = g_currentMission.controlledVehicle
    for _, vehicle in pairs(g_currentMission.vehicles) do

        -- legalize a name for ROS
        local vehicle_name = ros_names.sanatize(vehicle:getFullName() .. "_" .. vehicle.id)

        -- retrieve the vehicle node we're interested in
        local veh_node = vehicle.components[1].node

        -- retrieve global (ie: world) coordinates of this node
        local p_x, p_y, p_z = getWorldTranslation(veh_node)

        -- retrieve global (ie: world) quaternion of this node
        local q_x, q_y, q_z, q_w = getWorldQuaternion(veh_node)

        -- get twist data
        local l_v_x, l_v_y, l_v_z = getLocalLinearVelocity(veh_node)
        -- we don't use getAngularVelocity(veh_node) here as the return value is wrt the world frame not local frame


        -- convert y up world to z up world (farmsim coordinate system: x right, z towards me, y up; ROS: y right, x towards me, z up)
        -- https://stackoverflow.com/questions/16099979/can-i-switch-x-y-z-in-a-quaternion
        -- https://gamedev.stackexchange.com/questions/129204/switch-axes-and-handedness-of-a-quaternion
        -- https://stackoverflow.com/questions/18818102/convert-quaternion-representing-rotation-from-one-coordinate-system-to-another

        -- FS time is "frozen" within a single call to update(..), so this
        -- will assign the same stamp to all Odometry messages
        local t = ros_time.now()

        -- create nav_msgs/Odometry instance
        local odom_msg = nav_msgs_Odometry:init()

        -- populate fields (not using Odometry:set(..) here as this is much
        -- more readable than a long list of anonymous args)
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = t
        odom_msg.child_frame_id = vehicle_name
        -- note the order of the axes here (see earlier comment about FS chirality)
        odom_msg.pose.pose.position.x = p_z
        odom_msg.pose.pose.position.y = p_x
        odom_msg.pose.pose.position.z = p_y
        -- note again the order of the axes
        odom_msg.pose.pose.orientation.x = q_z
        odom_msg.pose.pose.orientation.y = q_x
        odom_msg.pose.pose.orientation.z = q_y
        odom_msg.pose.pose.orientation.w = q_w
        -- since the train returns nil when passed to getLocalLinearVelocity, set 0 to prevent an error
        if l_v_x == nil then
            odom_msg.twist.twist.linear.x = 0
            odom_msg.twist.twist.linear.y = 0
            odom_msg.twist.twist.linear.z = 0
        else
        -- note again the order of the axes
            odom_msg.twist.twist.linear.x = l_v_z
            odom_msg.twist.twist.linear.y = l_v_x
            odom_msg.twist.twist.linear.z = l_v_y
        end
        -- TODO get AngularVelocity wrt local vehicle frame
        -- since the farmsim "getAngularVelocity()" can't get body-local angular velocity, we don't set odom_msg.twist.twist.angular for now


        -- serialise to JSON and write to pipe
        self.file_pipe:write(nav_msgs_Odometry.ros_msg_name .. "\n" .. odom_msg:to_json())

        -- get tf from odom to vehicles
        -- setting case_ih_7210_pro_9 as our robot (note: the numbber "_9" might differ depending on your vehicle.xml)
        if vehicle_name == mod_config.controlled_vehicle.base_link_frame_id then
            -- update the transforms_array
            local tf_odom_base_link = geometry_msgs_TransformStamped:init()
            tf_odom_base_link:set("odom", t, "base_link", p_z, p_x, p_y, q_z, q_x, q_y, q_w)
            self.tf_msg:add_transform(tf_odom_base_link)
        else
            -- update the transforms_array
            local tf_odom_vehicle_link = geometry_msgs_TransformStamped:init()
            tf_odom_vehicle_link:set("odom", t, vehicle_name, p_z, p_x, p_y, q_z, q_x, q_y, q_w)
            self.tf_msg:add_transform(tf_odom_vehicle_link)
        end
    end
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.3. laser scan publisher  (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
function ModROS:laser_data_gen(x, y, z, dx_r, dy, dz_r)
    self.raycastDistance = self.INIT_RAY_DISTANCE
    self.object = nil
    raycastClosest(x, y, z, dx_r, dy, dz_r, "raycastCallback", mod_config.laser_scan.range_max, self, self.raycastMask)

    -- push back the self.raycastDistance to self.laser_scan_array table
    -- if  self.raycastDistance is updated which mean there is object detected and raycastCallback is called
    -- otherwise, fill the range with self.INIT_RAY_DISTANCE (1000) (no object detected)

    -- if laser_scan.ignore_terrain is set true then ignore the terrain when detected
    if mod_config.laser_scan.ignore_terrain then
        if self.raycastDistance ~= self.INIT_RAY_DISTANCE and self.raycastTransformId ~= g_currentMission.terrainRootNode then
            -- table.insert(self.laser_scan_array, self.raycastDistance/10)
            table.insert(self.laser_scan_array, self.raycastDistance)
        else
            table.insert(self.laser_scan_array, self.INIT_RAY_DISTANCE)
        end
    else
        if self.raycastDistance ~= self.INIT_RAY_DISTANCE then
            -- table.insert(self.laser_scan_array, self.raycastDistance/10)
            table.insert(self.laser_scan_array, self.raycastDistance)
        else
            table.insert(self.laser_scan_array, self.INIT_RAY_DISTANCE)
        end
    end

end

function ModROS:publish_laser_scan_func()
    local radius = 0.2
    -- cache locally
    local cos = math.cos
    local sin = math.sin
    local LS_FOV = mod_config.laser_scan.angle_max - mod_config.laser_scan.angle_min
    -- calculate nr of steps between rays
    local delta_theta = LS_FOV / (mod_config.laser_scan.num_rays - 1)

    for i = 0, mod_config.laser_scan.num_layers-1 do
        self.laser_scan_array = {}
        -- get the (world) coordinate of each laser scanner's origin: (orig_x, orig_y, orig_z)
        -- "laser_dy" is added between the scanning planes, along +y direction (locally) from the lowest laser scan plane
        -- and all laser scan planes are parallel to each other
        local laser_dy = mod_config.laser_scan.inter_layer_distance * i
        local orig_x, orig_y, orig_z = localToWorld(self.laser_frame_1, 0, laser_dy, 0)

        for j = 0, (mod_config.laser_scan.num_rays - 1) do
            local seg_theta = j * delta_theta
            -- (i_laser_dx, 0 , i_laser_dz) is a local space direction to define the world space raycasting (scanning) direction
            local i_laser_dx = -sin(seg_theta) * radius
            local i_laser_dz = -cos(seg_theta) * radius
            local dx, dy, dz = localDirectionToWorld(self.laser_frame_1, i_laser_dx, 0 , i_laser_dz)
            self:laser_data_gen(orig_x, orig_y, orig_z, dx, dy, dz)
        end

        -- FS time is "frozen" within a single call to update(..), so this
        -- will assign the same stamp to all LaserScan messages
        local t = ros_time.now()

        -- create LaserScan instance
        local scan_msg = sensor_msgs_LaserScan:init()

        -- populate fields (not using LaserScan:set(..) here as this is much
        -- more readable than a long list of anonymous args)
        scan_msg.header.frame_id = "laser_frame_" .. i
        scan_msg.header.stamp = t
        -- with zero angle being forward along the x axis, scanning fov +-180 degrees
        scan_msg.angle_min = mod_config.laser_scan.angle_min
        scan_msg.angle_max = mod_config.laser_scan.angle_max
        scan_msg.angle_increment = LS_FOV / mod_config.laser_scan.num_rays
        -- assume sensor gives 50 scans per second
        scan_msg.time_increment = (1.0 / 50) / mod_config.laser_scan.num_rays
        --scan_msg.scan_time = 0.0  -- we don't set this field (TODO: should we?)
        scan_msg.range_min = mod_config.laser_scan.range_min
        scan_msg.range_max = mod_config.laser_scan.range_max
        scan_msg.ranges = self.laser_scan_array
        --scan_msg.intensities = {}  -- we don't set this field (TODO: should we?)

        -- serialise to JSON and write to pipe
        self.file_pipe:write(sensor_msgs_LaserScan.ros_msg_name .. "\n" .. scan_msg:to_json())

        -- convert to quaternion for ROS TF
        -- note the order of the axes here (see earlier comment about FS chirality)
        -- the rotation from base_link to raycastnode is the same as rotation from raycastnode to virtaul laser_frame_i as there is no rotation between base_link to raycastnode
        local q = ros_quaternion.from_euler(mod_config.laser_scan.laser_transform.rotation.z, mod_config.laser_scan.laser_transform.rotation.x, mod_config.laser_scan.laser_transform.rotation.y)

        -- get the translation from base_link to laser_frame_i
        -- laser_dy is the offset from laser_frame_i to laser_frame_i+1
        local base_to_laser_x, base_to_laser_y, base_to_laser_z = localToLocal(self.laser_frame_1, g_currentMission.controlledVehicle.components[1].node, 0, laser_dy, 0)

        -- create single TransformStamped message
        local tf_base_link_laser_frame_i = geometry_msgs_TransformStamped:init()
        tf_base_link_laser_frame_i:set(
            "base_link",
            t,
            "laser_frame_" .. i,
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
        self.tf_msg:add_transform(tf_base_link_laser_frame_i)
    end
end

function ModROS:raycastCallback(transformId, _, _, _, distance, _, _, _)
    self.raycastDistance = distance
    self.raycastTransformId = transformId
    -- for debugging
    -- self.object = g_currentMission:getNodeObject(self.raycastTransformId)
    -- print(string.format("hitted object is %s",getName(self.raycastTransformId)))
    -- print(string.format("hitted object id is %f",self.raycastTransformId))
    -- print(string.format("self.raycastDistance is %f",self.raycastDistance))
    -- print("v_id ",g_currentMission.controlledVehicle.components[1].node)
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.4. imu publisher (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
-- a function to publish get the position and orientaion of unmanned or manned vehicle(s) get and write to the named pipe (symbolic link)
function ModROS:publish_imu_func()
    local vehicle = g_currentMission.controlledVehicle
    -- for i, vehicle in pairs(g_currentMission.vehicles) do
    -- if self.file_odom == nil then
    -- print("Run ros odom_publisher.py first!! No named pipe created")
    -- self.doPubIMU = false
    -- else

    local filename_t = {}

    for token in string.gmatch(vehicle.configFileName, "[^/]+") do
        table.insert(filename_t, token .. "_" .. vehicle.id)
    end

    -- retrieve the vehicle node we're interested in
    local veh_node = vehicle.components[1].node

    -- retrieve global (ie: world) coordinates of this node
    local q_x, q_y, q_z, q_w = getWorldQuaternion(veh_node)

    -- get twist data and calculate acc info

    -- check getVelocityAtWorldPos and getVelocityAtLocalPos
    -- local linear vel: Get velocity at local position of transform object; "getLinearVelocity" is the the velocity wrt world frame
    -- local l_v_z max is around 8(i guess the unit is m/s here) when reach 30km/hr(shown in speed indicator)
    local l_v_x, l_v_y, l_v_z = getLocalLinearVelocity(veh_node)
    -- we don't use getAngularVelocity(veh_node) here as the return value is wrt the world frame not local frame

    -- TODO add condition to filter out the vehicle: train because it does not have velocity info
    -- for now we'll just use 0.0 as a replacement value
    if not l_v_x then l_v_x = 0.0 end
    if not l_v_y then l_v_y = 0.0 end
    if not l_v_z then l_v_z = 0.0 end

    -- calculation of linear acceleration in x,y,z directions
    local acc_x = (l_v_x - self.l_v_x_0) / (g_currentMission.environment.dayTime / 1000 - self.sec)
    local acc_y = (l_v_y - self.l_v_y_0) / (g_currentMission.environment.dayTime / 1000 - self.sec)
    local acc_z = (l_v_z - self.l_v_z_0) / (g_currentMission.environment.dayTime / 1000 - self.sec)
    -- update the velocity and time
    self.l_v_x_0 = l_v_x
    self.l_v_y_0 = l_v_y
    self.l_v_z_0 = l_v_z
    self.sec = g_currentMission.environment.dayTime / 1000

    -- create sensor_msgs/Imu instance
    local imu_msg = sensor_msgs_Imu:init()

    -- populate fields (not using sensor_msgs_Imu:set(..) here as this is much
    -- more readable than a long list of anonymous args)
    imu_msg.header.frame_id = "base_link"
    imu_msg.header.stamp = ros_time.now()
    -- note the order of the axes here (see earlier comment about FS chirality)
    imu_msg.orientation.x = q_z
    imu_msg.orientation.y = q_x
    imu_msg.orientation.z = q_y
    imu_msg.orientation.w = q_w
    -- TODO get AngularVelocity wrt local vehicle frame
    -- since the farmsim `getAngularVelocity()` can't get body-local angular velocity, we don't set imu_msg.angular_velocity for now

    -- note again the order of the axes
    imu_msg.linear_acceleration.x = acc_z
    imu_msg.linear_acceleration.y = acc_x
    imu_msg.linear_acceleration.z = acc_y

    -- serialise to JSON and write to pipe
    self.file_pipe:write(sensor_msgs_Imu.ros_msg_name .. "\n" .. imu_msg:to_json())

    -- end
    -- end
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.5. TF publisher (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
function ModROS:publish_tf()
    -- serialize tf table into JSON
    local json_format_tf = self.tf_msg:to_json()
    --  publish tf
    self.file_pipe:write(tf2_msgs_TFMessage.ros_msg_name .. "\n" .. json_format_tf)
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- A.6. A command for writing all messages to a named pipe: "rosPubMsg true/false"
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
-- messages publisher console command
addConsoleCommand("rosPubMsg", "write ros messages to named pipe", "rosPubMsg", ModROS)
function ModROS:rosPubMsg(flag)
    if flag ~= nil and flag ~= "" and flag == "true" then
        self.doPubMsg = true
        self.path = ModROS.modDirectory .. "ROS_messages"

        print("connecting to named pipe")
        self.file_pipe = io.open(self.path, "w")

        -- check we could open the pipe
        if self.file_pipe then
            print("Opened '" .. self.path .. "'")
        else
            -- if not, print error to console and return
            print("Could not open named pipe: unknown reason (FS Lua does not seem to provide it)")
            print("Possible reasons:")
            print(" - symbolic link was not created")
            print(" - the 'all_in_one_publisher.py' script is not running")
            return
        end

        -- raycastNode initialization
        local vehicle = g_currentMission.controlledVehicle
        -- if the player is not in the vehicle, print error and return
        if not vehicle then
            print("You are not inside any vehicle, come on! Enter 'e' to hop in one next to you!")
            return
        else
            self.instance_veh = VehicleCamera:new(vehicle, ModROS)
            local xml_path = self.instance_veh.vehicle.configFileName
            local xmlFile = loadXMLFile("vehicle", xml_path)
            -- index 0 is outdoor camera; index 1 is indoor camera
            -- local cameraKey = string.format("vehicle.enterable.cameras.camera(%d)", 0)

            --  get the cameraRaycast node 2(on top of ) which is 0 index .raycastNode(0)
            --  get the cameraRaycast node 3 (in the rear) which is 1 index .raycastNode(1)

            local cameraKey = string.format("vehicle.enterable.cameras.camera(%d).raycastNode(0)", 0)
            XMLUtil.checkDeprecatedXMLElements(xmlFile, xml_path, cameraKey .. "#index", "#node") -- FS17 to FS19
            local camIndexStr = getXMLString(xmlFile, cameraKey .. "#node")
            self.instance_veh.cameraNode =
                I3DUtil.indexToObject(
                self.instance_veh.vehicle.components,
                camIndexStr,
                self.instance_veh.vehicle.i3dMappings
            )
            if self.instance_veh.cameraNode == nil then
                print("nil camera")
            -- else
            --     print(instance_veh.cameraNode)
            end
            -- create self.laser_frame_1 attached to raycastNode (x left, y up, z into the page)
            -- and apply a transform to the self.laser_frame_1
            local tran_x, tran_y, tran_z = mod_config.laser_scan.laser_transform.translation.x, mod_config.laser_scan.laser_transform.translation.y, mod_config.laser_scan.laser_transform.translation.z
            local rot_x, rot_y, rot_z = mod_config.laser_scan.laser_transform.rotation.x, mod_config.laser_scan.laser_transform.rotation.y, mod_config.laser_scan.laser_transform.rotation.z
            self.laser_frame_1 = frames.create_attached_node(self.instance_veh.cameraNode, "self.laser_frame_1", tran_x, tran_y, tran_z, rot_x, rot_y, rot_z)
        end

    elseif flag == nil or flag == "" or flag == "false" then
        self.doPubMsg = false
        print("stop publishing data, set true, if you want to publish Pose")

        if self.file_pipe then
            self.file_pipe:close()
            print("closing named pipe")
        end
    end
end

--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- B.1. ros_cmd_teleop subscriber (TODO:add description)
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]
-- a function to load the ROS joystick state from XML file to take over control of manned vehicle in the game
function ModROS:subscribe_ROScontrol_manned_func(dt)
    if g_currentMission.controlledVehicle == nil then
        print("You have left your vehicle, come on! Please hop in one and type the command again!")
        self.doRosControl = false
    elseif g_currentMission.controlledVehicle ~= nil and self.v_ID ~= nil then
        -- retrieve the first 32 chars from the buffer
        -- note: this does not remove them, it simply copies them
        local buf_read = self.buf:read(64)

        -- print to the game console if what we've found in the buffer is different
        -- from what was there the previous iteration
        -- the counter is just there to make sure we don't see the same line twice
        local allowedToDrive = false
        if buf_read ~= self.last_read and buf_read ~= "" then
            self.last_read = buf_read
            local read_str_list = {}
            -- loop over whitespace-separated components
            for read_str in string.gmatch(self.last_read, "%S+") do
                table.insert(read_str_list, read_str)
            end

            self.acc = tonumber(read_str_list[1])
            self.rotatedTime_param = tonumber(read_str_list[2])
            allowedToDrive = read_str_list[3]
        end

        if allowedToDrive == "true" then
            self.allowedToDrive = true
        elseif allowedToDrive == "false" then
            self.allowedToDrive = false
        end

        vehicle_util.ROSControl(self.vehicle, dt, self.acc, self.allowedToDrive, self.rotatedTime_param)
    end
end


--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- B.2. A command for taking over control of a vehicle in the game : "rosControlVehicle true/false"
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]

-- TODO Allow control of vehicles other than the 'active one'. (the console name has already been changed, but the implementation hasn't yet)

--  console command to take over control of manned vehicle in the game
addConsoleCommand("rosControlVehicle", "let ROS control the current vehicle", "rosControlVehicle", ModROS)
function ModROS:rosControlVehicle(flag)
    if flag ~= nil and flag ~= "" and flag == "true" and g_currentMission.controlledVehicle ~= nil then
        self.vehicle = g_currentMission.controlledVehicle
        self.v_ID = g_currentMission.controlledVehicle.components[1].node
        self.doRosControl = true
        print("start ROS teleoperation")
    elseif g_currentMission.controlledVehicle == nil then
        print("you are not inside any vehicle, come on! Enter 'e' to hop in one next to you!")
    elseif flag == nil or flag == "" or flag == "false" then
        self.doRosControl = false
        print("stop ROS teleoperation")

    -- self.acc = 0
    -- self.rotatedTime_param  = 0
    -- self.allowedToDrive = false
    end
end


--[[
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
-- C.1 A command for force-centering the current camera: "forceCenteredCamera true/false"
------------------------------------------------
------------------------------------------------
------------------------------------------------
------------------------------------------------
--]]

-- centering the camera by setting the camera rotX, rotY to original angles
addConsoleCommand("forceCenteredCamera", "force-center the current camera", "forceCenteredCamera", ModROS)
function ModROS:forceCenteredCamera(flag)
    if flag ~= nil and flag ~= "" and flag == "true" then
        if g_currentMission.controlledVehicle ~= nil then
            print("start centering the camera")
            self.doCenterCamera = true
        else
            print("You have left your vehicle, come on! Please hop in one and type the command again!")
        end
    elseif flag == nil or flag == "" or flag == "false" then
        self.doCenterCamera = false
        print("stop centering the camera")
    end
end

addModEventListener(ModROS)
