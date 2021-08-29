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

A. Shared bits of information/infrastructure with RosVehicle spec
1. getPublisher - instantiate publishers for each spec instance, called from RosVehicle.lua

------------------------------------------------
------------------------------------------------

B: Publishing data
1. sim time publisher - publish in-game simulated clock. This message stops being published when the game is paused/exited
2. odom publisher - publish ground-truth Pose and Twist of vehicles based on their in-game position and orientation
3. laser scan publisher - publish the laser scan data
4. imu publisher - publish the imu data (especially the acc info)
5. A command for writing all messages to a named pipe: "rosPubMsg true/false"

------------------------------------------------
------------------------------------------------

C. Subscribing data
1. ros_cmd_teleop subscriber - give the vehicle control to ROS
2. A command for taking over control of a vehicle in the game : "rosControlVehicle true/false"

------------------------------------------------
------------------------------------------------

D. Force-centering the camera
1. A command for force-centering the current camera: "forceCenteredCamera true/false"

------------------------------------------------
------------------------------------------------

author: Ting-Chia Chiang, G.A. vd. Hoorn
maintainer: Ting-Chia Chiang, G.A. vd. Hoorn


--]]

-- ModROS class is not local here
-- it's now in the global scope so the installSpecializations(..) can be called in loader.lua
ModROS = {}
ModROS.MOD_DIR = g_modROSModDirectory
ModROS.MOD_NAME = g_modROSModName
ModROS.MOD_VERSION = g_modManager.nameToMod[ModROS.MOD_NAME]["version"]

local function center_camera_func()
    local camIdx = g_currentMission.controlledVehicle.spec_enterable.camIndex
    local camera = g_currentMission.controlledVehicle.spec_enterable.cameras[camIdx]
    camera.rotX = camera.origRotX
    camera.rotY = camera.origRotY
end

function ModROS:loadMap()
    self.last_read = ""
    self.buf = SharedMemorySegment:init(64)
    self.sec = 0
    self.nsec = 0
    self.l_v_x_0 = 0
    self.l_v_y_0 = 0
    self.l_v_z_0 = 0

    -- initialise connection to the Python side (but do not connect it yet)
    self.path = ModROS.MOD_DIR .. "ROS_messages"
    self._conx = WriteOnlyFileConnection.new(self.path)

    -- initialise publishers
    self._pub_tf = Publisher.new(self._conx, "tf", tf2_msgs_TFMessage)
    self._pub_clock = Publisher.new(self._conx, "clock", rosgraph_msgs_Clock)
    self._pub_odom = Publisher.new(self._conx, "odom", nav_msgs_Odometry)
    self._pub_scan = Publisher.new(self._conx, "scan", sensor_msgs_LaserScan)
    self._pub_imu = Publisher.new(self._conx, "imu", sensor_msgs_Imu)

    print("modROS (" .. ModROS.MOD_VERSION .. ") loaded")
end

function ModROS.installSpecializations(vehicleTypeManager, specializationManager, modDirectory, modName)
    specializationManager:addSpecialization("rosVehicle", "RosVehicle", Utils.getFilename("src/vehicles/RosVehicle.lua", modDirectory), nil) -- Nil is important here

    for typeName, _ in pairs(vehicleTypeManager:getVehicleTypes()) do
        vehicleTypeManager:addSpecialization(typeName, modName .. ".rosVehicle")
    end

end

function ModROS:update(dt)
    -- create TFMessage object
    self.tf_msg = tf2_msgs_TFMessage.new()

    if self.doPubMsg then
        -- avoid writing to the pipe if it isn't actually open
        -- avoid publishing data if one is not inside a vehicle
        if self._conx:is_connected() and g_currentMission.controlledVehicle ~= nil then
            self:publish_sim_time_func()
            self:publish_veh_odom_func()
            self:publish_laser_scan_func()
            self:publish_imu_func()
            self._pub_tf:publish(self.tf_msg)

        end
    end

    if self.doRosControl then
        self:subscribe_ROScontrol_func(dt)
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


-- -- A.1 getPublisher - instantiate publishers for each spec instance, called from RosVehicle.lua
-- function ModROS:getPublisher(topic_name, topic_type)
--     local pub = Publisher.new(self._conx, topic_name, topic_type)
--     return pub
-- end


-- B.1 sim_time publisher: publish the farmsim time
function ModROS:publish_sim_time_func()
    local msg = rosgraph_msgs_Clock.new()
    msg.clock = ros.Time.now()
    self._pub_clock:publish(msg)
end


-- B.2. odom publisher
-- a function to publish ground-truth Pose and Twist of all vehicles (including non-drivable) based on their in-game position and orientation
function ModROS:publish_veh_odom_func()
    -- FS time is "frozen" within a single call to update(..), so this
    -- will assign the same stamp to all Odometry messages
    local ros_time = ros.Time.now()
    for _, vehicle in pairs(g_currentMission.vehicles) do
        vehicle:pubOdom(ros_time, self.tf_msg, self._pub_odom)
    end
end


-- B.3. laser scan publisher
function ModROS:publish_laser_scan_func()
    -- FS time is "frozen" within a single call to update(..), so this
    -- will assign the same stamp to all LaserScan messages
    local ros_time = ros.Time.now()
    if mod_config.control_only_active_one then
        local vehicle = g_currentMission.controlledVehicle
        vehicle:pubLaserScan(ros_time, self.tf_msg, self._pub_scan)
    else
        for _, vehicle in pairs(g_currentMission.vehicles) do
            vehicle:pubLaserScan(ros_time, self.tf_msg, self._pub_scan)
        end
    end
end


-- B.4. imu publisher
-- a function to publish get the position and orientaion of unmanned or manned vehicle(s) get and write to the named pipe (symbolic link)
function ModROS:publish_imu_func()
    if mod_config.control_only_active_one then
        local vehicle = g_currentMission.controlledVehicle
        local ros_time = ros.Time.now()
        vehicle:pubImu(ros_time, self._pub_imu)
    else
        for _, vehicle in pairs(g_currentMission.vehicles) do
            local ros_time = ros.Time.now()
            vehicle:pubImu(ros_time, self._pub_imu)
        end
    end
end

-- B.5. Console command allows to toggle publishers on/off: "rosPubMsg true/false"
-- messages publisher console command
addConsoleCommand("rosPubMsg", "write ros messages to named pipe", "rosPubMsg", ModROS)
function ModROS:rosPubMsg(flag)
    if flag ~= nil and flag ~= "" and flag == "true" then

        if not self._conx:is_connected() then
            print("connecting to named pipe")
            local ret, err = self._conx:connect()
            if ret then
                print("Opened '" .. self._conx:get_uri() .. "'")
            else
                -- if not, print error to console and return
                print(("Could not connect: %s"):format(err))
                print("Possible reasons:")
                print(" - symbolic link was not created")
                print(" - the 'all_in_one_publisher.py' script is not running")
                return
            end
        end
        -- initialisation was successful
        self.doPubMsg = true

    elseif flag == nil or flag == "" or flag == "false" then
        self.doPubMsg = false
        print("stop publishing data, set true, if you want to publish Pose")

        local ret, err = self._conx:disconnect()
        if not ret then
            print(("Could not disconnect: %s"):format(err))
        else
            print("Disconnected")
        end
    end
end


-- C.1. ros_cmd_teleop subscriber
-- a function to input the ROS geometry_msgs/Twist into the game to take over control of all vehicles
function ModROS:subscribe_ROScontrol_func(dt)
    for _, vehicle in pairs(g_currentMission.vehicles) do
        if vehicle.spec_drivable then
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

            vehicle_util.ROSControl(vehicle, dt, self.acc, self.allowedToDrive, self.rotatedTime_param)
        end
    end
end



-- C.2. A command for taking over control of a vehicle in the game : "rosControlVehicle true/false"

-- TODO Allow control of vehicles other than the 'active one'. (the console name has already been changed, but the implementation hasn't yet)

--  console command to take over control of all vehicles in the game
addConsoleCommand("rosControlVehicle", "let ROS control the current vehicle", "rosControlVehicle", ModROS)
function ModROS:rosControlVehicle(flag)
    if flag ~= nil and flag ~= "" and flag == "true" then
        self.doRosControl = true
        print("start ROS teleoperation")
    elseif flag == nil or flag == "" or flag == "false" then
        self.doRosControl = false
        print("stop ROS teleoperation")
    end
end


-- D.1 A command for force-centering the current camera: "forceCenteredCamera true/false"
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
