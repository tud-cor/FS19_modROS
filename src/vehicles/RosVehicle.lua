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
-- source(Utils.getFilename("src/modROS.lua", g_modROSModDirectory))

RosVehicle = {}

-- RosVehicle.MOD_DIR = g_modROSModDirectory
-- RosVehicle.MOD_NAME = g_modROSModName


function RosVehicle.prerequisitesPresent(specializations)
    -- make <drivable> spec as a prerequisite for now so that we only control vehicles which are drivable
    return SpecializationUtil.hasSpecialization(Drivable, specializations)
end


function RosVehicle.initSpecialization()
end


function RosVehicle.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "addTF", RosVehicle.addTF)
    SpecializationUtil.registerFunction(vehicleType, "getLaserFrameNode", RosVehicle.getLaserFrameNode)
    SpecializationUtil.registerFunction(vehicleType, "pubImu", RosVehicle.pubImu)
    SpecializationUtil.registerFunction(vehicleType, "pubLaserScan", RosVehicle.pubLaserScan)
    SpecializationUtil.registerFunction(vehicleType, "pubOdom", RosVehicle.pubOdom)
end


function RosVehicle.registerEventListeners(vehicleType)
    SpecializationUtil.registerEventListener(vehicleType, "onLoad", RosVehicle)
    -- SpecializationUtil.registerEventListener(vehicleType, "onUpdate", RosVehicle)
end


-- -- rosVehicle spec own update
-- function RosVehicle:onUpdate(dt)

-- end


-- this will be loaded for every rosVehicle vehicle before starting the game
function RosVehicle:onLoad()
    -- rosVehicle namespace
    self.spec_rosVehicle = self["spec_" .. g_modROSModName .. ".rosVehicle"]
    local spec = self.spec_rosVehicle
    -- rosVehicle variables

    -- there are two veh_name variables, one is with an id number and the other is without
    -- because there could be situations where the same vehicle model is spawned more than once in the FS world
    -- Hence, to create unique topic names for each vehicle, the name with id is necessary

    -- variables without id are used as keys to retrieve config settings/tables
    -- .id is initialized by FS
    spec.ros_veh_name_with_id = ros.Names.sanatize(self:getFullName() .. "_" .. self.id)
    spec.ros_veh_name = ros.Names.sanatize(self:getFullName())
    spec.base_link_frame = spec.ros_veh_name_with_id .. "/base_link"

    -- initialize linear velocity and time(s) for Imu messages
    spec.l_v_x_0 = 0
    spec.l_v_y_0 = 0
    spec.l_v_z_0 = 0
    spec.sec = 0

    -- store the individual vehicle config file in the specialization to avoid reloading in the loop
    spec.instance_veh = VehicleCamera:new(self, RosVehicle)
    spec.xml_path = spec.instance_veh.vehicle.configFileName
    spec.xmlFile = loadXMLFile("vehicle", spec.xml_path)

    -- initialize publishers for Odometry, LaserScan and Imu messages for each rosVehicle
    spec.pub_odom = Publisher.new(ModROS._conx, spec.ros_veh_name_with_id .."/odom", nav_msgs_Odometry)
    spec.pub_scan = Publisher.new(ModROS._conx, spec.ros_veh_name_with_id .."/scan", sensor_msgs_LaserScan)
    spec.pub_imu = Publisher.new(ModROS._conx, spec.ros_veh_name_with_id .."/imu", sensor_msgs_Imu)
end


function RosVehicle:pubOdom(ros_time, tf_msg)

    local spec = self.spec_rosVehicle
    local vehicle_base_link = spec.base_link_frame

    -- retrieve the vehicle node we're interested in:
    -- A drivable vehicle is often composed of 2 components and the first component is the main part.
    -- The second component is mostly the axis object. Hence we take component1 as our vehicle
    -- The components can be checked/viewed in each vehicle's 3D model.
    local veh_node = self.components[1].node

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


    -- create nav_msgs/Odometry instance
    local odom_msg = nav_msgs_Odometry.new()

    -- populate fields (not using Odometry:set(..) here as this is much
    -- more readable than a long list of anonymous args)
    odom_msg.header.frame_id = "odom"
    odom_msg.header.stamp = ros_time
    odom_msg.child_frame_id = vehicle_base_link
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
    -- publish the message
    spec.pub_odom:publish(odom_msg)

    -- get tf from odom to vehicles
    local tf_odom_vehicle_link = geometry_msgs_TransformStamped.new()
    tf_odom_vehicle_link:set("odom", ros_time, vehicle_base_link, p_z, p_x, p_y, q_z, q_x, q_y, q_w)
    -- update the transforms_array
    self:addTF(tf_msg, tf_odom_vehicle_link)

end


function RosVehicle:addTF(tf_msg, TransformStamped)
    tf_msg:add_transform(TransformStamped)
end


function RosVehicle:pubLaserScan(ros_time, tf_msg)

    local spec = self.spec_rosVehicle
    -- make sure the vechile is drivable and has laser frame node
    if self:getLaserFrameNode() then
        spec.laser_scan_obj:doScan(ros_time, tf_msg)
    end
end


function RosVehicle:getLaserFrameNode()
    local spec = self.spec_rosVehicle

    if self.spec_drivable then

        -- if there is no custom laser scanner setting for this vehicle, use the default settings to initialize an object of LaserScanner class
        -- note: a laser scanner is always mounted in the default settings
        if not mod_config.vehicle[spec.ros_veh_name] then
            spec.laser_scan_obj = LaserScanner.new(self, mod_config.vehicle["default_vehicle"])
        -- if the custom laser scanner is mounted (parameter enabled = true), initialize an object of LaserScanner class
        elseif mod_config.vehicle[spec.ros_veh_name].laser_scan.enabled then
            spec.laser_scan_obj = LaserScanner.new(self, mod_config.vehicle[spec.ros_veh_name])
        -- if the custom laser scanner is not mounted (parameter enabled = false), do not initialize an object of LaserScanner class
        else
            return
        end
        --  get the cameraRaycast node 2(on top of ) which is 0 index .raycastNode(0)
        --  get the cameraRaycast node 3 (in the rear) which is 1 index .raycastNode(1)
        local cameraKey = string.format("vehicle.enterable.cameras.camera(%d).%s", 0, spec.laser_scan_obj.vehicle_table.laser_scan.laser_attachments)
        XMLUtil.checkDeprecatedXMLElements(spec.xmlFile, spec.xml_path, cameraKey .. "#index", "#node") -- FS17 to FS19
        local camIndexStr = getXMLString(spec.xmlFile, cameraKey .. "#node")
        spec.instance_veh.cameraNode =
            I3DUtil.indexToObject(
            spec.instance_veh.vehicle.components,
            camIndexStr,
            spec.instance_veh.vehicle.i3dMappings
        )
        if spec.instance_veh.cameraNode == nil then
            print("nil camera")
        -- else
        --     print(instance_veh.cameraNode)
        end
        -- create self.laser_frame_1 attached to raycastNode (x left, y up, z into the page)
        -- and apply a transform to the self.laser_frame_1
        local tran_x, tran_y, tran_z = spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.x, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.y, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.z
        local rot_x, rot_y, rot_z = spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.x, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.y, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.z
        local laser_frame_1 = frames.create_attached_node(spec.instance_veh.cameraNode, self:getFullName(), tran_x, tran_y, tran_z, rot_x, rot_y, rot_z)
        spec.LaserFrameNode = laser_frame_1
        return spec.LaserFrameNode
    else
        return nil
    end
end


function RosVehicle:pubImu(ros_time)

    local spec = self.spec_rosVehicle
    -- if there are no custom settings for this vehicle, use the default settings
    if not mod_config.vehicle[spec.ros_veh_name] then
        spec.imu = mod_config.vehicle["default_vehicle"].imu.enabled
    else
        spec.imu = mod_config.vehicle[spec.ros_veh_name].imu.enabled
    end

    -- if imu of this vehicle is disabled, return
    if not spec.imu then return end

    -- retrieve the vehicle node we're interested in
    local veh_node = self.components[1].node

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
    local acc_x = (l_v_x - spec.l_v_x_0) / (g_currentMission.environment.dayTime / 1000 - spec.sec)
    local acc_y = (l_v_y - spec.l_v_y_0) / (g_currentMission.environment.dayTime / 1000 - spec.sec)
    local acc_z = (l_v_z - spec.l_v_z_0) / (g_currentMission.environment.dayTime / 1000 - spec.sec)
    -- update the linear velocity and time
    spec.l_v_x_0 = l_v_x
    spec.l_v_y_0 = l_v_y
    spec.l_v_z_0 = l_v_z
    spec.sec = g_currentMission.environment.dayTime / 1000


    -- create sensor_msgs/Imu instance
    local imu_msg = sensor_msgs_Imu.new()
    -- populate fields (not using sensor_msgs_Imu:set(..) here as this is much
    -- more readable than a long list of anonymous args)
    imu_msg.header.frame_id = "base_link"
    imu_msg.header.stamp = ros_time
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

    -- publish the message
    spec.pub_imu:publish(imu_msg)

end
