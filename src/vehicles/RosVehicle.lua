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


function RosVehicle.prerequisitesPresent()
    return true
end


function RosVehicle.initSpecialization()
end


function RosVehicle.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "addTF", RosVehicle.addTF)
    SpecializationUtil.registerFunction(vehicleType, "fillLaserData", RosVehicle.fillLaserData)
    SpecializationUtil.registerFunction(vehicleType, "getLaserFrameNode", RosVehicle.getLaserFrameNode)
    SpecializationUtil.registerFunction(vehicleType, "pubOdom", RosVehicle.pubOdom)

end


function RosVehicle.registerEventListeners(vehicleType)
    SpecializationUtil.registerEventListener(vehicleType, "onLoad", RosVehicle)
    -- SpecializationUtil.registerEventListener(vehicleType, "onUpdate", RosVehicle)
end


-- -- rosVehicle spec own update
-- function RosVehicle:onUpdate(dt)

-- end


function RosVehicle:onLoad()
    -- rosVehicle namespace
    self.spec_rosVehicle = self["spec_" .. g_modROSModName .. ".rosVehicle"]
    local spec = self.spec_rosVehicle
    -- rosVehicle variables
    spec.ros_veh_name = ros.Names.sanatize(self:getFullName() .. "_" .. self.id)
    spec.base_link_frame = spec.ros_veh_name .. "/base_link"

end


function RosVehicle:pubOdom(ros_time, tf_msg, pub_odom)

    local spec = self.spec_rosVehicle

    -- legalize a name for ROS
    -- local vehicle_name = ros.Names.sanatize(vehicle:getFullName() .. "_" .. vehicle.id)
    local vehicle_name = spec.base_link_frame


    -- retrieve the vehicle node we're interested in
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

    -- FS time is "frozen" within a single call to update(..), so this
    -- will assign the same stamp to all Odometry messages
    local t = ros_time

    -- create nav_msgs/Odometry instance
    local odom_msg = nav_msgs_Odometry.new()

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
    -- publish the message
    pub_odom:publish(odom_msg)

    -- get tf from odom to vehicles
    local tf_odom_vehicle_link = geometry_msgs_TransformStamped.new()
    tf_odom_vehicle_link:set("odom", t, vehicle_name, p_z, p_x, p_y, q_z, q_x, q_y, q_w)
    -- update the transforms_array
    self:addTF(tf_msg, tf_odom_vehicle_link)

end


function RosVehicle:addTF(tf_msg, TransformStamped)
    tf_msg:add_transform(TransformStamped)
end


function RosVehicle:fillLaserData(ros_time, tf_msg, pub_scan)

    local spec = self.spec_rosVehicle
    -- make sure the vechile is drivable and has laser frame node
    if self:getLaserFrameNode() then
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

            -- FS time is "frozen" within a single call to update(..), so this
            -- will assign the same stamp to all LaserScan messages
            local t = ros_time

            -- create LaserScan instance
            local scan_msg = sensor_msgs_LaserScan.new()

            -- populate fields (not using LaserScan:set(..) here as this is much
            -- more readable than a long list of anonymous args)
            scan_msg.header.frame_id = spec.base_link_frame .."/laser_frame_" .. i
            scan_msg.header.stamp = t
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
            local base_to_laser_x, base_to_laser_y, base_to_laser_z = localToLocal(spec.LaserFrameNode, self.components[1].node, 0, laser_dy, 0)

            -- create single TransformStamped message
            local tf_base_link_laser_frame_i = geometry_msgs_TransformStamped.new()
            tf_base_link_laser_frame_i:set(
                spec.base_link_frame,
                t,
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
            self:addTF(tf_msg, tf_base_link_laser_frame_i)
        end
    end
end


function RosVehicle:getLaserFrameNode()
    local spec = self.spec_rosVehicle

    if self.spec_drivable then
        local current_veh_name = ros.Names.sanatize(self:getFullName())
        spec.laser_scan_obj = LaserScanner.new(self, mod_config.vehicle[current_veh_name])

        local instance_veh = VehicleCamera:new(self, RosVehicle)
        local xml_path = instance_veh.vehicle.configFileName
        local xmlFile = loadXMLFile("vehicle", xml_path)
        --  get the cameraRaycast node 2(on top of ) which is 0 index .raycastNode(0)
        --  get the cameraRaycast node 3 (in the rear) which is 1 index .raycastNode(1)
        local cameraKey = string.format("vehicle.enterable.cameras.camera(%d).%s", 0, spec.laser_scan_obj.vehicle_table.laser_scan.laser_attachments)
        XMLUtil.checkDeprecatedXMLElements(xmlFile, xml_path, cameraKey .. "#index", "#node") -- FS17 to FS19
        local camIndexStr = getXMLString(xmlFile, cameraKey .. "#node")
        instance_veh.cameraNode =
            I3DUtil.indexToObject(
            instance_veh.vehicle.components,
            camIndexStr,
            instance_veh.vehicle.i3dMappings
        )
        if instance_veh.cameraNode == nil then
            print("nil camera")
        -- else
        --     print(instance_veh.cameraNode)
        end
        -- create self.laser_frame_1 attached to raycastNode (x left, y up, z into the page)
        -- and apply a transform to the self.laser_frame_1
        local tran_x, tran_y, tran_z = spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.x, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.y, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.translation.z
        local rot_x, rot_y, rot_z = spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.x, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.y, spec.laser_scan_obj.vehicle_table.laser_scan.laser_transform.rotation.z
        local laser_frame_1 = frames.create_attached_node(instance_veh.cameraNode, self:getFullName(), tran_x, tran_y, tran_z, rot_x, rot_y, rot_z)
        spec.LaserFrameNode = laser_frame_1
        return spec.LaserFrameNode
    else
        return nil
    end
end
