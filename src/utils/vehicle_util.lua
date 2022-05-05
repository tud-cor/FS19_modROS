vehicle_util = {}

function vehicle_util.ROSControl(self, dt, acceleration, allowedToDrive, rotatedTime)
    aiSteeringSpeed=0.0005
    if acceleration ~= nil and rotatedTime ~= nil and allowedToDrive ~= nil then
        if rotatedTime>=0 then
            rotatedTime=math.min(rotatedTime, self.maxRotTime)
        else
            rotatedTime=math.max(rotatedTime, self.minRotTime)
        end
        if rotatedTime>self.rotatedTime then
            self.rotatedTime=math.min(self.rotatedTime+dt*aiSteeringSpeed, rotatedTime)
        else
            self.rotatedTime=math.max(self.rotatedTime-dt*aiSteeringSpeed, rotatedTime)
        end
        -- if self.rotatedTime = 7 turn left
        -- if self.rotatedTime = -7 turn right
        if self.firstTimeRun then
            -- print(self.firstTimeRun)
            local acc = acceleration

            -- set speed limit
            local motor = self.spec_motorized.motor
            motor:setSpeedLimit(25)
            motor:setAccelerationLimit(5)
            -- motor:setMotorRotationAccelerationLimit(150)

            if not allowedToDrive then
                acc = 0
            end
            -- the last argument if set false, the acceleration needs to be 0 before a change of direction is allowed
            vehicle_util.upROSControlledWheels(
                self,
                dt,
                self.lastSpeedReal * self.movingDirection,
                acc,
                not allowedToDrive,
                true
            )
        -- unit of lastSpeedReal m/ms
        end
    end
end

function vehicle_util.upROSControlledWheels(self, dt, currentSpeed, acceleration, doHandbrake, stopAndGoBraking)
    local acceleratorPedal = 0
    local brakePedal = 0
    local reverserDirection = 1
    if self.spec_drivable ~= nil then
        reverserDirection = self.spec_drivable.reverserDirection
        acceleration = acceleration * reverserDirection
    end

    local motor = self.spec_motorized.motor
    local absCurrentSpeed = math.abs(currentSpeed)
    local accSign = MathUtil.sign(acceleration)
    self.nextMovingDirection = Utils.getNoNil(self.nextMovingDirection, 0)

    local automaticBrake = false
    if math.abs(acceleration) < 0.001 then
        automaticBrake = true

        -- Non-stop&go only allows change of direction if the vehicle speed is smaller than 1km/h or the direction has already changed (e.g. because the brakes are not hard enough)
        if stopAndGoBraking or currentSpeed * self.nextMovingDirection < 0.0003 then
            self.nextMovingDirection = 0
        end
    else
        -- Disable the known moving direction if the vehicle is driving more than 5km/h (0.0014 * 3600 =  5.04km/h) in the opposite direction
        if self.nextMovingDirection * currentSpeed < -0.0014 then
            self.nextMovingDirection = 0
        end
        -- Continue accelerating if we want to go in the same direction
        -- or if the vehicle is only moving slowly in the wrong direction (0.0003 * 3600 = 1.08 km/h) and we are allowed to change direction
        if
            accSign == self.nextMovingDirection or
                (currentSpeed * accSign > -0.0003 and (stopAndGoBraking or self.nextMovingDirection == 0))
         then
            acceleratorPedal = acceleration
            brakePedal = 0
            self.nextMovingDirection = accSign
        else
            acceleratorPedal = 0
            brakePedal = math.abs(acceleration)
            if stopAndGoBraking then
                self.nextMovingDirection = accSign
            end
        end
    end
    if automaticBrake then
        acceleratorPedal = 0
    end
    acceleratorPedal = motor:updateGear(acceleratorPedal, dt)

    if motor.gear == 0 and motor.targetGear ~= 0 then
        -- brake automatically if the vehicle is rolling backwards while shifting
        if currentSpeed * MathUtil.sign(motor.targetGear) < 0 then
            automaticBrake = true
        end
    end
    if automaticBrake then
        local isSlow = absCurrentSpeed < motor.lowBrakeForceSpeedLimit
        local isArticulatedSteering =
            self.spec_articulatedAxis ~= nil and self.spec_articulatedAxis.componentJoint ~= nil and
            math.abs(self.rotatedTime) > 0.01
        if (isSlow or doHandbrake) and not isArticulatedSteering then
            brakePedal = 1
        else
            -- interpolate between lowBrakeForce and 1 if speed is below 3.6 km/h
            local factor = math.min(absCurrentSpeed / 0.001, 1)
            brakePedal = MathUtil.lerp(1, motor.lowBrakeForceScale, factor)
        end
    end

    -- this getSmoothedAcceleratorAndBrakePedals doens't work properly, don't use!
    -- acceleratorPedal, brakePedal = WheelsUtil.getSmoothedAcceleratorAndBrakePedals(self, acceleratorPedal, brakePedal, dt)
    -- print(acceleratorPedal, brakePedal)

    local maxSpeed = motor:getMaximumForwardSpeed() * 3.6
    if self.movingDirection < 0 then
        maxSpeed = motor:getMaximumBackwardSpeed() * 3.6
    end
    --active braking if over the speed limit
    local overSpeedLimit = self:getLastSpeed() - math.min(motor:getSpeedLimit(), maxSpeed)
    if overSpeedLimit > 0 then
        brakePedal = math.max(math.min(math.pow(overSpeedLimit + 0.8, 2) - 1, 1), brakePedal) -- start to brake over 0.2km/h
        acceleratorPedal = 0.3 * math.max(1 - overSpeedLimit / 0.2, 0) * acceleratorPedal -- fadeout the accelerator pedal over 0.2km/h, but immediately reduce to 30% (don't set to 0 directly so that the physics engine can still compensate if the brakes are too hard)
    end
    if next(self.spec_motorized.differentials) ~= nil and self.spec_motorized.motorizedNode ~= nil then
        local absAcceleratorPedal = math.abs(acceleratorPedal)
        local minGearRatio, maxGearRatio = motor:getMinMaxGearRatio()
        local maxSpeed
        if maxGearRatio >= 0 then
            maxSpeed = motor:getMaximumForwardSpeed()
        else
            maxSpeed = motor:getMaximumBackwardSpeed()
        end
        local acceleratorPedalControlsSpeed = false
        if acceleratorPedalControlsSpeed then
            maxSpeed = maxSpeed * absAcceleratorPedal
            if absAcceleratorPedal > 0.001 then
                absAcceleratorPedal = 1
            end
        end
        maxSpeed = math.min(maxSpeed, motor:getSpeedLimit() / 3.6)
        local maxAcceleration = motor:getAccelerationLimit()
        local maxMotorRotAcceleration = motor:getMotorRotationAccelerationLimit()
        local minMotorRpm, maxMotorRpm = motor:getRequiredMotorRpmRange()
        local neededPtoTorque = PowerConsumer.getTotalConsumedPtoTorque(self) / motor:getPtoMotorRpmRatio()
        controlVehicle(
            self.spec_motorized.motorizedNode,
            absAcceleratorPedal,
            maxSpeed,
            maxAcceleration,
            minMotorRpm * math.pi / 30,
            maxMotorRpm * math.pi / 30,
            maxMotorRotAcceleration,
            minGearRatio,
            maxGearRatio,
            motor:getMaxClutchTorque(),
            neededPtoTorque
        )
    end
    self:brake(brakePedal)
end
