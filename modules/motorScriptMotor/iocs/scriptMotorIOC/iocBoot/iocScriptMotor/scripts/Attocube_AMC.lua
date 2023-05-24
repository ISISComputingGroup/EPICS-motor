--[[
	Attocube AMC100 and AMC300 driver
	
	
	Requires the JSON.lua library from https://github.com/rxi/json.lua/blob/master/json.lua . 
	Copy that file into the same scripts directory as this script. Or you can put the JSON
	library in a different folder if you modify the 'package.path' definition below.
	
	This library expects a variable OPEN_LOOP to be defined in the ScriptAxisConfig command
	for each motor, defining whether the motor has an encoder or not. 'YES' is the only
	accepted value for open-loop operation, any other value is parsed as closed-loop.
--]]

asyn = require('asyn')

package.path='./scripts/?.lua'

json = require('json')

IdlePollPeriod = 1.0
MovingPollPeriod = 0.25
ForcedFastPolls = 2

ReadTimeout = 1.0

InTerminator = ''
OutTermintator = ''

function send(request)
	local readback_str = asyn.writeread( json.encode(request), PORT)
	
	--print(readback_str)
	
	local readback = json.decode(readback_str)
	
	if (readback == nil) then
		error("Received no readback")
	end
	
	if (readback.result[1] ~= 0) then
		error(string.format("Error code: %d", readback.result[1]))
	end
	
	return readback
	
end


function move(position, relative, minVel, maxVel, accel)
	if (OPEN_LOOP == "YES") then
		local steps = position
		local old_pos = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_POSITION")
		
		if (not relative) then
			steps = steps - old_pos
		end
		
		asyn.setDoubleParam( DRIVER, AXIS, "MOTOR_POSITION", old_pos + steps )
		
		local backwards = 0
		
		if (steps < 0) then
			steps = math.abs(steps)
			backwards = 1
		end
		
		local request = { 
			jsonrpc = "2.0",
			method = "com.attocube.amc.move.setNSteps",
			params = { AXIS, backwards, steps},
			id = AXIS,
			api = 2 }
	
		send(request)
	else
		local new_pos = position
	
		if (relative) then
			local old_pos = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_POSITION")
			new_pos = new_pos + old_pos
		end
		
		local request = { 
			jsonrpc = "2.0",
			method = "com.attocube.amc.move.setControlTargetPosition",
			params = { AXIS, new_pos },
			id = AXIS,
			api = 2 }
	
		send(request)

		local request = {
                        jsonrpc = "2.0",
                        method = "com.attocube.amc.control.setControlMove",
                        params = { AXIS ,true },
                        id = AXIS,
                        api = 2 }
		send(request)

	end
end

function home(forward)
	local request = { 
		jsonrpc = "2.0",
		method = "com.attocube.amc.move.moveReference",
		params = { AXIS },
		id = AXIS,
		api = 2 }

	send(request)
end

function stop(acceleration)
	local request = { 
		jsonrpc = "2.0",
		method = "com.attocube.amc.control.setControlMove",
		params = { AXIS, false},
		id = AXIS,
		api = 2 }

	send(request)
end

function poll()
	local request = { 
		jsonrpc = "2.0",
		method = "com.attocube.amc.move.getPosition",
		params = { AXIS },
		id = AXIS,
		api = 2 }

	if (OPEN_LOOP ~= "YES") then
		local readback = send(request)
		
		asyn.setDoubleParam( DRIVER, AXIS, "MOTOR_POSITION", readback.result[2] );
		asyn.setDoubleParam( DRIVER, AXIS, "MOTOR_ENCODER_POSITION", readback.result[2] );
	end
	
	request.method = "com.attocube.amc.status.getFullCombinedStatus"
	
	readback = send(request)
	
	local status = readback.result[2]
	
	--print(status)
	
	local moving = 0
	local low_lim = 0
	local high_lim = 0

	if (status == "moving") then moving = 1 end
	if (status == "backward limit reached") then 
		low_lim = 1 
		local request = {
                	jsonrpc = "2.0",
                	method = "com.attocube.amc.control.setControlMove",
                	params = { AXIS, false},
                	id = AXIS,
                	api = 2 }
        	send(request)
	end
	if (status == "forward limit reached") then 
		high_lim = 1 
	
		local request = {
                	jsonrpc = "2.0",
                	method = "com.attocube.amc.control.setControlMove",
                	params = { AXIS, false},
                	id = AXIS,
                	api = 2 }
        	send(request)
	end
	
	asyn.setIntegerParam(DRIVER, AXIS, "MOTOR_STATUS_DONE", moving ~ 1)
	asyn.setIntegerParam(DRIVER, AXIS, "MOTOR_STATUS_MOVING", moving)
	
	asyn.setIntegerParam(DRIVER, AXIS, "MOTOR_STATUS_LOW_LIMIT", low_lim)
	asyn.setIntegerParam(DRIVER, AXIS, "MOTOR_STATUS_HIGH_LIMIT", high_lim)
	
	if (status == "in target range") then
        	local request = {
                        jsonrpc = "2.0",
                        method = "com.attocube.amc.control.setControlMove",
                        params = { AXIS, false},
                        id = AXIS,
                        api = 2 }
		send(request)
	end

			
	return (moving == 1)
end
