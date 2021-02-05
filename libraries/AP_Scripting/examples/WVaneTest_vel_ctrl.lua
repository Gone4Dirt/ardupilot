
local mission_started = false
local CMD_LOITER_TIME = 19
local MODE_AUTO = 3
local MODE_GUIDED = 4
local vel_cmd_start_ms = 0
local guided_cmds_complete = false
local guided_cmds_count = 1

local function read_mission(file_name)

    -- Open file
    file = assert(io.open(file_name), 'Could open :' .. file_name)
  
    -- check header
    assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')
  
    -- clear any existing mission
    assert(mission:clear(), 'Could not clear current mission')
  
    -- read each line and write to mission
    local item = mavlink_mission_item_int_t()
    local index = 0
    while true do
  
      local data = {}
      for i = 1, 12 do
        data[i] = file:read('n')
        if data[i] == nil then
          if i == 1 then
            gcs:send_text(6, 'loaded mission: ' .. file_name)
            return -- got to the end of the file
          else
            mission:clear() -- clear part loaded mission
            error('failed to read file')
          end
        end
      end
  
      item:seq(data[1])
      item:frame(data[3])
      item:command(data[4])
      item:param1(data[5])
      item:param2(data[6])
      item:param3(data[7])
      item:param4(data[8])
      item:x(data[9]*10^7)
      item:y(data[10]*10^7)
      item:z(data[11])
  
      if not mission:set_item(index,item) then
        mission:clear() -- clear part loaded mission
        error(string.format('failed to set mission item %i',index))
      end
      index = index + 1
    end
  
  end


  function set_param(param_name,value)
    if param:set(param_name,value) then
        gcs:send_text(6, param_name .. ' set')
    else
        gcs:send_text(6, param_name .. ' set fail')
    end
  end


  function setup()
    -- setup parameters
    set_param('AUTO_OPTIONS',15)
    set_param('GUID_OPTIONS',8)
    set_param('WVANE_ENABLE',1)
    set_param('WVANE_GAIN',0.5)
    set_param('WVANE_MIN_ANG',1)

    -- load mission
    read_mission('DeliveryMisson.waypoints')
    return update, 3000

  end


  function update()

    -- Arm and start mission
    if (not mission_started) and ahrs:healthy() and (gps:status(0)>=3) then
        -- mission not started, ekf is healthy and we have 3d gps fix
        if vehicle:set_mode(MODE_AUTO) and arming:arm() then
            gcs:send_text(6, 'Mission started')
            mission_started = true
        end
    end

    -- Whilst in mission check for first loiter time cmd to signify were at the drop location
    if (vehicle:get_mode() == MODE_AUTO) and mission_started and (mission:get_current_nav_id() == CMD_LOITER_TIME) and (not guided_cmds_complete) then
        vehicle:set_mode(MODE_GUIDED)
    end


    if (vehicle:get_mode() == MODE_GUIDED) and (not guided_cmds_complete) then
        now = millis()
        local cmd_times = {10000, 20000, 5000, 20000}

        local vel_cmd_status = false
        if not guided_cmds_complete then
            -- issue velocity control command
            local target_vel = Vector3f()
            if guided_cmds_count == 1 then
                target_vel:x(5)
                target_vel:y(5)
                target_vel:z(0)
            end
            if guided_cmds_count == 2 then
                target_vel:x(0)
                target_vel:y(0)
                target_vel:z(0)
            end
            if guided_cmds_count == 3 then
                target_vel:x(0)
                target_vel:y(10)
                target_vel:z(0)
            end
            if guided_cmds_count == 4 then
                target_vel:x(0)
                target_vel:y(0)
                target_vel:z(0)
            end

            vel_cmd_status = vehicle:set_target_velocity_NED(target_vel)

        end

        if (vel_cmd_start_ms > 0) and (now - vel_cmd_start_ms > cmd_times[guided_cmds_count]) then
            -- Advance cmds count
            guided_cmds_count = guided_cmds_count+1
            gcs:send_text(6, 'DB:Guided cmd no ' .. tostring(guided_cmds_count))

            -- Reset timer
            vel_cmd_start_ms = 0

            if guided_cmds_count > 4 then
                guided_cmds_complete = true
                gcs:send_text(6, 'Drop complete')
            end
        end

        -- start timer
        if vel_cmd_status and (vel_cmd_start_ms == 0) then
            gcs:send_text(6, 'DB:Velocity cmd 1st issue')
            vel_cmd_start_ms = now
        end

    end

    -- Drop complete, advance mission and set back to auto
    if guided_cmds_complete then
        -- advance mission index
        local next_cmd = mission:get_current_nav_index()+1
        if mission:set_current_cmd(next_cmd) then
            vehicle:set_mode(MODE_AUTO)
            gcs:send_text(6, 'DB:In Auto cmd ' .. tostring(next_cmd))
            return
        end
    end

    return update, 2000

  end
  
  return setup, 5000















