-- Install on the flight controller SD card as:
--   APM/scripts/jetson_nogps_status.lua
-- Uses SCR_USER1 for the Jetson state code and SCR_USER2 for the target EKF source set.

local STATUS_PARAM = "SCR_USER1"
local SOURCE_SET_PARAM = "SCR_USER2"
local UPDATE_MS = 1000
local WAITING_REPORT_INTERVAL_TICKS = 10

local STATE_NONE = 0
local STATE_JETSON_BOOT = 10
local STATE_GPS_ASSIST_ACTIVE = 20
local STATE_ORIGIN_LOCKED = 30
local STATE_FLOW_STARTED = 40
local STATE_SOURCE_SET_ACTIVE = 50
local STATE_GPS_SOURCE_ACTIVE = 51
local STATE_NO_GPS_SOURCE_ACTIVE = 52
local STATE_GPS_LESS_FLIGHT_ACTIVE = 53
local STATE_MANUAL_ORIGIN = 60
local STATE_CONFIG_FAILED = 80
local STATE_GPS_TIMEOUT = 81
local STATE_SOURCE_SWITCH_FAILED = 82
local STATE_SOURCE_SWITCH_NO_ACK = 83

local last_state = nil
local last_source_set = nil
local waiting_ticks = 0
local gps_less_ticks = 0
local last_home_is_set = nil

local function round_param(value)
    if value == nil then
        return 0
    end
    return math.floor(value + 0.5)
end

local function get_primary_gps_instance()
    local primary = gps:primary_sensor()
    if primary == nil then
        return 0
    end
    return primary
end

local function gps_summary_text()
    local instance = get_primary_gps_instance()
    local fix = gps:status(instance) or 0
    local sats = gps:num_sats(instance) or 0
    return string.format("GPS waiting for home fix=%d sats=%d", fix, sats)
end

local function send_notice(text)
    gcs:send_text(5, text)
end

local function send_warning(text)
    gcs:send_text(4, text)
end

local function relay_state_change(state_code, source_set_id)
    if state_code == STATE_JETSON_BOOT then
        send_notice("Jetson no-GPS bridge detected")
        return
    end

    if state_code == STATE_GPS_ASSIST_ACTIVE then
        send_notice("GPS assist active, locking home")
        return
    end

    if state_code == STATE_ORIGIN_LOCKED then
        send_notice("Home/origin locked from GPS")
        return
    end

    if state_code == STATE_FLOW_STARTED then
        send_notice("No-GPS flow bridge started")
        return
    end

    if state_code == STATE_SOURCE_SET_ACTIVE then
        if source_set_id > 0 then
            send_notice(string.format("No-GPS EKF source set %d active", source_set_id))
        else
            send_notice("No-GPS EKF source active")
        end
        return
    end

    if state_code == STATE_GPS_SOURCE_ACTIVE then
        if source_set_id > 0 then
            send_notice(string.format("GPS EKF source set %d active", source_set_id))
        else
            send_notice("GPS EKF source active")
        end
        return
    end

    if state_code == STATE_NO_GPS_SOURCE_ACTIVE then
        if source_set_id > 0 then
            send_notice(string.format("No-GPS EKF source set %d active", source_set_id))
        else
            send_notice("No-GPS EKF source active")
        end
        return
    end

    if state_code == STATE_GPS_LESS_FLIGHT_ACTIVE then
        send_notice("GPS-Less flight active")
        return
    end

    if state_code == STATE_MANUAL_ORIGIN then
        send_notice("Manual origin set, flow source pending")
        return
    end

    if state_code == STATE_CONFIG_FAILED then
        send_warning("Jetson no-GPS config failed")
        return
    end

    if state_code == STATE_GPS_TIMEOUT then
        send_warning("Jetson GPS home lock failed")
        return
    end

    if state_code == STATE_SOURCE_SWITCH_FAILED then
        send_warning("EKF source switch failed")
        return
    end

    if state_code == STATE_SOURCE_SWITCH_NO_ACK then
        send_warning("EKF source switch no ack")
        return
    end
end

local function update()
    local current_state = round_param(param:get(STATUS_PARAM))
    local current_source_set = round_param(param:get(SOURCE_SET_PARAM))
    local home_is_set = ahrs:home_is_set()

    if last_state == nil then
        last_state = current_state
        last_source_set = current_source_set
        last_home_is_set = home_is_set
        send_notice("Jetson no-GPS relay Lua loaded")
        return update, UPDATE_MS
    end

    if current_state ~= last_state then
        relay_state_change(current_state, current_source_set)
        last_state = current_state
        last_source_set = current_source_set
        waiting_ticks = 0
        gps_less_ticks = 0
    elseif current_source_set ~= last_source_set then
        last_source_set = current_source_set
    end

    if current_state == STATE_GPS_ASSIST_ACTIVE and not home_is_set then
        waiting_ticks = waiting_ticks + 1
        if waiting_ticks >= WAITING_REPORT_INTERVAL_TICKS then
            send_notice(gps_summary_text())
            waiting_ticks = 0
        end
    else
        waiting_ticks = 0
    end

    if current_state == STATE_GPS_LESS_FLIGHT_ACTIVE then
        gps_less_ticks = gps_less_ticks + 1
        if gps_less_ticks >= 6 then
            send_notice("GPS-Less flight active")
            gps_less_ticks = 0
        end
    else
        gps_less_ticks = 0
    end

    if last_home_is_set ~= nil and home_is_set ~= last_home_is_set then
        if home_is_set then
            send_notice("FC home position is set")
        else
            send_warning("FC home position cleared")
        end
        last_home_is_set = home_is_set
    end

    return update, UPDATE_MS
end

return update, 2000
