-- GPS Craft: flies a redstone-thruster craft to a target.
--
-- On a Sable ship (CC: Sable installed) it reads the ship's exact position,
-- rotation and speed every tick through the `sublevel` API. Otherwise it
-- falls back to the gps API, which needs a wireless modem and GPS hosts.
--
-- Usage:  gps_craft <x> <y> <z>      or just  gps_craft  and type them in.
--
-- Flight plan:
--   1. CLIMB      lift thruster (LIFT_SIDE) climbs to CRUISE_Y
--   2. CALIBRATE  pulses each side thruster once to learn which way it pushes
--   3. CRUISE     flies level to the target X/Z
--   4. DESCEND    sinks at DESCENT_SPEED onto the target, holding X/Z over it
-- During the descent, a downward-facing Create Simulated Optical Sensor watches
-- the distance to the ground. When it's within DETONATE_DISTANCE, the side
-- outputs fire. Hold Ctrl+T to abort; all outputs switch off.

-- ======================== SETTINGS ===========================

LIFT_SIDE = "bottom"
SENSOR_SIDE = "top"
THRUST_SIDES = { "front", "back", "left", "right" }

CRUISE_Y = 200          -- altitude to fly at
-- Read these off the craft (goggles on the ship and the lift thruster).
-- Hover power is worked out from them and fine-tuned in flight.
CRAFT_WEIGHT_PN = 55    -- total weight of the craft
LIFT_THRUST_PN = 133    -- lift thruster output at full power (redstone 15)
MAX_CLIMB = 6           -- max climb/sink speed while changing altitude (blocks per second)
ARRIVE_RADIUS = 3       -- blocks from target X/Z counted as "over the target"
DESCENT_SPEED = 5       -- blocks per second while dropping onto the target

-- Optical Sensor (laser pointing down), found automatically on the network or
-- touching the computer. It's only checked during the descent.
DETONATE_DISTANCE = 3   -- fire when the laser hits something this close (blocks)
-- Without an Optical Sensor peripheral, a redstone signal of at least this
-- strength on SENSOR_SIDE fires instead, or reaching the target Y.
TRIGGER_STRENGTH = 14

-- Payload outputs, fired together when the sensor triggers. By default these are
-- the side faces (the thrusters shut off first). To use dedicated outputs,
-- put a Redstone Relay on the network and set PAYLOAD_RELAY to its name.
PAYLOAD_SIDES = { "front", "back", "left", "right" }
PAYLOAD_RELAY = nil     -- e.g. "redstone_relay_0"
PAYLOAD_SECONDS = 2

-- Controller tuning. Raise the P values if it's sluggish, raise D if it overshoots.
ALT_P = 0.5             -- how hard it chases the target altitude
SPEED_GAIN = 2.0        -- lift change per block/s of vertical speed error
ALT_I = 0.4             -- how fast it learns the real hover power
MAX_SPEED = 20          -- top horizontal speed (blocks per second)
-- Braking uses this fraction of the side thrusters' measured strength, so it
-- starts slowing down early enough. Lower it if it still overshoots.
BRAKE_MARGIN = 0.5
-- Seconds a full-power side thruster gets to fix a speed error. Lower is
-- snappier, higher is gentler.
RESPONSE_TIME = 0.5
CAL_PULSE = 1.5         -- seconds per calibration pulse

-- ===================== END OF SETTINGS =======================

local ALL_SIDES = { "top", "bottom", "left", "right", "front", "back" }

local target = nil
local pos, vel = nil, { x = 0, y = 0, z = 0 }
local lastFix = 0
local altTarget = CRUISE_Y
local HOVER_POWER = 15 * CRAFT_WEIGHT_PN / LIFT_THRUST_PN
local hover = HOVER_POWER
local mode = "CLIMB"
local thrustDir = {}   -- side -> unit vector it pushes the craft (ship frame on Sable)
local sideAccel = {}   -- side -> blocks/s^2 at full power, from calibration
local orient = nil     -- ship rotation quaternion on Sable, nil with GPS
local navSource = "GPS"
local status = ""

local function now()
  return os.epoch("utc") / 1000
end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function allOff()
  for _, side in ipairs(ALL_SIDES) do rs.setAnalogOutput(side, 0) end
  if PAYLOAD_RELAY and peripheral.isPresent(PAYLOAD_RELAY) then
    for _, side in ipairs(ALL_SIDES) do
      pcall(peripheral.call, PAYLOAD_RELAY, "setAnalogOutput", side, 0)
    end
  end
end

local function setThrust(side, power)
  rs.setAnalogOutput(side, math.floor(clamp(power, 0, 15) + 0.5))
end

local function horizontalOff()
  for _, side in ipairs(THRUST_SIDES) do setThrust(side, 0) end
end

-- ---------- target input ----------

local function askNumber(label)
  while true do
    write(label .. ": ")
    local n = tonumber(read())
    if n then return n end
    print("Enter a number.")
  end
end

local function getTarget(args)
  local x, y, z = tonumber(args[1]), tonumber(args[2]), tonumber(args[3])
  if x and y and z then return { x = x, y = y, z = z } end
  print("Target coordinates")
  return { x = askNumber("X"), y = askNumber("Y"), z = askNumber("Z") }
end

-- ---------- loops ----------

-- Thrust needs some headroom over weight to climb and to correct drops.
local function liftWarning()
  local ratio = LIFT_THRUST_PN / CRAFT_WEIGHT_PN
  if ratio <= 1 then
    return "Lift thrust is not more than the weight - it can't take off."
  elseif ratio < 1.5 then
    return string.format("Lift is only %.1fx the weight: slow climb. 2x is better.", ratio)
  end
end

-- Rotate vector v by quaternion q (or by its inverse).
local function rotate(q, v, inverse)
  local qx, qy, qz, qw = q.x, q.y, q.z, q.w
  if inverse then qx, qy, qz = -qx, -qy, -qz end
  local tx = 2 * (qy * v.z - qz * v.y)
  local ty = 2 * (qz * v.x - qx * v.z)
  local tz = 2 * (qx * v.y - qy * v.x)
  return {
    x = v.x + qw * tx + (qy * tz - qz * ty),
    y = v.y + qw * ty + (qz * tx - qx * tz),
    z = v.z + qw * tz + (qx * ty - qy * tx),
  }
end

-- Thruster push direction in world space right now.
local function worldDir(side)
  local d = thrustDir[side]
  if not d then return nil end
  if orient then return rotate(orient, d) end
  return d
end

-- Reads the ship's pose every tick. Velocity comes from the change in
-- position (smoothed), which works whatever units Sable reports speed in.
local function sableLoop()
  while true do
    local ok, pose = pcall(sublevel.getLogicalPose)
    if ok and pose then
      local p, t = pose.position, now()
      if pos and t > lastFix then
        local dt = t - lastFix
        local a = 0.5
        vel = {
          x = vel.x * (1 - a) + (p.x - pos.x) / dt * a,
          y = vel.y * (1 - a) + (p.y - pos.y) / dt * a,
          z = vel.z * (1 - a) + (p.z - pos.z) / dt * a,
        }
      end
      pos, orient, lastFix = { x = p.x, y = p.y, z = p.z }, pose.orientation, t
      os.queueEvent("fix")
    else
      status = "Lost the ship pose - holding hover power"
      setThrust(LIFT_SIDE, hover)
    end
    sleep(0)
  end
end

-- Keeps pos/vel up to date from GPS and announces each fix with a "fix" event.
local function gpsLoop()
  while true do
    local x, y, z = gps.locate(0.5)
    if x then
      local t = now()
      if pos and t > lastFix then
        local dt = t - lastFix
        vel = { x = (x - pos.x) / dt, y = (y - pos.y) / dt, z = (z - pos.z) / dt }
      end
      pos, lastFix = { x = x, y = y, z = z }, t
      os.queueEvent("fix")
    else
      status = "NO GPS FIX - holding hover power"
      setThrust(LIFT_SIDE, hover)
      sleep(0.2)
    end
  end
end

-- Lift control: turns the altitude error into a capped climb/sink speed,
-- then drives the lift to hit that speed. `hover` slowly learns the real
-- power that holds the craft level, so a wrong HOVER_POWER corrects itself.
-- While descending it holds a steady sink rate, so it keeps going down
-- until the sensor fires, wherever the ground actually is.
local function altitudeLoop()
  local last = now()
  while true do
    os.pullEvent("fix")
    local t = now()
    local dt = math.min(t - last, 1)
    last = t

    local wantVy
    if mode == "DESCEND" then
      wantVy = -DESCENT_SPEED
    else
      wantVy = clamp(ALT_P * (altTarget - pos.y), -MAX_CLIMB, MAX_CLIMB)
    end
    local speedErr = wantVy - vel.y
    hover = clamp(hover + ALT_I * speedErr * dt, 0, 15)
    setThrust(LIFT_SIDE, hover + SPEED_GAIN * speedErr)
  end
end

local function waitFix()
  os.pullEvent("fix")
  return pos
end

local function waitSeconds(s)
  local untilT = now() + s
  while now() < untilT do waitFix() end
end

-- Steers toward (tx, tz). It picks the speed it wants from the distance
-- left, capped so it can always brake in time (v = sqrt(2 * a * d) using
-- the weakest thruster's measured strength), then fires each side thruster
-- in proportion to how much it helps close the gap to that speed.
local function steer(tx, tz)
  local ex, ez = tx - pos.x, tz - pos.z
  local dist = math.sqrt(ex * ex + ez * ez)

  local brake = math.huge
  for _, a in pairs(sideAccel) do brake = math.min(brake, a) end
  if brake == math.huge then brake = 1 end
  brake = brake * BRAKE_MARGIN

  local wantVx, wantVz = 0, 0
  if dist > 0.05 then
    local speed = math.min(MAX_SPEED, math.sqrt(2 * brake * dist))
    wantVx, wantVz = ex / dist * speed, ez / dist * speed
  end
  local errX, errZ = wantVx - vel.x, wantVz - vel.z

  for _, side in ipairs(THRUST_SIDES) do
    local d = worldDir(side)
    local push = 0
    if d and sideAccel[side] then
      push = (errX * d.x + errZ * d.z) / (sideAccel[side] * RESPONSE_TIME)
    end
    setThrust(side, push > 0 and push * 15 or 0)
  end
end

-- Pulses each side thruster and measures the change in velocity it caused,
-- so the craft's facing doesn't matter. Opposite sides are pulsed back to
-- back so the second pulse cancels the first one's drift.
local function calibrate()
  -- Pulses are only readable once the climb has settled.
  status = "Settling at cruise altitude"
  local calmSince = now()
  while now() - calmSince < 2 do
    waitFix()
    if math.abs(vel.y) > 1 or math.abs(pos.y - CRUISE_Y) > 3 then calmSince = now() end
  end
  for _, side in ipairs({ "front", "back", "left", "right" }) do
    status = "Calibrating " .. side
    waitSeconds(0.3)
    local v0 = { x = vel.x, z = vel.z }
    local t0 = now()
    setThrust(side, 15)
    waitSeconds(CAL_PULSE)
    setThrust(side, 0)
    local dx, dz = vel.x - v0.x, vel.z - v0.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len < 0.05 then
      error("Side '" .. side .. "' didn't move the craft during calibration", 0)
    end
    sideAccel[side] = len / (now() - t0)
    local dir = { x = dx / len, y = 0, z = dz / len }
    -- On Sable, store it in the ship's own frame so turning mid-flight is handled.
    thrustDir[side] = orient and rotate(orient, dir, true) or dir
  end
  -- Brake any leftover drift before cruising
  status = "Calibrating: settling"
  local hold = { x = pos.x, z = pos.z }
  local untilT = now() + 3
  while now() < untilT do
    steer(hold.x, hold.z)
    waitFix()
  end
end

local function horizontalDistance()
  local dx, dz = target.x - pos.x, target.z - pos.z
  return math.sqrt(dx * dx + dz * dz)
end

local function firePayload()
  mode = "PAYLOAD"
  status = "Sensor triggered - payload fired"
  horizontalOff()
  if PAYLOAD_RELAY then
    for _, side in ipairs(PAYLOAD_SIDES) do
      peripheral.call(PAYLOAD_RELAY, "setAnalogOutput", side, 15)
    end
  else
    for _, side in ipairs(PAYLOAD_SIDES) do rs.setAnalogOutput(side, 15) end
  end
  sleep(PAYLOAD_SECONDS)
end

local sensor = nil

-- Distance to whatever the laser hits, or nil if nothing is in range.
local function sensorDistance()
  if not sensor then return nil end
  local ok, hit = pcall(sensor.hasHit)
  if not ok or not hit then return nil end
  local ok2, d = pcall(sensor.getDistance)
  return ok2 and d or nil
end

local function shouldDetonate()
  if sensor then
    local d = sensorDistance()
    return d ~= nil and d <= DETONATE_DISTANCE
  end
  return rs.getAnalogInput(SENSOR_SIDE) >= TRIGGER_STRENGTH or pos.y <= target.y
end

-- Armed only during the descent, so nothing below the launch pad or the
-- flight path can set it off.
local function sensorLoop()
  while true do
    os.pullEvent()
    if mode == "DESCEND" and shouldDetonate() then
      firePayload()
      return
    end
  end
end

local function missionLoop()
  waitFix()

  mode = "CLIMB"
  altTarget = CRUISE_Y
  while pos.y < CRUISE_Y - 2 do
    status = string.format("Climbing to %d", CRUISE_Y)
    waitFix()
  end

  mode = "CALIBRATE"
  local ok, err = pcall(calibrate)
  if not ok then
    -- Don't drop out of the sky: hold altitude and report.
    horizontalOff()
    mode = "HOLD"
    status = tostring(err) .. ". Hovering - Ctrl+T to stop."
    while true do waitFix() end
  end

  mode = "CRUISE"
  while horizontalDistance() > ARRIVE_RADIUS do
    status = string.format("%.0f blocks to target", horizontalDistance())
    steer(target.x, target.z)
    waitFix()
  end

  mode = "DESCEND"
  while true do
    local d = sensorDistance()
    status = string.format("Dropping onto target, ground %s", d and string.format("%.1f blocks", d) or "not in range")
    steer(target.x, target.z)
    waitFix()
  end
end

local function screenLoop()
  while true do
    term.clear()
    term.setCursorPos(1, 1)
    print("GPS Craft  [" .. mode .. "]")
    print("")
    print(string.format("Target: %d %d %d", target.x, target.y, target.z))
    if pos then
      print(string.format("Pos:    %.1f %.1f %.1f", pos.x, pos.y, pos.z))
      print(string.format("Dist:   %.1f", horizontalDistance()))
    end
    if sensor then
      local d = sensorDistance()
      print(string.format("Sensor: %s  (fires at %d)", d and string.format("%.1f", d) or "--", DETONATE_DISTANCE))
    else
      print(string.format("Sensor: redstone %d  (fires at %d)", rs.getAnalogInput(SENSOR_SIDE), TRIGGER_STRENGTH))
    end
    print(string.format("Lift:   hover power %.1f   Nav: %s", hover, navSource))
    print("")
    print(status)
    print("")
    print("Ctrl+T to abort")
    sleep(0.25)
  end
end

-- ---------- main ----------

local args = { ... }
term.clear()
term.setCursorPos(1, 1)
local onSable = sublevel ~= nil and sublevel.isInPlotGrid()
if onSable then
  navSource = "Sable"
elseif not peripheral.find("modem", function(_, m) return m.isWireless() end) then
  error("Not on a Sable ship and no wireless modem for GPS", 0)
end
sensor = peripheral.find("optical_sensor")
if not sensor then
  print("No Optical Sensor found; using redstone on " .. SENSOR_SIDE .. " instead.")
end
target = getTarget(args)
local warn = liftWarning()
if warn then
  print(warn)
  if LIFT_THRUST_PN <= CRAFT_WEIGHT_PN then return end
  print("Press any key to launch anyway, Ctrl+T to cancel.")
  os.pullEvent("key")
end

allOff()
local navLoop = onSable and sableLoop or gpsLoop
local ok, err = pcall(parallel.waitForAny, navLoop, altitudeLoop, missionLoop, sensorLoop, screenLoop)
allOff()
term.clear()
term.setCursorPos(1, 1)
if mode == "PAYLOAD" then
  print("Payload fired.")
elseif not ok then
  print("Stopped: " .. tostring(err))
end
