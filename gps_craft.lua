-- GPS Craft: flies a redstone-thruster craft to a target.
--
-- On a Sable ship (CC: Sable installed) it reads the ship's exact position,
-- rotation and speed every tick through the `sublevel` API. Otherwise it
-- falls back to the gps API, which needs a wireless modem and GPS hosts.
--
-- Usage:  craft <x> <y> <z>      or just  craft  and type them in.
-- startup.lua keeps this file up to date and runs it on boot.
--
-- Flight plan:
--   1. CLIMB      lift thruster (LIFT_SIDE) climbs CAL_HEIGHT above the pad
--   2. CALIBRATE  pulses each side thruster once to learn which way it pushes
--                 and which way it tips the craft
--   3. CLIMB      keeps climbing to CRUISE_Y, holding X/Z over the pad
--   4. CRUISE     flies to the target X/Z
--   5. DESCEND    sinks at DESCENT_SPEED onto the target, holding X/Z over it
-- During the descent, a downward-facing Create Simulated Optical Sensor watches
-- the distance to the ground. When it's within DETONATE_DISTANCE, the side
-- outputs fire. Hold Ctrl+T to abort; all outputs switch off.

VERSION = "1.5.0"  -- startup.lua compares this with version.txt on GitHub

-- ======================== SETTINGS ===========================
-- Updates replace this file. To keep your own values, put them in
-- craft_settings.lua instead (same lines, e.g.  CRUISE_Y = 150 ).

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
CAL_HEIGHT = 10         -- calibrate this far above the launch point

-- Leveling (Sable only). Side thrusters that tip the craft when they fire
-- (mounted above or below its center of mass) are used to hold it level.
LEVEL_P = 2.0           -- how hard it pulls back toward level
LEVEL_D = 2.0           -- damping on the lean; raise it if it wobbles
LEVEL_I = 0.5           -- how fast it learns a steady lean (lift off-center)
MAX_LEAN = 15           -- most it leans on purpose to steer (degrees)
LEAN_RESPONSE = 1.5     -- seconds the lean gets to fix a speed error
-- Past this lean (degrees) the lift shuts off, so a tipped craft doesn't
-- drive itself sideways or into the ground.
MAX_TILT = 60
-- Side thrusters that would tip the craft further ease off from half this
-- lean (degrees) and stop at it, until it's back under half.
SIDE_CUT_TILT = 20

-- ===================== END OF SETTINGS =======================

if fs and fs.exists("craft_settings.lua") then
  local f, err = loadfile("craft_settings.lua", nil, _ENV)
  if not f then error("craft_settings.lua: " .. err, 0) end
  f()
end

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
local up = { x = 0, y = 1, z = 0 }  -- craft's up direction in world space
local upRate = { x = 0, z = 0 }     -- how fast it's leaning (per second)
local sideTip = {}     -- side -> how it tips the craft at full power (ship frame)
local lever = nil      -- side push per unit of tip; set when all sides can level
local gravity = 9.81   -- blocks/s^2, read from aero.getGravity() on Sable
local navSource = "GPS"
local status = ""

local function now()
  return os.epoch("utc") / 1000
end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local nozzle = nil      -- vector thruster peripheral, if one is attached

local function allOff()
  for _, side in ipairs(ALL_SIDES) do rs.setAnalogOutput(side, 0) end
  if nozzle then pcall(nozzle.setVector, 0, 0) end
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
      -- CC: Sable hands back a quaternion object (v = x/y/z, a = w).
      local o = pose.orientation
      if o.v then
        orient = { x = o.v.x, y = o.v.y, z = o.v.z, w = o.a }
      else
        orient = { x = o.x, y = o.y, z = o.z, w = o.w }
      end
      local u = rotate(orient, { x = 0, y = 1, z = 0 })
      if pos and t > lastFix then
        local dt, a = t - lastFix, 0.5
        upRate = {
          x = upRate.x * (1 - a) + (u.x - up.x) / dt * a,
          z = upRate.z * (1 - a) + (u.z - up.z) / dt * a,
        }
      end
      pos, up, lastFix = { x = p.x, y = p.y, z = p.z }, u, t
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

local function waitFix()
  os.pullEvent("fix")
  return pos
end

local function waitSeconds(s)
  local untilT = now() + s
  while now() < untilT do waitFix() end
end

-- ---------- vector thruster leveling ----------
-- A vector thruster aims its nozzle up to 30 degrees two ways (setVector,
-- -1..1 each). Calibration learns how each aim tips the craft; after that
-- the nozzle holds the craft level every tick.

local nozzleTip = nil   -- { x = , y = } tip per unit of aim, ship frame
local nozzleTried = false
local nozzleStrength = 0  -- weakest tip at full aim
local nozzlePaused = false  -- centered while a side thruster is measured
local nozzleI = { x = 0, z = 0 }
local lastLevel = nil

local function aimNozzle(ax, ay)
  if nozzle then pcall(nozzle.setVector, clamp(ax, -1, 1), clamp(ay, -1, 1)) end
end

local function levelNozzle()
  local t = now()
  local dt = lastLevel and math.min(t - lastLevel, 0.5) or 0
  lastLevel = t
  local a, b = rotate(orient, nozzleTip.x), rotate(orient, nozzleTip.y)
  local strength = math.min(math.sqrt(a.x * a.x + a.z * a.z), math.sqrt(b.x * b.x + b.z * b.z))
  -- Same gain softening as the side thrusters: a MAX_LEAN error asks for
  -- half the nozzle's reach at most.
  local p = math.min(LEVEL_P, 0.5 * strength / math.sin(math.rad(MAX_LEAN)))
  local d = LEVEL_D * math.sqrt(p / LEVEL_P)
  -- The integral shrinks with the gains too; left at full strength it
  -- outruns the damping on a weak nozzle and the craft wobbles harder and harder.
  local ki = LEVEL_I * (p / LEVEL_P) ^ 1.5
  nozzleI.x = clamp(nozzleI.x - ki * up.x * dt, -strength, strength)
  nozzleI.z = clamp(nozzleI.z - ki * up.z * dt, -strength, strength)
  local wantX = -p * up.x - d * upRate.x + nozzleI.x
  local wantZ = -p * up.z - d * upRate.z + nozzleI.z
  -- Mix the two aim axes to get that tip.
  local det = a.x * b.z - a.z * b.x
  if math.abs(det) < 1e-9 then return end
  aimNozzle((wantX * b.z - wantZ * b.x) / det, (a.x * wantZ - a.z * wantX) / det)
end

-- Tips the nozzle each way and measures how fast the craft starts to lean.
-- Each axis goes + then -, so the second pulse undoes the first.
local function calibrateNozzle()
  nozzleTried = true
  local tips = {}
  for _, axis in ipairs({ "x", "y" }) do
    local sumX, sumZ = 0, 0
    for _, sign in ipairs({ 1, -1 }) do
      status = "Calibrating vector thruster " .. axis
      aimNozzle(0, 0)
      local b0, bt = { x = upRate.x, z = upRate.z }, now()
      waitSeconds(0.3)
      local base = now() - bt
      local driftX, driftZ = (upRate.x - b0.x) / base, (upRate.z - b0.z) / base
      local r0, u0, t0 = { x = upRate.x, z = upRate.z }, { x = up.x, z = up.z }, now()
      local aim = 0.25 * sign
      if axis == "x" then aimNozzle(aim, 0) else aimNozzle(0, aim) end
      while now() - t0 < CAL_PULSE
          and (now() - t0 < 0.3 or (up.x - u0.x) ^ 2 + (up.z - u0.z) ^ 2 < 0.0075) do
        waitFix()
      end
      aimNozzle(0, 0)
      local dt = now() - t0
      while now() - t0 < dt + 0.3 do waitFix() end  -- nozzle swinging back
      local span = now() - t0
      sumX = sumX + (upRate.x - r0.x - driftX * span) / (dt * aim)
      sumZ = sumZ + (upRate.z - r0.z - driftZ * span) / (dt * aim)
    end
    tips[axis] = { x = sumX / 2, z = sumZ / 2 }
  end
  local function strong(t) return t.x * t.x + t.z * t.z > 0.003 * 0.003 end
  if strong(tips.x) and strong(tips.y) then
    nozzleStrength = math.min(math.sqrt(tips.x.x ^ 2 + tips.x.z ^ 2), math.sqrt(tips.y.x ^ 2 + tips.y.z ^ 2))
    nozzleTip = {
      x = rotate(orient, { x = tips.x.x, y = 0, z = tips.x.z }, true),
      y = rotate(orient, { x = tips.y.x, y = 0, z = tips.y.z }, true),
    }
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
    -- The nozzle needs the lift running to level the craft, so with one
    -- it only cuts out when upside down.
    local cutoff = nozzleTip and 0 or math.cos(math.rad(MAX_TILT))
    if up.y < cutoff then
      -- Tipped over: the lift would only push sideways or down.
      setThrust(LIFT_SIDE, 0)
    else
      -- Only learn hover power while nearly level, and push harder when
      -- leaning since only part of the lift points up.
      if up.y > 0.95 then hover = clamp(hover + ALT_I * speedErr * dt, 0, 15) end
      setThrust(LIFT_SIDE, (hover + SPEED_GAIN * speedErr) / math.max(up.y, 0.5))
    end
    if nozzleTip and orient and not nozzlePaused then levelNozzle() end
  end
end

local leanI = { x = 0, z = 0 }
local lastSteer = nil

-- Weakest tipping strength among the measured sides.
local function tipStrength()
  local s = math.huge
  for _, tip in pairs(sideTip) do
    s = math.min(s, math.sqrt(tip.x * tip.x + tip.y * tip.y + tip.z * tip.z))
  end
  return s
end

-- Leveling gains, softened for weak side thrusters so they don't max out
-- and overshoot: a MAX_LEAN error asks for half their strength at most.
local function levelGains()
  local p = math.min(LEVEL_P, 0.5 * tipStrength() / math.sin(math.rad(MAX_LEAN)))
  return p, LEVEL_D * math.sqrt(p / LEVEL_P)
end

-- Fires the side thrusters. Sides known to tip the craft hold its lean at
-- (wx, wz) (0, 0 is level); the rest push to fix the speed error.
local sidesCut = false

local function driveSides(wx, wz, errX, errZ)
  -- How much of their push the sides that tip the craft further may use.
  local lean = 1
  if orient then
    local tilt = math.deg(math.acos(clamp(up.y, -1, 1)))
    if tilt > SIDE_CUT_TILT then sidesCut = true
    elseif tilt < SIDE_CUT_TILT / 2 then sidesCut = false end
    lean = sidesCut and 0 or clamp((SIDE_CUT_TILT - tilt) / (SIDE_CUT_TILT / 2), 0, 1)
  end
  local p, d = levelGains()
  local levX = p * (wx - up.x) - d * upRate.x + leanI.x
  local levZ = p * (wz - up.z) - d * upRate.z + leanI.z
  for _, side in ipairs(THRUST_SIDES) do
    local d = worldDir(side)
    local tip = orient and sideTip[side] and rotate(orient, sideTip[side])
    local push = 0
    if tip and not nozzleTip then
      push = (levX * tip.x + levZ * tip.z) / (tip.x * tip.x + tip.z * tip.z)
    elseif d and sideAccel[side] then
      push = (errX * d.x + errZ * d.z) / (sideAccel[side] * RESPONSE_TIME)
      -- With nozzle leveling, keep each side from tipping the craft harder
      -- than half of what the nozzle can correct.
      if tip and nozzleTip then
        push = math.min(push, 0.5 * nozzleStrength / math.sqrt(tip.x * tip.x + tip.z * tip.z))
      end
      -- Unless it's known to tip the craft back toward level, back it off.
      if not (tip and tip.x * up.x + tip.z * up.z < 0) then push = push * lean end
    end
    setThrust(side, push > 0 and push * 15 or 0)
  end
end

-- Steers toward (tx, tz). It picks the speed it wants from the distance
-- left, capped so it can always brake in time (v = sqrt(2 * a * d)).
-- Without leveling it fires each side thruster in proportion to how much it
-- helps close the gap to that speed. With leveling it flies like a drone:
-- it picks a lean that points the lift where it wants to go, and the side
-- thrusters tip the craft to that lean.
local function steer(tx, tz)
  local ex, ez = tx - pos.x, tz - pos.z
  local dist = math.sqrt(ex * ex + ez * ez)
  local leveling = lever ~= nil and orient ~= nil
  local t = now()
  local dt = lastSteer and math.min(t - lastSteer, 0.5) or 0
  lastSteer = t

  -- The lean can't steer faster than the leveling can tip the craft.
  local response = math.max(LEAN_RESPONSE, 3 / math.sqrt(levelGains()))
  local brake = math.huge
  if leveling then
    brake = gravity * math.sin(math.rad(MAX_LEAN))
  else
    for _, a in pairs(sideAccel) do brake = math.min(brake, a) end
    if brake == math.huge then brake = 1 end
  end
  brake = brake * BRAKE_MARGIN

  local wantVx, wantVz = 0, 0
  if dist > 0.05 then
    local speed = math.min(MAX_SPEED, math.sqrt(2 * brake * dist))
    wantVx, wantVz = ex / dist * speed, ez / dist * speed
  end
  local errX, errZ = wantVx - vel.x, wantVz - vel.z

  local wx, wz = 0, 0
  if leveling then
    -- Tipping the craft also shoves it sideways, so take that shove out
    -- of the speed the lean is steering.
    local ax = (wantVx - (vel.x - lever * upRate.x)) / response
    local az = (wantVz - (vel.z - lever * upRate.z)) / response
    wx, wz = ax / gravity, az / gravity
    local m, cap = math.sqrt(wx * wx + wz * wz), math.sin(math.rad(MAX_LEAN))
    if m > cap then wx, wz = wx / m * cap, wz / m * cap end
    -- Integral holds the lean against a steady push (lift off-center).
    local limit = tipStrength()
    local ki = LEVEL_I * (levelGains() / LEVEL_P) ^ 1.5  -- shrinks with the gains, as above
    leanI.x = clamp(leanI.x + ki * (wx - up.x) * dt, -limit, limit)
    leanI.z = clamp(leanI.z + ki * (wz - up.z) * dt, -limit, limit)
  end
  driveSides(wx, wz, errX, errZ)
end

-- Pulses each side thruster and measures the change in velocity it caused,
-- so the craft's facing doesn't matter. Opposite sides are pulsed back to
-- back so the second pulse cancels the first one's drift.
local function calibrate()
  -- Pulses are only readable once the climb has settled.
  if nozzle and orient and not nozzleTried then calibrateNozzle() end
  status = "Settling"
  -- With nozzle leveling, also wait for it to come level (gives up after 15 s).
  local calmSince, giveUp = now(), now() + 15
  while now() - calmSince < 2 do
    waitFix()
    if math.abs(vel.y) > 1 or math.abs(pos.y - altTarget) > 3
        or (nozzleTip and up.y < 0.9986 and now() < giveUp) then
      calmSince = now()
    end
  end
  for i, side in ipairs({ "front", "back", "left", "right" }) do
    status = "Calibrating " .. side
    -- Let the nozzle bring it level first (up to 8 s), since it's paused
    -- while the side is measured.
    if nozzleTip then
      horizontalOff()
      local untilT = now() + 8
      while now() < untilT and (up.y < 0.9962 or upRate.x ^ 2 + upRate.z ^ 2 > 0.0004) do
        waitFix()
      end
    end
    -- Drift with nothing firing (drag, lean), so it can be taken out of the
    -- pulse. The nozzle is centered too, or it would hide the side's tip.
    horizontalOff()
    nozzlePaused = true
    aimNozzle(0, 0)
    local b0, bv, bt = { x = upRate.x, z = upRate.z }, { x = vel.x, z = vel.z }, now()
    waitSeconds(0.5)
    local base = now() - bt
    local driftX, driftZ = (upRate.x - b0.x) / base, (upRate.z - b0.z) / base
    local dragX, dragZ = (vel.x - bv.x) / base, (vel.z - bv.z) / base
    local v0, r0 = { x = vel.x, z = vel.z }, { x = upRate.x, z = upRate.z }
    local t0, last = now(), now()
    local u0 = { x = up.x, z = up.z }
    local liftX, liftZ = 0, 0  -- push from the lift while the craft leans
    local function track()
      waitFix()
      local t = now()
      liftX, liftZ = liftX + gravity * up.x * (t - last), liftZ + gravity * up.z * (t - last)
      last = t
    end
    setThrust(side, 15)
    -- Cut the pulse short once it has tipped the craft about 5 degrees.
    while now() - t0 < CAL_PULSE
        and (now() - t0 < 0.3 or (up.x - u0.x) ^ 2 + (up.z - u0.z) ^ 2 < 0.0075) do
      track()
    end
    setThrust(side, 0)
    local dt = now() - t0
    -- Keep measuring while the thruster spins down; its ramp-up and
    -- ramp-down lag roughly cancel, so the pulse counts as dt at full power.
    while now() - t0 < dt + 0.3 do track() end
    local span = now() - t0
    nozzlePaused = false
    local dx = vel.x - v0.x - liftX - dragX * span
    local dz = vel.z - v0.z - liftZ - dragZ * span
    local len = math.sqrt(dx * dx + dz * dz)
    if len < 0.05 then
      error("Side '" .. side .. "' didn't move the craft during calibration", 0)
    end
    sideAccel[side] = len / dt
    local dir = { x = dx / len, y = 0, z = dz / len }
    -- On Sable, store it in the ship's own frame so turning mid-flight is handled.
    thrustDir[side] = orient and rotate(orient, dir, true) or dir
    if orient then
      local tx = (upRate.x - r0.x - driftX * span) / dt
      local tz = (upRate.z - r0.z - driftZ * span) / dt
      if tx * tx + tz * tz > 0.003 * 0.003 then
        sideTip[side] = rotate(orient, { x = tx, y = 0, z = tz }, true)
      end
      -- After each pair, level out with the sides measured so far.
      if i % 2 == 0 and not nozzleTip then
        status = "Calibrating: leveling"
        local untilT = now() + 2
        while now() < untilT do
          driveSides(0, 0, 0, 0)
          waitFix()
        end
      end
    end
  end
  -- Leveling needs every side, so it can tip the craft every way.
  local sum, n = 0, 0
  for side, tip in pairs(sideTip) do
    local d = thrustDir[side]
    local len = math.sqrt(tip.x * tip.x + tip.y * tip.y + tip.z * tip.z)
    local sign = (d.x * tip.x + d.y * tip.y + d.z * tip.z) >= 0 and 1 or -1
    sum, n = sum + sign * sideAccel[side] / len, n + 1
  end
  if n == #THRUST_SIDES and not nozzleTip then lever = sum / n end
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
  local pad = { x = pos.x, z = pos.z }
  local padY = pos.y
  altTarget = math.min(pos.y + CAL_HEIGHT, CRUISE_Y)
  while pos.y < altTarget - 2 do
    status = string.format("Climbing to %.0f to calibrate", altTarget)
    -- Calibrate the vector thruster as soon as it's clear of the ground,
    -- so it's leveling before an off-center lift can tip the craft far.
    if nozzle and orient and not nozzleTried and pos.y > padY + 3 then
      calibrateNozzle()
    end
    waitFix()
  end

  mode = "CALIBRATE"
  local ok, err = pcall(calibrate)
  if not ok then
    -- Don't drop out of the sky: hold altitude and report.
    horizontalOff()
    mode = "HOLD"
    status = tostring(err) .. ". Hovering - Ctrl+T to stop."
    local hold = { x = pos.x, z = pos.z }
    while true do
      steer(hold.x, hold.z)
      waitFix()
    end
  end

  mode = "CLIMB"
  altTarget = CRUISE_Y
  while pos.y < CRUISE_Y - 2 do
    status = string.format("Climbing to %d", CRUISE_Y)
    steer(pad.x, pad.z)
    waitFix()
  end

  mode = "CRUISE"
  -- A craft steering by leaning can end up circling just outside the radius,
  -- so close enough for 10 seconds also counts.
  local nearSince = nil
  while horizontalDistance() > ARRIVE_RADIUS do
    if horizontalDistance() < ARRIVE_RADIUS * 4 then
      nearSince = nearSince or now()
      if now() - nearSince > 10 then break end
    else
      nearSince = nil
    end
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
    print("GPS Craft v" .. VERSION .. "  [" .. mode .. "]")
    print("")
    print(string.format("Target: %.0f %.0f %.0f", target.x, target.y, target.z))
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
    if orient then
      local tilt = math.deg(math.acos(clamp(up.y, -1, 1)))
      local n = 0
      for _ in pairs(sideAccel) do n = n + 1 end
      local lv = nozzleTip and "on (vector thruster)" or lever and "on (side thrusters)"
        or n < #THRUST_SIDES and (nozzle and "vector thruster, after calibration" or "after calibration")
        or nozzle and "OFF - aiming the vector thruster didn't tip it"
        or "OFF - side thrusters don't tip it"
      local cut = nozzleTip and up.y < 0 or not nozzleTip and tilt > MAX_TILT
      print(string.format("Tilt:   %.0f deg%s", tilt, cut and " - TIPPED, lift off" or ""))
      print("Level:  " .. lv)
    end
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
  if aero then
    local ok, g = pcall(aero.getGravity)
    local m = ok and type(g) == "table" and g.length and g:length()
    if m and m > 0.5 and m < 100 then gravity = m end
  end
elseif not peripheral.find("modem", function(_, m) return m.isWireless() end) then
  error("Not on a Sable ship and no wireless modem for GPS", 0)
end
for _, t in ipairs({ "vector_thruster", "creative_vector_thruster", "liquid_vector_thruster" }) do
  nozzle = nozzle or peripheral.find(t)
end
if nozzle and not onSable then
  print("Vector thruster found, but aiming it needs CC: Sable; it stays centered.")
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
