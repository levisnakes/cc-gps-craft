-- GPS Craft launcher. On boot it checks GitHub for a newer craft.lua,
-- downloads it if there is one, then runs it.
--   startup              update, then ask for target coordinates
--   startup <x> <y> <z>  update, then fly straight there

local BASE = "https://raw.githubusercontent.com/levisnakes/cc-gps-craft/main/"
local FILE = "craft.lua"

local function localVersion()
  if not fs.exists(FILE) then return nil end
  local h = fs.open(FILE, "r")
  local code = h.readAll()
  h.close()
  return code:match('VERSION = "([^"]+)"')
end

-- The timestamp skips GitHub's cache, so a fresh push shows up right away.
local function fetch(name)
  if not http then return nil end
  local ok, res = pcall(http.get, BASE .. name .. "?t=" .. os.epoch("utc"))
  if not ok or not res then return nil end
  local body = res.readAll()
  res.close()
  return body
end

term.clear()
term.setCursorPos(1, 1)
local have = localVersion()
print("GPS Craft launcher  (installed: " .. (have and "v" .. have or "none") .. ")")

local latest = fetch("version.txt")
latest = latest and latest:match("%S+")
if not latest then
  print("Couldn't reach GitHub - running the installed version.")
elseif latest == have then
  print("Up to date.")
else
  print("Downloading v" .. latest .. "...")
  local code = fetch("gps_craft.lua")
  -- Only replace the old file with a complete download of the right version.
  if code and code:match('VERSION = "([^"]+)"') == latest then
    local h = fs.open(FILE .. ".new", "w")
    h.write(code)
    h.close()
    if fs.exists(FILE) then fs.delete(FILE) end
    fs.move(FILE .. ".new", FILE)
    print("Updated to v" .. latest .. ".")
  else
    print("Download failed - running the installed version.")
  end
end

if not fs.exists(FILE) then
  print("No craft.lua yet. Check the internet connection and reboot.")
  return
end
print("")
shell.run(FILE, ...)
