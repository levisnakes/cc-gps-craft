-- GPS Craft launcher. On boot it checks GitHub for a newer craft.lua,
-- downloads it if there is one, then runs it.
--   startup              update, then ask for target coordinates
--   startup <x> <y> <z>  update, then fly straight there

local REPO = "levisnakes/cc-gps-craft"
local FILE = "craft.lua"

local function fetch(url)
  if not http then return nil end
  local ok, res = pcall(http.get, url)
  if not ok or not res then return nil end
  local body = res.readAll()
  res.close()
  return body
end

local function readFile(path)
  if not fs.exists(path) then return nil end
  local h = fs.open(path, "r")
  local s = h.readAll()
  h.close()
  return s
end

local function writeFile(path, s)
  local h = fs.open(path .. ".new", "w")
  h.write(s)
  h.close()
  if fs.exists(path) then fs.delete(path) end
  fs.move(path .. ".new", path)
end

term.clear()
term.setCursorPos(1, 1)
local code = readFile(FILE)
local have = code and code:match('VERSION = "([^"]+)"')
print("GPS Craft launcher  (installed: " .. (have and "v" .. have or "none") .. ")")

-- Ask the API for the newest commit and download from it: plain
-- raw.githubusercontent.com/.../main/ links can be up to 5 minutes stale.
local api = fetch("https://api.github.com/repos/" .. REPO .. "/commits/main")
local sha = api and api:match('"sha"%s*:%s*"(%x+)"')
local base = "https://raw.githubusercontent.com/" .. REPO .. "/" .. (sha or "main") .. "/"

-- Keep this launcher up to date too.
local me = shell.getRunningProgram()
local newMe = fetch(base .. "startup.lua")
if newMe and newMe:find("GPS Craft launcher", 1, true) and newMe ~= readFile(me)
    and not _G.gpsCraftLauncherUpdated then
  writeFile(me, newMe)
  print("Launcher updated - restarting it.")
  _G.gpsCraftLauncherUpdated = true  -- only once, so it can't loop
  return shell.run(me, ...)
end

local latest = fetch(base .. "version.txt")
latest = latest and latest:match("%S+")
if not latest then
  print("Couldn't reach GitHub - running the installed version.")
elseif latest == have then
  print("Up to date.")
else
  print("Downloading v" .. latest .. "...")
  local new = fetch(base .. "gps_craft.lua")
  -- Only replace the old file with a complete download of the right version.
  if new and new:match('VERSION = "([^"]+)"') == latest then
    writeFile(FILE, new)
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
