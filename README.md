# CC GPS Craft

ComputerCraft autopilot for a redstone-thruster craft (for example, Create Propulsion thrusters on a Create Aeronautics ship). It climbs on the bottom thruster, stops 10 blocks up to learn which way each side thruster pushes and tips the craft, climbs to cruise height, flies to a GPS target, then sinks onto it. A downward-facing Create Simulated Optical Sensor measures the distance to the ground, and the side outputs fire when it's within 3 blocks. Without an Optical Sensor, a redstone signal of strength 14 or more on top fires instead.

On a Sable ship (CC: Sable installed) it reads the ship's exact position, rotation and speed every tick through the `sublevel` API, so it needs no GPS and handles the ship turning mid-flight. Off a Sable ship it falls back to GPS, which needs a wireless modem and GPS hosts in range.

## Install

Run these on the craft's computer:

```
wget https://raw.githubusercontent.com/levisnakes/cc-gps-craft/main/startup.lua startup.lua
reboot
```

If it says the file already exists, run `delete startup.lua` first.

On every boot, `startup.lua` checks GitHub for a newer version, downloads it as `craft.lua`, and runs it. The version is shown in the screen title (`GPS Craft v1.4.0`). If it can't reach GitHub it runs the copy it already has.

To keep your own settings across updates, put them in `craft_settings.lua` (same lines as the settings at the top of the script, for example `CRUISE_Y = 150`). Updates never touch that file.

## Leveling

On a Sable ship it watches how far the craft is leaning and holds it level.

- **Vector thruster (best).** If a vector thruster is attached to the computer (touching it, or on a wired modem), it's found automatically. A few blocks off the ground it aims the nozzle each way to learn which way that tips the craft, then aims it every tick to keep the craft level. The side thrusters then only push it toward the target. The screen shows `Level: on (vector thruster)`.
- **Side thrusters.** Without a vector thruster, side thrusters mounted above or below the center of mass hold it level instead, and it steers by leaning like a drone (`Level: on (side thrusters)`). If they're level with the center of mass they can't tip it (`Level: OFF`).

Put the lift thruster as close to under the center of mass as you can, especially without a vector thruster. Without nozzle leveling, the lift cuts out past MAX_TILT (60°) instead of driving the craft into the ground. Side thrusters ease off as the craft leans and stop at SIDE_CUT_TILT (20°), until the craft is back under half that.

## Website control

**https://levisnakes.github.io/cc-gps-craft/**

The craft's screen shows a website code (it's saved in `craft_remote_id`, so it stays the same). Type it into the website to see the craft on a map and send it commands: launch at coordinates (or click the map), change target, hold position, return to the pad, land where it is, arm/disarm, and cut all power. Return to pad and Land here always disarm first, so it sets down instead of firing. Tick **Demo** on the website to try it with a pretend craft.

Messages go through the free ntfy.sh relay. It allows about 250 messages a day per IP address, and every computer on a Minecraft server shares one, so the craft sends an update every TELEMETRY_SECONDS while flying, on changes otherwise, and stops at REMOTE_DAILY_LIMIT (200) a day. Receiving commands doesn't count. Anyone with the code can control the craft, so keep it private (delete `craft_remote_id` for a new one). Set `REMOTE = false` in craft_settings.lua to turn it off.

## Flight log

Every flight writes `craft.log` on the computer: the calibration results, then the height, lean, nozzle aim and every thruster output five times a second. To share it, run `pastebin put craft.log` and send the link it prints.

## Use

On boot it asks for the target coordinates. You can also run `craft <x> <y> <z>` (or `startup <x> <y> <z>` to update first). Hold Ctrl+T to abort; every output turns off.

Settings (cruise altitude, climb speed, detonation distance, payload outputs, controller tuning) are at the top of the file. Set CRAFT_WEIGHT_PN and LIFT_THRUST_PN from the craft; the hover power is worked out from them and fine-tuned during flight.
