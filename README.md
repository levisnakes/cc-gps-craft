# CC GPS Craft

ComputerCraft autopilot for a redstone-thruster craft (for example, Create Propulsion thrusters on a Create Aeronautics ship). It climbs on the bottom thruster, stops 10 blocks up to learn which way each side thruster pushes and tips the craft, climbs to cruise height, flies to a GPS target, then sinks onto it. A downward-facing Create Simulated Optical Sensor measures the distance to the ground, and the side outputs fire when it's within 3 blocks. Without an Optical Sensor, a redstone signal of strength 14 or more on top fires instead.

On a Sable ship (CC: Sable installed) it reads the ship's exact position, rotation and speed every tick through the `sublevel` API, so it needs no GPS and handles the ship turning mid-flight. Off a Sable ship it falls back to GPS, which needs a wireless modem and GPS hosts in range.

## Leveling

On a Sable ship it watches how far the craft is leaning and holds it level.

- **Vector thruster (best).** If a vector thruster is attached to the computer (touching it, or on a wired modem), it's found automatically. A few blocks off the ground it aims the nozzle each way to learn which way that tips the craft, then aims it every tick to keep the craft level. The side thrusters then only push it toward the target. The screen shows `Level: on (vector thruster)`.
- **Side thrusters.** Without a vector thruster, side thrusters mounted above or below the center of mass hold it level instead, and it steers by leaning like a drone (`Level: on (side thrusters)`). If they're level with the center of mass they can't tip it (`Level: OFF`).

Put the lift thruster as close to under the center of mass as you can, especially without a vector thruster. Without nozzle leveling, the lift cuts out past MAX_TILT (60°) instead of driving the craft into the ground. Side thrusters ease off as the craft leans and stop at SIDE_CUT_TILT (20°), until the craft is back under half that.

## Flight log

Every flight writes `craft.log` on the computer: the calibration results, then the height, lean, nozzle aim and every thruster output five times a second. To share it, run `pastebin put craft.log` and send the link it prints.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-gps-craft/main/startup.lua startup.lua
reboot
```

On every boot, `startup.lua` checks GitHub for a newer version, downloads it as `craft.lua`, and runs it. The version is shown in the screen title (`GPS Craft v1.4.0`). If it can't reach GitHub it runs the copy it already has.

To keep your own settings across updates, put them in `craft_settings.lua` (same lines as the settings at the top of the script, for example `CRUISE_Y = 150`). Updates never touch that file.

## Use

On boot it asks for the target coordinates. You can also run `craft <x> <y> <z>` (or `startup <x> <y> <z>` to update first). Hold Ctrl+T to abort; every output turns off.

Settings (cruise altitude, climb speed, detonation distance, payload outputs, controller tuning) are at the top of the file. Set CRAFT_WEIGHT_PN and LIFT_THRUST_PN from the craft; the hover power is worked out from them and fine-tuned during flight.
