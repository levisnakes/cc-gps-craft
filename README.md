# CC GPS Craft

ComputerCraft autopilot for a redstone-thruster craft (for example, Create Propulsion thrusters on a Create Aeronautics ship). It climbs on the bottom thruster, stops 10 blocks up to learn which way each side thruster pushes and tips the craft, climbs to cruise height, flies to a GPS target, then sinks onto it. A downward-facing Create Simulated Optical Sensor measures the distance to the ground, and the side outputs fire when it's within 3 blocks. Without an Optical Sensor, a redstone signal of strength 14 or more on top fires instead.

On a Sable ship (CC: Sable installed) it reads the ship's exact position, rotation and speed every tick through the `sublevel` API, so it needs no GPS and handles the ship turning mid-flight. Off a Sable ship it falls back to GPS, which needs a wireless modem and GPS hosts in range.

## Leveling

On a Sable ship it watches how far the craft is leaning. If the side thrusters tip the craft when they fire (they sit above or below its center of mass), it uses them to hold it level, and it steers by leaning, the way a drone does. The screen shows `Level: on` once that's working. `Level: OFF` means the side thrusters push the craft without tipping it, so nothing can correct a lean. Mount them a block or two above or below the center of mass. Either way, put the lift thruster right under the center of mass, because an off-center lift tips the craft before calibration can start. If it leans past MAX_TILT (60°) the lift cuts out instead of driving the craft into the ground.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-gps-craft/main/gps_craft.lua craft.lua
```

## Use

```
craft <x> <y> <z>
```

Or run `craft` and type the coordinates. Hold Ctrl+T to abort; every output turns off.

Settings (cruise altitude, climb speed, detonation distance, payload outputs, controller tuning) are at the top of the file. Set CRAFT_WEIGHT_PN and LIFT_THRUST_PN from the craft; the hover power is worked out from them and fine-tuned during flight.
