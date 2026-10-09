# CC GPS Craft

ComputerCraft autopilot for a redstone-thruster craft (for example, Create Propulsion thrusters on a Create Aeronautics ship). It climbs on the bottom thruster, learns which way each side thruster pushes, flies level to a GPS target, then sinks onto it. A downward-facing Create Simulated Optical Sensor measures the distance to the ground, and the side outputs fire when it's within 3 blocks. Without an Optical Sensor, a redstone signal of strength 14 or more on top fires instead.

Needs a wireless modem on the craft and GPS hosts in range.

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
