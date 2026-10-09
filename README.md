# CC GPS Craft

ComputerCraft autopilot for a redstone-thruster craft (for example, Create Propulsion thrusters on a Create Aeronautics ship). It climbs on the bottom thruster, learns which way each side thruster pushes, flies level to a GPS target, descends, and fires a payload when a sensor on top outputs redstone strength 14.

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

Settings (cruise altitude, hover power, arming distance, payload outputs, controller tuning) are at the top of the file.
