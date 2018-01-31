
# Brushless Motor Controller V2 Firmware

This repository contains firmware and a UI for interacting with the [DogBot] robotic hardware platform developed by [React AI]

Note: this code is still work-in-progress.

Corresponding higher-level ROS control software, along with CAD packages and other artifacts, can be found in the [Dogbot repo].  While that code requires the firmware to operate, the code in this repository is self-contained.

# Installation

## Pre-requisites
[Qt] is required to build and run the UI, it was built and tested with Qt version 5.9.2 64-bit.

The software was developed and has been tested on Linux Ubuntu 16.04

## Setup steps
To use USB you need access to the appropriate device:

```bat
sudo cp ./API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```
There is a [configuration file](./Config/configexample.json): to enable the ROS components in the [Dogbot repo] to access this it should be copied into ~/.config/DogBot

The code under [API](./API) should be built with `cmake --build` and `make install`. There is a script to wrap these steps at [buildall.sh](./Scripts/buildall.sh)

There is an installation script at [setup.sh](./Scripts/setup.sh), calling it as `setup.sh 1` also calls the build script.

# Operation

Please refer to the product user-guide for warnings and safety considerations.

## Switch On

Use a benchtop power supply of between 14V and 30V

* Check cables are red-red throughout; check for loose wires, obstructions
* Connect power supply
* Switch on power supply.  Green light on each PCB should flash (and a steady red light in the middle, inside the housing)
* Connect the PCB via a USB cable to your computer

## Connecting to the Controllers

The UI can make a direct USB connection, or connect to a local instance of 'dogBotServer'.  The latter is recommended, as it allows for multiple local clients, e.g. multiple UIs, or UI plus a ROS client.

run the local server:
```bat
cd API/build/src
./dogBotServer
```
When you build and run the client in Qt, it will open to a connection screen:

* open Qt Creator
* open and run GUI/DogBotUI.pro
* pick 'local' as the connection type
* click 'Connect'

![connection tab screenshot](resources/UI_connect.png "Connection tab in UI")

On the Overview tab you should then see a summary row for each detected motor.

## Homing and Moving

To operate the motors via ROS, they must be homed, i.e. made aware of their absolute position.   This requires them to move past their end-stops, which are detected magnetically.

The 'Homed' column on the UI overview tab will probably say 'Measuring' in their pre-homed start-up state.

Homing the motors, method 1:
* turn all motors off on the Overview tab
* manually move joints past two sensor stops
* homing status of each should change to Homed

Method 2:
* note the ID for the required joint on the Overview tab
* move to the Motor Setup tab, pictured below, and select the motor by its ID
* Set the Dynamic Mode to Relative
* set the current limit to around 2.5A
* carefully move the motor by using the Position slider, backwards and forwards until it reaches its end-stop points.  The Homing status should change to Homed.

**If the motor oscillates** there is insufficient current for the control loop to complete, and you must increase the current limit.

![setup tab screenshot](resources/UI_motor_setup.png "Motor setup tab in UI")

The motor(s) may now be controlled via ROS if you wish, further details for that are in the [Dogbot repo]

# Useful Tools
* stlink programming: https://github.com/texane/stlink
* Bootloader:         https://www.feaser.com/openblt

# Contributing

If you wish to contribute please fork the project on GitHub. More details to follow

# License

Details to follow

[Dogbot repo]: https://github.com/craftit/Dogbot
[DogBot]: https://www.reactai.com/dog-bot/
[React AI]: https://www.reactai.com
[ROS]: http://www.ros.org
[Qt]: https://www.qt.io
