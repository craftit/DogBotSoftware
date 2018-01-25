
# Brushless Motor Controller V2 Firmware

This repository contains firmware and a UI for interacting with the [DogBot] robotic hardware platform developed by [React AI]

Note: this code is still work-in-progress.

Corresponding higher-level ROS control software, along with CAD packages and other artifacts, can be found in the [Dogbot repo].  While that code requires the firmware to operate, the code in this repository is self-contained.

# Installation

## Pre-requisites
[Qt] is required to build and run the UI.

## Setup steps
To use USB you need access to the appropriate device:

```bash
sudo cp ./API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```
There is a [configuration file](./Config/configexample.json): to enable the ROS components in the [Dogbot repo] to access this it should be copied into ~/.config/DogBot

The code under [API](./API) should be built with `cmake --build` and `make install`. There is a script to wrap these steps at [buildall.sh](./Scripts/buildall.sh)

There is an installation script at [setup.sh](./Scripts/setup.sh), calling it as `setup.sh 1` also calls the build script.

# Operation

More details to follow

## Useful Tools
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
