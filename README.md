
Brushless Motor Controller V2 firmware.  This is very much work in progress and not ready for use yet.

This is part of the dogbot project which can be found here: https://github.com/craftit/Dogbot


Useful tools:

stlink programming: https://github.com/texane/stlink
Bootloader:         https://www.feaser.com/openblt

To use USB you need access to the appropriate device:

sudo cp ./API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger