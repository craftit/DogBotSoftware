#! /bin/bash

# Setup steps for local installation of DogBot Firmware

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#copy the sample config (individual users will replace with their own dog-specific configs)
mkdir -p /home/$USER/.config/DogBot
cp $DIR/../Config/configexample.json /home/$USER/.config/DogBot/

#set the rules for  accessing USB
sudo cp $DIR/../API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
