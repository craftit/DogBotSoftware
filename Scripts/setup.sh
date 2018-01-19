#! /bin/bash

# Setup steps for local installation of DogBot Firmware
# if first argument is 1, also build the code

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#copy the sample config (individual users will replace with their own dog-specific configs)
echo "copying configuration to /home/$USER/.config/DogBot/"
mkdir -p /home/$USER/.config/DogBot
cp $DIR/../Config/configexample.json /home/$USER/.config/DogBot/

#set the rules for  accessing USB
sudo cp $DIR/../API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules

BUILDARG=""

if [ $# -gt 0 ]; then
  BUILDARG=$1
  if [ $BUILDARG -eq 1 ]; then
    echo "building project"
    $DIR/buildall.sh
  fi
fi
  