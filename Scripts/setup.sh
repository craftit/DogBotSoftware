#! /bin/bash

# Setup steps for local installation of DogBot Firmware
# if first argument is 1, also build the code
# second argument is the dog's name

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOGNAME="tango"

if [ $# -gt 0 ]; then
  DOGNAME=$1
fi

#copy the sample config (individual users will replace with their own dog-specific configs)
echo "linking configuration for $DOGNAME to /home/$USER/.config/dogbot/"
mkdir -p /home/$USER/.config/dogbot
ln -sf $DIR/../Config/$DOGNAME.json /home/$USER/.config/dogbot/robot.json

#set the rules for  accessing USB
sudo cp $DIR/../API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

BUILDARG=""

if [ $# -gt 1 ]; then
  BUILDARG=$2
  if [ $BUILDARG -eq 1 ]; then
    echo "building project"
    $DIR/buildall.sh
    
    echo "building ROS components"
    $DIR/rosbuild.sh
  fi
fi
  