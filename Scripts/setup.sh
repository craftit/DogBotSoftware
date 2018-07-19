#! /bin/bash

# Setup steps for local installation of DogBot Firmware
# first argument is the default dog's name
# if second argument is 1, also build the code

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOGNAME="tango"

#set links to all the available config files
mkdir -p /home/$USER/.config/dogbot

cd $DIR/../Config
for f in *.json
do
  echo "linking configuration for $f to /home/$USER/.config/dogbot/$f"
  ln -sf $DIR/../Config/$f /home/$USER/.config/dogbot/$f
done

#check the default, passed as an argument
if [ $# -gt 0 ]; then
  DOGNAME=$1
fi

if [ "$DOGNAME" ]; then
  #copy the sample config (individual users will replace with their own dog-specific configs)
  echo "linking configuration for $DOGNAME as default"
  ln -sf $DIR/../Config/$DOGNAME.json /home/$USER/.config/dogbot/robot.json
fi

#set the rules for  accessing USB
sudo cp $DIR/../API/src/reactai.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

BUILDARG=""

if [ $# -gt 1 ]; then
  BUILDARG=$2
  if [ $BUILDARG -eq 1 ]; then
    echo "building project"
    $DIR/buildall.sh
    $DIR/pythonapi.sh
    
    echo "building ROS components"
    $DIR/rosbuild.sh
  fi
fi
  