#! /bin/bash

# Setup steps for local installation of DogBot control software including ROS layers

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Building ROS packages with catkin build"
cd $DIR/../ROS
catkin build --pre-clean
