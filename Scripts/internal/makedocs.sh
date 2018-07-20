#! /bin/bash

# Setup steps for local installation of DogBot control software including ROS layers

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Generating documentation with Doxygen"
cd $DIR/../../resources/Documentation
doxygen DogBotAPIdocs.doxygen
