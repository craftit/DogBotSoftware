#! /bin/bash

# Install Python locally.  TODO - this all needs a rework for Pypi later!

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Installing Python API on local system..."
cd $DIR/../API/Python
cp build/dogbot_api.py reactrobotics/dogbot_api.py
cp build/_dogbot_api.so reactrobotics/_dogbot_api.so
sudo pip install -e .


