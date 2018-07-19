#! /bin/bash

# Install Python locally.  TODO - this all needs a rework for Pypi later!
# note - installs via symlink back to the build directory

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Installing Python API on local system..."
cd $DIR/../API/Python
#cp build/pydogbotapi.py reactrobotics/pydogbotapi.py
#cp build/_pydogbotapi.so reactrobotics/_pydogbotapi.so

ln -sf $DIR/../API/Python/build/pydogbotapi.py $DIR/../API/Python/reactrobotics/pydogbotapi.py
ln -sf $DIR/../API/Python/build/_pydogbotapi.so $DIR/../API/Python/reactrobotics/_pydogbotapi.so

sudo pip install -e .


