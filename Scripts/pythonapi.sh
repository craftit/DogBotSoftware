#! /bin/bash

# Install Python locally.  TODO - this all needs a rework for Pypi later!

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Installing Python API on local system..."
cd $DIR/../API/Python
cp build/pydogbotapi.py reactrobotics/pydogbotapi.py
cp build/_pydogbotapi.so reactrobotics/_pydogbotapi.so
sudo pip install -e .


