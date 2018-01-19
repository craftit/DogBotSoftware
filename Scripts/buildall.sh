#! /bin/bash

# Build code and set up DogBot from initial git pull

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $DIR/../API
mkdir -p build
cd build
cmake ..
cmake --build .
sudo make install 

