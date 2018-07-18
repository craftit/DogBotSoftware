#! /bin/bash

# Build code and set up DogBot from initial git pull

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Building API..."
cd $DIR/../API
mkdir -p build
cd build
cmake ../src/
make -j
sudo make install 

echo "Building Python API..."
cd $DIR/../API/Python
mkdir -p build
cd build
cmake ..
make -j

echo "Building Data Recorder..."
cd $DIR/../Utilities/DataRecorder
mkdir -p build
cd build
cmake ../src/
make -j
sudo make install 
