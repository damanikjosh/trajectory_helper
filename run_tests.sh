#!/bin/bash

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure with tests enabled
cmake -DBUILD_TESTS=ON ..

# Build
make

# Run tests
ctest --output-on-failure
