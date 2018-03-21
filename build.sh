#!/bin/bash
cd `dirname $0`

# Compile code.
mkdir -p build
cd build
cmake ..
make -j `nproc` $*
