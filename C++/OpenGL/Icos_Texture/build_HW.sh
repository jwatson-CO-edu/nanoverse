#!/bin/bash

if [ ! -d build ]; then
	echo "bash: 'build' directory not found, creating ..."
	mkdir build
else 
	echo "bash: 'build' directory found!"
fi
cd build
echo "bash: About to cmake ..."
cmake ../
echo ""
echo "bash: About to build ..."
make -j4
echo ""
echo "bash: Build process has finished!"
cd ..
