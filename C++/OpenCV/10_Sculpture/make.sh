if [ -d "build" ]; then
    rm -rf build
fi
mkdir build
cd $_
unbuffer cmake .. | tee CMakeOutput.txt
make -j6
# make VERBOSE=1