if [ -d "build" ]; then
    rm -rf build
fi
mkdir build
cd $_
unbuffer cmake .. | tee CMakeOutput.txt
make
# make VERBOSE=1