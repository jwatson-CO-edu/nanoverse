if [ -d "build" ]; then
    rm -rf build
fi
mkdir build
cd $_
cmake ..
make